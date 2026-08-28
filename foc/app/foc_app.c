/*******************************************************************************
 * @file    foc_app.c
 * @brief   FOC 应用层 — MODUS 挂载实现
 *  - foc_app_RunFSM() 只做产品事件处理并驱动 motor_RunFSM()；
 *  - 高频算法由 ADC 抢占转换完成中断（TMR1 CH4 触发，20 kHz）通过
 *    foc_app_HighFrequencyISR() 调 motor_HighFrequencyStep()；
 *  - 低频级联由 MODUS 1 ms 系统时钟（foc_app_Clock，SysTick 调度）调
 *    motor_LowFrequencyStep()；
 *  - 按钮与 Shell 只使用 motor_Start()/motor_Stop()/引用 setter 发命令，
 *    决策与打印只使用 motor_GetSnapshot()；事件环仅用于日志。
 ******************************************************************************/

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "peripheral.h"
#include "mdi_hw.h"
#include "foc_hal_mdi_adapter.h"
#include "foc_app.h"
#include "motor.h"
#include "motor_control.h"
#include "foc_pid.h"
#include "foc_controller.h"
#include "foc_smo.h"
#include "foc_nlfo.h"
#include "mdebug/mshell.h"
#include "perfc_port.h"
#include "motor/motor_private.h"

#if defined(MDI_HW_HAS_I2C_ENCODER)
#include "foc_encoder.h"
#include "as5600.h"
#endif


/* 控制周期（归一化秒）：
 * 高频 — TMR1 CH4 在中心对齐 TWO_WAY_1 模式下仅向下计数时产生一次比较
 *        事件，ADC 抢占转换完成中断每 20 kHz PWM 载波触发一次（50 us）；
 * 低频 — MODUS 1 ms 系统时钟（1 kHz）。 */
#define FOC_APP_HF_PERIOD_S     0.00005f
#define FOC_APP_LF_PERIOD_S     0.001f

/* 开环产品默认值（2205 云台电机，参考 STOPLL_FOC_2205）：
 * 1 电角 turn/s，5 turn/s^2 加速，Vq 0.03 pu（12V 母线下约 0.36V） */
#define FOC_APP_OPEN_LOOP_SPEED 1.0f
#define FOC_APP_OPEN_LOOP_ACCEL 5.0f
#define FOC_APP_VOLTAGE_REF_Q   0.03f

static uint32_t  foc_app_GetMilliseconds(void *pContext);
static uintptr_t foc_app_EnterCritical(void *pContext);
static void      foc_app_ExitCritical(void *pContext, uintptr_t wState);

mcoroutine_handle_t tMcoroutineFocAppHandle = {
    .bIsRunning = false,
    .pfcn       = NULL,
};

static modus_base_t     s_tFocAppBase;

static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run  (uintptr_t wObjectAddr);

static motor_config_t s_tMotorConfig = {
    .tParams = {
        .wResistanceMilliOhm          = 2500U,
        .wLdMicroHenry                = 500U,
        .wLqMicroHenry                = 500U,
        .wBackEmfMicroVoltPerRadSec   = 23000U,
        .wInertiaNanoKgM2             = 100000U,
        .wRatedVoltageMilliVolt       = 12000U,
        .wRatedCurrentMilliAmp        = 800U,
        .chPolePairs                  = 7U,
    },
    .eTopology = SENSING_TOPOLOGY_3P,
    .tTime = {
        .pTimeContext = NULL,
        .fnGetMilliseconds = foc_app_GetMilliseconds,
    },
    .tSync = {
        .pSyncContext = NULL,
        .fnEnter = foc_app_EnterCritical,
        .fnExit  = foc_app_ExitCritical,
    },
    .qHighFrequencyPeriod = FOC_SCALAR(FOC_APP_HF_PERIOD_S),
    .qLowFrequencyPeriod  = FOC_SCALAR(FOC_APP_LF_PERIOD_S),
    .tPosition = {
        .chPolePairs = 7U,
        .chDirection = 1,
    },
    .qTransitionMinimumConfidence = FOC_SCALAR(0.8f),
    .qTransitionMinimumSpeed      = FOC_SCALAR(0.05f),
    .qTransitionMaximumAngleError = FOC_SCALAR(0.25f),
    .wTransitionTimeoutMs         = 1000U,
    .hwTransitionQualificationSamples = 3U,
    .hwTransitionBlendSamples         = 8U,
    .wStartupDelayMs = 200U,
};

/* 产品持有的唯一 run config：D/Q 电压开环（内部开环角度发生器）。 */
static motor_run_config_t s_tMotorRunConfig = {
    .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
    .ptInitialPositionSource = NULL,
    .ptTargetPositionSource  = NULL,
    .qInitialAngle  = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(FOC_APP_OPEN_LOOP_SPEED),
    .qAcceleration  = FOC_SCALAR(FOC_APP_OPEN_LOOP_ACCEL),
    .tVoltageReference = {
        .qD = FOC_ZERO,
        .qQ = FOC_SCALAR(FOC_APP_VOLTAGE_REF_Q),
    },
};

/* 编码器电流闭环的速度安全上限（电 Hz，0=看门狗关闭）。
 * 电流闭环不天然限速，超限由 foc_app_Clock 看门狗立即停机。 */
static float s_fSpeedLimitHz;
static uint8_t s_chOverLimitCount;

#if defined(MDI_HW_HAS_I2C_ENCODER)
/* AS5600 编码器位置源（stm32g431: I2C1 PB7/PB8）。
 * 1 kHz foc_app_Clock 里 as5600_Update() 阻塞采样并缓存；
 * 20 kHz fnStep 经回调只读缓存（无阻塞），观测器负责速度外插。 */
static as5600_t s_tAs5600;
static foc_encoder_t s_tEncoder;
static foc_encoder_source_adapter_t s_tEncoderAdapter;
static foc_position_source_if_t s_tEncoderSourceIf;
static bool s_bEncoderReady;

static bool foc_app_ReadEncoderSample(void *pContext,
                                      uint16_t *phwRawAngle,
                                      uint32_t *pwSequence,
                                      bool *pbMagnetOk)
{
    as5600_sample_t tSample;

    (void)pContext;
    as5600_GetSample(&s_tAs5600, &tSample);
    if (!tSample.bValid) {
        return false;
    }
    *phwRawAngle = tSample.hwRawAngle;
    *pwSequence  = tSample.wSequence;
    *pbMagnetOk  = tSample.bMagnetOk;
    return true;
}
#endif /* MDI_HW_HAS_I2C_ENCODER */

/* 并行观测器实例与位置源接口（只观测不参与控制，由 `motor obs` 选择）。
 * 接口表存入静态存储，run config 引用其地址，保证跨 motor_Start 稳定。 */
static foc_smo_t   s_tSmo;
static foc_nlfo_t  s_tNlfo;
static foc_position_source_if_t s_tSmoSource;
static foc_position_source_if_t s_tNlfoSource;

static void foc_app_InitSmo(void)
{
    foc_smo_params_t tParams = {
        .qModelGain       = FOC_SCALAR(0.25f),
        .qResistance      = FOC_SCALAR(0.1667f),
        /* 滑模增益必须 > 反电动势幅值：50 e-turn/s 时 BEMF≈0.6 pu，
           原 0.10 远小于 BEMF，滑模面保持不住、BEMF 估计被衰减
           （实测 0.03 pu vs 实际 0.6 pu）→ 角度误差大。 */
        .qSlidingGain     = FOC_SCALAR(0.80f),
        .qBoundaryInverse = FOC_SCALAR(10.0f),
        .qEmfFilterAlpha  = FOC_SCALAR(0.10f),
        .qMinimumBemf     = FOC_SCALAR(0.005f),
    };
    if (foc_smo_Init(&s_tSmo, &tParams) == FOC_RESULT_OK) {
        s_tSmoSource = foc_smo_PositionSourceInterface(&s_tSmo);
        MLOG(I, "[FOC] SMO observer source ready\r\n");
    } else {
        MLOG(W, "[FOC] SMO init failed, observer unavailable\r\n");
    }
}

static void foc_app_InitNlfo(void)
{
    foc_nlfo_params_t tParams = {
        .qIntegratorGain    = FOC_SCALAR(0.01f),
        .qResistance        = FOC_SCALAR(0.1667f),
        .qAverageInductance = FOC_SCALAR(0.50f),
        .qFlux              = FOC_SCALAR(0.05f),
        .qCorrectionGain    = FOC_SCALAR(0.02f),
        .qMinimumFluxRatio  = FOC_SCALAR(0.05f),
    };
    if (foc_nlfo_Init(&s_tNlfo, &tParams) == FOC_RESULT_OK) {
        s_tNlfoSource = foc_nlfo_PositionSourceInterface(&s_tNlfo);
        MLOG(I, "[FOC] NLFO observer source ready\r\n");
    } else {
        MLOG(W, "[FOC] NLFO init failed, observer unavailable\r\n");
    }
}

static modus_base_cfg_t s_tFocAppBaseCfg = {
    .wId     = FOC_APP,
    .wParent = 0,
    .FcnInterface = {
        .Clock = foc_app_Clock,
        .Run   = foc_app_Run,
    },
};

/* ---------------------------------------------------------------------------
 * motor_time_if_t / motor_sync_if_t 目标侧绑定
 * 临界区走 perf_counter 多架构全局中断守卫（Cortex-M 用 PRIMASK、RISC-V 用
 * mstatus.MIE，中断安全，允许嵌套）；毫秒时钟取 perf_counter。
 * ------------------------------------------------------------------------- */
static uint32_t foc_app_GetMilliseconds(void *pContext)
{
    (void)pContext;
    return (uint32_t)get_system_ms();
}

static uintptr_t foc_app_EnterCritical(void *pContext)
{
    (void)pContext;
    return (uintptr_t)perfc_port_disable_global_interrupt();
}

static void foc_app_ExitCritical(void *pContext, uintptr_t wState)
{
    (void)pContext;
    perfc_port_resume_global_interrupt((uint32_t)wState);
}

static const char *foc_app_StateName(motor_state_e eState)
{
    switch (eState) {
        case MOTOR_STATE_IDLE:     return "IDLE";
        case MOTOR_STATE_STARTING: return "STARTING";
        case MOTOR_STATE_STOPPING: return "STOPPING";
        case MOTOR_STATE_RUNNING:  return "RUNNING";
        case MOTOR_STATE_FAULT:    return "FAULT";
        default:                   return "?";
    }
}

/* 事件环只做日志；正常控制路径不依赖事件消费。 */
static void foc_app_DrainEvents(foc_app_t *ptThis)
{
    static const char * const s_apcEventNames[] = {
        "CMD_ACCEPT", "CMD_REJECT", "STATE", "SRC_VALID",
        "TRANS_START", "TRANS_DONE", "TRANS_TIMEOUT", "FAULT",
    };
    motor_event_t tEvent;

    while (motor_DebugReadEvent(ptThis->ptMotor, &tEvent)) {
        MLOGF(D, "[FOC][Evt#%lu] %s %d->%d cmd=%d res=%d flt=0x%lX\r\n",
              (unsigned long)tEvent.wSequence,
              (unsigned int)tEvent.eType < 8U ?
                  s_apcEventNames[tEvent.eType] : "?",
              (int)tEvent.eFromState, (int)tEvent.eToState,
              (int)tEvent.eCommand, (int)tEvent.eResult,
              (unsigned long)tEvent.wFaults);
    }
}


/* ---------------------------------------------------------------------------
 * 应用 FSM：纯编排，只提交产品命令并驱动 motor_RunFSM()。
 * ------------------------------------------------------------------------- */
fsm_rt_t foc_app_RunFSM(foc_app_t *ptThis)
{
    if (ptThis == NULL || ptThis->ptMotor == NULL) {
        return fsm_rt_err;
    }
    return motor_RunFSM(ptThis->ptMotor);
}

void foc_app_Start(foc_app_t *ptThis)
{
    motor_state_e eRunState;
    uint32_t wFaults;
    foc_result_t eResult;

    if (ptThis == NULL || ptThis->ptMotor == NULL) { return; }
    if (motor_GetStatus(ptThis->ptMotor, &eRunState, &wFaults) ==
            FOC_RESULT_OK &&
        wFaults != MOTOR_FAULT_NONE) {
        MLOGF(W, "[FOC] Motor faults active: 0x%lX, clear first\r\n",
              (unsigned long)wFaults);
        return;
    }
    eResult = motor_Start(ptThis->ptMotor, &s_tMotorRunConfig);
    if (eResult == FOC_RESULT_OK) {
        MLOG(I, "[FOC] Start command accepted\r\n");
    } else {
        MLOGF(W, "[FOC] Start command rejected: %d\r\n", (int)eResult);
    }
}

void foc_app_Stop(foc_app_t *ptThis)
{
    foc_result_t eResult;

    if (ptThis == NULL || ptThis->ptMotor == NULL) { return; }
    eResult = motor_Stop(ptThis->ptMotor);
    if (eResult == FOC_RESULT_OK) {
        MLOG(I, "[FOC] Stop command accepted\r\n");
    } else {
        MLOGF(W, "[FOC] Stop command rejected: %d\r\n", (int)eResult);
    }
}

extern foc_app_t tFocApp;

/* 高频控制入口：仅在 ADC 抢占转换完成中断上下文中调用。 */
void foc_app_HighFrequencyISR(void)
{
    if (tFocApp.ptMotor != NULL) {
        (void)motor_HighFrequencyStep(tFocApp.ptMotor);
#if MWAVEFORM_ENABLE
        phase_test_waveform_hf_step(tFocApp.ptMotor);
#endif
    }
}

static void foc_app_HandleButton(foc_app_t *ptThis)
{
#if defined(MDI_HW_HAS_BUTTON_START)
    uint32_t wNow = (uint32_t)get_system_ms();
    bool bCurrBtnState = (MDI_Read(HW.ptButtonStart) == MDI_GPIO_LOW);
    motor_state_e eRunState;
    uint32_t wFaults;

    if (bCurrBtnState == ptThis->bLastButtonState) { return; }
    if ((uint32_t)(wNow - ptThis->wLastButtonTick) < 50U) { return; }
    ptThis->bLastButtonState = bCurrBtnState;
    ptThis->wLastButtonTick = wNow;
    if (!bCurrBtnState) { return; }     /* 只在按下沿动作 */

    if (motor_GetStatus(ptThis->ptMotor, &eRunState, &wFaults) !=
        FOC_RESULT_OK) {
        return;
    }
    if (wFaults != MOTOR_FAULT_NONE) {
        MLOGF(I, "[Button] Faults 0x%lX, clearing...\r\n",
              (unsigned long)wFaults);
        if (motor_ClearFault(ptThis->ptMotor) != FOC_RESULT_OK) {
            MLOG(W, "[Button] ClearFault rejected (PWM on?)\r\n");
        }
    } else if (eRunState == MOTOR_STATE_IDLE) {
        MLOG(I, "[Button] Press: Starting Motor...\r\n");
        foc_app_Start(ptThis);
    } else {
        MLOG(I, "[Button] Press: Stopping Motor...\r\n");
        foc_app_Stop(ptThis);
    }
#else
    (void)ptThis;   /* no start/stop button in this chip's MDI hardware pool */
#endif
}

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    uint32_t   wEvent;
    uint32_t   wNow;
    motor_state_e eRunState;
    uint32_t wFaults;
    motor_telemetry_t tTelemetry;
    foc_adc_calib_t tCalib;

    if (ptThis == NULL) { return MODUS_EFAIL; }

#if MODUS_ENABLE
    wEvent = mbase_EventPend(ptThis->ptBase);
    (void)wEvent;
#endif

    foc_app_HandleButton(ptThis);

    /* 物理串口接收等待：当有串口输入数据时，跨模块 Post 转发给
       TEMPLATE_CLASS 的 RingBuffer，达到 Echo 效果 */
#if MODUS_ENABLE
    {
        uint8_t chBuf[64];
        int32_t nReadBytes = MDI_Read(HW.ptSerial, chBuf, sizeof(chBuf));
        if (nReadBytes > 0) {
            mbase_MessagePostToRing(TEMPLATE_CLASS, chBuf, nReadBytes);
        }
    }
#endif

    (void)foc_app_RunFSM(ptThis);
    foc_app_DrainEvents(ptThis);

    wNow = (uint32_t)get_system_ms();
    if ((uint32_t)(wNow - ptThis->wLastHeartbeatTick) >= 1000U) {
        ptThis->wLastHeartbeatTick = wNow;
        if (motor_GetStatus(ptThis->ptMotor, &eRunState, &wFaults) ==
                FOC_RESULT_OK &&
            motor_GetTelemetry(ptThis->ptMotor, &tTelemetry) ==
                FOC_RESULT_OK &&
            motor_GetCurrentCalibration(ptThis->ptMotor, &tCalib) ==
                FOC_RESULT_OK) {
#if FOC_HF_PROFILE
            motor_hf_profile_snapshot_t tProf = {0};
            (void)motor_GetHighFrequencyProfileSnapshot(ptThis->ptMotor, &tProf);
            /* 完整版 heartbeat（子模块周期数 + 三相电流），按需恢复：
            MLOGF(T, "[Heartbeat] motor: %s, seq: %lu, total: %lu, entry: %lu, sample: %lu, clarke: %lu, pos: %lu, park: %lu, pi: %lu, ipark: %lu, mod: %lu, commit: %lu, setduty: %lu, angle: %.4f, Iu: %.4f, Iv: %.4f, Iw: %.4f\r\n",
                  foc_app_StateName(tSnapshot.eRunState),
                  (unsigned long)tProf.wSampleSequence,
                  (unsigned long)tProf.wTotalCycles,
                  (unsigned long)tProf.wEntryCycles,
                  (unsigned long)tProf.wSampleCurrentCycles,
                  (unsigned long)tProf.wClarkeCycles,
                  (unsigned long)tProf.wPositionCycles,
                  (unsigned long)tProf.wParkCycles,
                  (unsigned long)tProf.wPiCycles,
                  (unsigned long)tProf.wIparkCycles,
                  (unsigned long)tProf.wModulateCycles,
                  (unsigned long)tProf.wCommitCycles,
                  (unsigned long)tProf.wSetDutyCycles,
                  (double)foc_angle_to_turns(tSnapshot.tActiveAngle),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIu),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIv),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIw));
            */
            /* 精简版：只留总周期数与角度 */
            MLOGF(T, "[Heartbeat] motor: %s, seq: %lu, total: %lu, angle: %.4f\r\n",
                  foc_app_StateName(eRunState),
                  (unsigned long)tProf.wSampleSequence,
                  (unsigned long)tProf.wTotalCycles,
                  (double)foc_angle_to_turns(tTelemetry.tActiveAngle));
#else
            MLOGF(T, "[Heartbeat] motor: %s, Iq: %.4f, Id: %.4f, angle: %.4f, Iu: %.4f, Iv: %.4f, Iw: %.4f, offset: %lu/%lu/%lu\r\n",
                  foc_app_StateName(eRunState),
                  (double)foc_to_float(tTelemetry.tCurrent.qQ),
                  (double)foc_to_float(tTelemetry.tCurrent.qD),
                  (double)foc_angle_to_turns(tTelemetry.tActiveAngle),
                  (double)foc_to_float(tTelemetry.tPhaseCurrent.qIu),
                  (double)foc_to_float(tTelemetry.tPhaseCurrent.qIv),
                  (double)foc_to_float(tTelemetry.tPhaseCurrent.qIw),
                  (unsigned long)tCalib.wOffsetU,
                  (unsigned long)tCalib.wOffsetV,
                  (unsigned long)tCalib.wOffsetW);
#endif
        }
    }

    return MODUS_SUCCESS;
}

/* 低频调度：MODUS 1 ms 系统时钟（SysTick → modus_Clock → foc_app_Clock）。
 * 这是 motor_LowFrequencyStep() 唯一的低频调度点。 */
static int foc_app_Clock(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    if (ptThis == NULL) { return MODUS_EFAIL; }

    if (ptThis->ptMotor != NULL) {
        (void)motor_LowFrequencyStep(ptThis->ptMotor);
    }
#if defined(MDI_HW_HAS_I2C_ENCODER)
    if (s_bEncoderReady) {
        (void)as5600_Update(&s_tAs5600);
    }
#endif

    /* 速度安全看门狗（1 kHz）：电流闭环不天然限速，编码器速度反馈在
       高速时有效（低速才有量化噪声），超限连续 50ms 立即停机。 */
    if (s_fSpeedLimitHz > 0.0f && ptThis->ptMotor != NULL) {
        motor_telemetry_t tTel;
        if (motor_GetTelemetry(ptThis->ptMotor, &tTel) == FOC_RESULT_OK) {
            float fSpeed = foc_to_float(tTel.qActiveSpeed);
            if (fSpeed > s_fSpeedLimitHz ||
                fSpeed < -s_fSpeedLimitHz) {
                s_chOverLimitCount++;
                if (s_chOverLimitCount >= 50U) {
                    MLOGF(W, "[FOC] Speed limit %.1f Hz exceeded "
                             "(%.1f), stopping\r\n",
                          (double)s_fSpeedLimitHz, (double)fSpeed);
                    (void)motor_Stop(ptThis->ptMotor);
                    s_fSpeedLimitHz = 0.0f;   /* 只触发一次 */
                    s_chOverLimitCount = 0U;
                }
            } else {
                s_chOverLimitCount = 0U;
            }
        }
    }

    phase_test_waveform_step(ptThis->ptMotor);

    return MODUS_SUCCESS;
}

static void foc_app_InitControlConfig(void)
{
    const foc_pid_params_t tCurrentParams = {
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        /* 电压限幅 ±0.55 pu（接近 SVPWM 上限 0.577）：
           50 e-turn/s 需 Vq ≈ BEMF(0.6 pu)+Iq·R，±0.20 只够 ~16 e-turn/s。 */
        .qOutputMinimum = FOC_SCALAR(-0.55f),
        .qOutputMaximum = FOC_SCALAR(0.55f),
        .qIntegratorMinimum = FOC_SCALAR(-0.50f),
        .qIntegratorMaximum = FOC_SCALAR(0.50f),
    };
    /* 速度环参数：PI 控制，输出限幅为最大 Iq 电流（安全预算 ±0.10 pu） */
    const foc_pid_params_t tSpeedParams = {
        .tKp = {0, FOC_SCALAR(0.2f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},     // 带积分以消除稳态速差
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.10f),  // 最大允许 Iq 电流 (-10%)
        .qOutputMaximum = FOC_SCALAR(0.10f),   // 最大允许 Iq 电流 (+10%)
        .qIntegratorMinimum = FOC_SCALAR(-0.10f),
        .qIntegratorMaximum = FOC_SCALAR(0.10f),
    };
    /* 位置环参数：纯 P 控制，输出限幅为最大目标转速 */
    const foc_pid_params_t tPositionParams = {
        .tKp = {0, FOC_SCALAR(4.0f)},         // 位置 P 增益通常较大
        .tKiTs = {0, FOC_ZERO},                // 位置环推荐 Ki = 0
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.5f),   // 最大允许目标转速 (-50% max speed)
        .qOutputMaximum = FOC_SCALAR(0.5f),    // 最大允许目标转速 (+50% max speed)
        .qIntegratorMinimum = FOC_ZERO,
        .qIntegratorMaximum = FOC_ZERO,
    };

    s_tMotorConfig.tControl.tIdParams = tCurrentParams;
    s_tMotorConfig.tControl.tIqParams = tCurrentParams;
    s_tMotorConfig.tControl.tSpeedParams = tSpeedParams;
    s_tMotorConfig.tControl.tPositionParams = tPositionParams;
    s_tMotorConfig.tControl.eModulation = MOTOR_MODULATION_SVPWM;
}

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t     *ptThis = (foc_app_t *)wObjectAddr;
    foc_app_cfg_t *ptCfg  = (foc_app_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        MLOGF(E, "foc_app_Init: NULL pointer.\n");
        return MODUS_EFAIL;
    }

    memset(ptThis, 0, sizeof(foc_app_t));
    ptThis->ptMotor = ptCfg->ptMotor;
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;

    foc_app_InitControlConfig();
    if (foc_hal_mdi_BindDefault(&s_tMotorConfig.tHal) != FOC_RESULT_OK ||
        motor_Init(ptThis->ptMotor, &s_tMotorConfig) != FOC_RESULT_OK) {
        MLOG(E, "foc_app_Init: motor init or binding failed.\r\n");
        return MODUS_EFAIL;
    }

#if defined(MDI_HW_HAS_I2C_ENCODER)
    {
        foc_encoder_params_t tEncoderParams;

        foc_encoder_DefaultParams(&tEncoderParams);
        s_bEncoderReady =
            (as5600_Init(&s_tAs5600, HW.ptI2c1) == 0) &&
            (foc_encoder_Init(&s_tEncoder, &tEncoderParams) == FOC_RESULT_OK) &&
            (foc_encoder_source_Init(&s_tEncoderAdapter, &s_tEncoder,
                                     NULL,
                                     foc_app_ReadEncoderSample) ==
                 FOC_RESULT_OK);
        if (s_bEncoderReady) {
            s_tEncoderSourceIf =
                foc_encoder_PositionSourceInterface(&s_tEncoderAdapter);
            MLOG(I, "[FOC] AS5600 encoder source ready\r\n");
        } else {
            MLOG(W, "[FOC] AS5600 encoder unavailable, open-loop only\r\n");
        }
    }
#endif

    foc_app_InitSmo();
    foc_app_InitNlfo();

    phase_test_waveform_init();

    phase_testA();
    phase_testB(ptThis->ptMotor);
    phase_testC(ptThis);

#if MODUS_ENABLE
    return mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
#else
    return MODUS_SUCCESS;
#endif
}

#if FOC_SUPPORT
/* FOC motor object */
motor_handle_t s_tMotor;
MODUS_DECLARE_OBJECT(foc_app, FocApp, .ptMotor = &s_tMotor);
#endif

static void cmd_motor(const char *args)
{
    extern foc_app_t tFocApp;
    if (tFocApp.ptMotor == NULL) {
        MLOG(E, "Motor object not initialized\r\n");
        return;
    }
    if (strncmp(args, "start", 5) == 0) {
        extern motor_run_config_t s_tMotorRunConfig;
        s_fSpeedLimitHz = 0.0f;     /* 开环电压模式由 Vq 限速，不用看门狗 */
        s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP;
        s_tMotorRunConfig.ptInitialPositionSource = NULL;
        s_tMotorRunConfig.ptTargetPositionSource = NULL;
        s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
        s_tMotorRunConfig.qOpenLoopSpeed = FOC_SCALAR(FOC_APP_OPEN_LOOP_SPEED);
        s_tMotorRunConfig.qAcceleration = FOC_SCALAR(FOC_APP_OPEN_LOOP_ACCEL);
        s_tMotorRunConfig.tVoltageReference.qD = FOC_ZERO;
        s_tMotorRunConfig.tVoltageReference.qQ = FOC_SCALAR(FOC_APP_VOLTAGE_REF_Q);
        foc_app_Start(&tFocApp);
    } else if (strncmp(args, "enc", 3) == 0) {
#if defined(MDI_HW_HAS_I2C_ENCODER)
        float fIqLimit = 0.10f;     /* Iq 初始参考（速度环接管后无效，占位） */
        float fSpeedHz = 50.0f;     /* 速度目标（电 Hz，带符号：正=正转 负=反转） */
        if (!s_bEncoderReady) {
            MLOG(E, "Encoder not ready, use 'motor start' instead\r\n");
            return;
        }
        if (sscanf(args + 3, "%f %f", &fIqLimit, &fSpeedHz) >= 1) {
            fIqLimit = fIqLimit > 0.10f ? 0.10f : fIqLimit;
            fIqLimit = fIqLimit < 0.02f ? 0.02f : fIqLimit;
        }
        if (fSpeedHz < -100.0f) { fSpeedHz = -100.0f; }
        if (fSpeedHz > 100.0f) { fSpeedHz = 100.0f; }
        /* 速度闭环 + 直连编码器：
           - 电流模式无速度控制会跑飞，必须速度环钳制；
           - 高速（≥50 e-turn/s）下编码器测速量化噪声占比小，速度环平滑；
           - 速度环输出 Iq（限幅 ±0.10）→ 电流环跟踪；
           - SMO 并行观测 αβ 电流/电压，反推角度 vs 编码器角度对比。 */
        s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_SPEED;
        s_tMotorRunConfig.ptInitialPositionSource = &s_tEncoderSourceIf;
        s_tMotorRunConfig.ptTargetPositionSource  = &s_tEncoderSourceIf;
        s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
        s_tMotorRunConfig.qOpenLoopSpeed = FOC_ZERO;
        s_tMotorRunConfig.qAcceleration = FOC_SCALAR(2.0f);
        s_tMotorRunConfig.qSpeedReference = FOC_SCALAR(
            fSpeedHz / (float)s_tMotorConfig.tParams.chPolePairs);
        s_tMotorRunConfig.tCurrentReference.qD = FOC_ZERO;
        s_tMotorRunConfig.tCurrentReference.qQ = FOC_SCALAR(fIqLimit);
        s_fSpeedLimitHz = 0.0f;     /* 看门狗禁用（速度环限速） */
        foc_app_Start(&tFocApp);
#else
        MLOG(E, "No I2C encoder on this target\r\n");
#endif
    } else if (strncmp(args, "obs", 3) == 0) {
        const char *pchObs = args + 3;
        while (*pchObs == ' ') { pchObs++; }
        if (strncmp(pchObs, "smo", 3) == 0) {
            s_tMotorRunConfig.ptObservationPositionSource = &s_tSmoSource;
            MLOG(I, "Observer: SMO\r\n");
        } else if (strncmp(pchObs, "nlfo", 4) == 0) {
            s_tMotorRunConfig.ptObservationPositionSource = &s_tNlfoSource;
            MLOG(I, "Observer: NLFO\r\n");
        } else {
            s_tMotorRunConfig.ptObservationPositionSource = NULL;
            MLOG(I, "Observer: disabled\r\n");
        }
    } else if (strncmp(args, "stop", 4) == 0) {
        foc_app_Stop(&tFocApp);
    } else if (strncmp(args, "clear", 5) == 0) {
        if (motor_ClearFault(tFocApp.ptMotor) == FOC_RESULT_OK) {
            MLOG(I, "Motor faults cleared\r\n");
        } else {
            MLOG(W, "ClearFault rejected (not in FAULT or PWM on)\r\n");
        }
    } else if (strncmp(args, "pid", 3) == 0) {
        float fKp = 0.0f;
        float fKi = 0.0f;
        if (sscanf(args + 3, "%f %f", &fKp, &fKi) == 2) {
            (void)foc_gain_from_float(fKp, &s_tMotorConfig.tControl.tIdParams.tKp);
            (void)foc_gain_from_float(fKi, &s_tMotorConfig.tControl.tIdParams.tKiTs);
            (void)foc_gain_from_float(fKp, &s_tMotorConfig.tControl.tIqParams.tKp);
            (void)foc_gain_from_float(fKi, &s_tMotorConfig.tControl.tIqParams.tKiTs);
            MLOGF(I, "PID params updated: Kp = %.3f, KiTs = %.3f\r\n", (double)fKp, (double)fKi);
        } else {
            MLOG(I, "Usage: motor pid <kp> <ki>\r\n");
        }
    } else if (strncmp(args, "vq", 2) == 0) {
        float fVq;
        if (sscanf(args + 2, "%f", &fVq) == 1) {
            s_tMotorRunConfig.tVoltageReference.qQ = FOC_SCALAR(fVq);
            motor_SetVoltageReference(tFocApp.ptMotor,
                                      FOC_ZERO, FOC_SCALAR(fVq));
            MLOGF(I, "Vq reference: %.3f\r\n", (double)fVq);
        } else {
            MLOG(I, "Usage: motor vq <volts-pu>\r\n");
        }
    } else if (strncmp(args, "status", 6) == 0) {
        motor_snapshot_t tSnapshot;
        if (motor_GetSnapshot(tFocApp.ptMotor, &tSnapshot) !=
            FOC_RESULT_OK) {
            MLOG(E, "Snapshot failed\r\n");
            return;
        }
        MLOGF(I, "Motor state: %s (phase %d, mode %d, pwm %d)\r\n",
              foc_app_StateName(tSnapshot.eRunState),
              (int)tSnapshot.eStartupPhase, (int)tSnapshot.eControlMode,
              (int)tSnapshot.bPwmEnabled);
        MLOGF(I, " - Faults: 0x%lX, events: seq=%lu ovf=%lu\r\n",
              (unsigned long)tSnapshot.wFaults,
              (unsigned long)tSnapshot.wEventSequence,
              (unsigned long)tSnapshot.wEventOverwriteCount);
        MLOGF(I, " - Angle: %.1f deg, speed: %.3f e-turn/s\r\n",
              (double)foc_angle_to_turns(tSnapshot.tActiveAngle) * 360.0,
              _D(tSnapshot.qActiveSpeed));
        MLOGF(I, " - Duty U=%.1f%%, V=%.1f%%, W=%.1f%%\r\n",
              _D(tSnapshot.tDuty.qU) * 100.0,
              _D(tSnapshot.tDuty.qV) * 100.0,
              _D(tSnapshot.tDuty.qW) * 100.0);
        MLOGF(I, " - Current U=%.3f, V=%.3f, W=%.3f\r\n",
              _D(tSnapshot.tPhaseCurrent.qIu),
              _D(tSnapshot.tPhaseCurrent.qIv),
              _D(tSnapshot.tPhaseCurrent.qIw));
        MLOGF(I, " - Iq ref=%.4f actual=%.4f | Id ref=%.4f actual=%.4f\r\n",
              _D(tSnapshot.tCurrentReference.qQ),
              _D(tSnapshot.tCurrent.qQ),
              _D(tSnapshot.tCurrentReference.qD),
              _D(tSnapshot.tCurrent.qD));
        MLOGF(I, " - Calib: U=%lu V=%lu W=%lu done=%d\r\n",
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetU,
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetV,
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetW,
              (int)tSnapshot.tCurrentCalibration.bIsCalibrated);
    } else {
        MLOG(I, "Usage: motor <start|enc|obs <smo|nlfo|off>|stop|clear|vq <x>|pid <p> <i>|status>\r\n");
    }
}
MODUS_SHELL_CMD(motor, cmd_motor, "Control FOC Motor (start/enc/obs/stop/clear/vq/pid/status)");

#if defined(MDI_HW_HAS_I2C_ENCODER)
static void cmd_encoder(const char *args)
{
    as5600_sample_t tSample;
    motor_state_e eRunState;
    uint32_t wFaults;

    if (!s_bEncoderReady) {
        MLOG(E, "Encoder not ready\r\n");
        return;
    }
    if (strncmp(args, "cal", 3) == 0) {
        /* 电气零位标定：开环静止场（Vd 保持）把转子吸到电角度 0，
           读编码器机械角反算偏移：offset = -(θm * Pp) mod 1。 */
        unsigned int i;
        bool bRunning = false;

        if (motor_GetStatus(tFocApp.ptMotor, &eRunState, &wFaults) !=
            FOC_RESULT_OK ||
            wFaults != MOTOR_FAULT_NONE ||
            eRunState != MOTOR_STATE_IDLE) {
            MLOG(W, "Cal requires IDLE & no faults (stop motor first)\r\n");
            return;
        }
        MLOG(I, "Encoder cal: aligning rotor, do NOT touch the shaft...\r\n");
        s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP;
        s_tMotorRunConfig.ptInitialPositionSource = NULL;
        s_tMotorRunConfig.ptTargetPositionSource = NULL;
        s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
        s_tMotorRunConfig.qOpenLoopSpeed = FOC_ZERO;
        s_tMotorRunConfig.qAcceleration = FOC_ZERO;
        s_tMotorRunConfig.tVoltageReference.qD = FOC_SCALAR(0.04f);
        s_tMotorRunConfig.tVoltageReference.qQ = FOC_ZERO;
        foc_app_Start(&tFocApp);
        /* Shell 上下文可能阻塞主循环，自行驱动 FSM 直至 RUNNING */
        for (i = 0U; i < 200U; i++) {
            delay_ms(20);
            (void)motor_RunFSM(tFocApp.ptMotor);
            if (motor_GetStatus(tFocApp.ptMotor, &eRunState, &wFaults) ==
                    FOC_RESULT_OK &&
                eRunState == MOTOR_STATE_RUNNING &&
                wFaults == MOTOR_FAULT_NONE) {
                bRunning = true;
                break;
            }
        }
        if (!bRunning) {
            foc_app_Stop(&tFocApp);
            MLOG(E, "Encoder cal: failed to reach RUNNING, abort\r\n");
            return;
        }
        delay_ms(1500);     /* 转子吸合稳定 */
        as5600_GetSample(&s_tAs5600, &tSample);
        foc_app_Stop(&tFocApp);
        if (!tSample.bValid) {
            MLOG(E, "Encoder cal: sample invalid, abort\r\n");
            return;
        }
        {
            float fMechTurns = (float)tSample.hwRawAngle / 4096.0f;
            float fPolePairs =
                (float)s_tMotorConfig.tPosition.chPolePairs;
            foc_angle_t tOffset =
                foc_angle_from_turns(-fMechTurns * fPolePairs);
            if (motor_SetPositionOffset(tFocApp.ptMotor, tOffset) ==
                FOC_RESULT_OK) {
                s_tMotorConfig.tPosition.tElectricalOffset = tOffset;
                MLOGF(I, "Encoder cal: raw=%u mech=%.4f turn "
                         "offset=%.4f turn (RAM only)\r\n",
                      (unsigned int)tSample.hwRawAngle,
                      (double)fMechTurns,
                      (double)foc_angle_to_turns(tOffset));
            } else {
                MLOG(E, "Encoder cal: SetPositionOffset failed\r\n");
            }
        }
        /* 恢复产品默认开环运行配置，避免按钮启动进入对齐场 */
        s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP;
        s_tMotorRunConfig.ptInitialPositionSource = NULL;
        s_tMotorRunConfig.ptTargetPositionSource = NULL;
        s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
        s_tMotorRunConfig.qOpenLoopSpeed =
            FOC_SCALAR(FOC_APP_OPEN_LOOP_SPEED);
        s_tMotorRunConfig.qAcceleration = FOC_SCALAR(FOC_APP_OPEN_LOOP_ACCEL);
        s_tMotorRunConfig.tVoltageReference.qD = FOC_ZERO;
        s_tMotorRunConfig.tVoltageReference.qQ =
            FOC_SCALAR(FOC_APP_VOLTAGE_REF_Q);
        return;
    }
    as5600_GetSample(&s_tAs5600, &tSample);
    MLOGF(I, "AS5600: raw=%u (%.2f deg), seq=%lu, valid=%d\r\n",
          (unsigned int)tSample.hwRawAngle,
          (double)tSample.hwRawAngle * 360.0 / 4096.0,
          (unsigned long)tSample.wSequence,
          (int)tSample.bValid);
    MLOGF(I, "  status=0x%02X: MD=%d(检测) ML=%d(过弱) MH=%d(过强) => %s\r\n",
          (unsigned int)tSample.chStatus,
          (tSample.chStatus & AS5600_STATUS_MD) ? 1 : 0,
          (tSample.chStatus & AS5600_STATUS_ML) ? 1 : 0,
          (tSample.chStatus & AS5600_STATUS_MH) ? 1 : 0,
          tSample.bMagnetOk ? "ok" : "bad");
    MLOGF(I, "  elec-offset=%.4f turn (%.1f deg)\r\n",
          (double)foc_angle_to_turns(
              s_tMotorConfig.tPosition.tElectricalOffset),
          (double)foc_angle_to_turns(
              s_tMotorConfig.tPosition.tElectricalOffset) * 360.0);
}
MODUS_SHELL_CMD(encoder, cmd_encoder, "Show AS5600 angle / magnet, cal = zero offset calibrate");

/* SMO 观测器内部状态诊断（被动读取，电机运行中可查） */
static void cmd_smo(const char *args)
{
    foc_scalar_t qMagA = foc_abs(s_tSmo.tBemf.qAlpha);
    foc_scalar_t qMagB = foc_abs(s_tSmo.tBemf.qBeta);
    foc_scalar_t qMag = foc_add_sat(
        qMagA > qMagB ? qMagA : qMagB,
        foc_mul_pu(qMagA > qMagB ? qMagB : qMagA, FOC_HALF));
    motor_snapshot_t tSnap;

    (void)args;
    MLOGF(I, "SMO: bemf a=%.4f b=%.4f mag~%.4f (min %.4f) "
             "ang=%.4f hasAngle=%d\r\n",
          _D(s_tSmo.tBemf.qAlpha), _D(s_tSmo.tBemf.qBeta), _D(qMag),
          _D(s_tSmo.tParams.qMinimumBemf),
          _D(foc_angle_to_turns(s_tSmo.tAngle)),
          (int)s_tSmo.bHasAngle);
    MLOGF(I, "  runcfg: obs=%s\r\n",
          s_tMotorRunConfig.ptObservationPositionSource != NULL ?
          "bound" : "NULL");
    if (motor_GetSnapshot(tFocApp.ptMotor, &tSnap) == FOC_RESULT_OK) {
        MLOGF(I, "  snap: cand=%.4f act=%.4f err=%.4f "
                 "candFlags=0x%02X\r\n",
              _D(foc_angle_to_turns(tSnap.tCandidateAngle)),
              _D(foc_angle_to_turns(tSnap.tActiveAngle)),
              _D(tSnap.qAngleError),
              (unsigned int)tSnap.eCandidateSourceValidFlags);
    }
}
MODUS_SHELL_CMD(smo, cmd_smo, "Show SMO observer internal state");
#endif
