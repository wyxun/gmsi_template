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
#include "foc_pid.h"
#include "foc_controller.h"
#include "mdebug/mshell.h"
#include "perfc_port.h"
#include "motor/motor_private.h"

extern uint32_t haladc_GetInjected(uint32_t wAdc, uint32_t wRank);

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
static motor_handle_t  *s_ptMotorISR;   /* 高频 ISR 绑定的实例 */

/* Id/Iq/速度/位置四个控制器实例，电压开环模式不使用，为后续闭环模式
 * 预先绑定（motor_Start 对闭环模式强制校验控制器存在）。 */
static foc_pid_t s_tPidId;
static foc_pid_t s_tPidIq;
static foc_pid_t s_tPidSpeed;
static foc_pid_t s_tPidPosition;

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
        .pContext = NULL,
        .fnGetMilliseconds = foc_app_GetMilliseconds,
    },
    .tSync = {
        .pContext = NULL,
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
    motor_snapshot_t tSnapshot;
    foc_result_t eResult;

    if (ptThis == NULL || ptThis->ptMotor == NULL) { return; }
    if (motor_GetSnapshot(ptThis->ptMotor, &tSnapshot) == FOC_RESULT_OK &&
        tSnapshot.wFaults != MOTOR_FAULT_NONE) {
        MLOGF(W, "[FOC] Motor faults active: 0x%lX, clear first\r\n",
              (unsigned long)tSnapshot.wFaults);
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

/* 高频控制入口：仅在 ADC 抢占转换完成中断上下文中调用。 */
void foc_app_HighFrequencyISR(void)
{
    if (s_ptMotorISR != NULL) {
        (void)motor_HighFrequencyStep(s_ptMotorISR);
    }
}

static void foc_app_HandleButton(foc_app_t *ptThis)
{
#if defined(MDI_HW_HAS_BUTTON_START)
    uint32_t wNow = (uint32_t)get_system_ms();
    bool bCurrBtnState = (MDI_Read(HW.ptButtonStart) == MDI_GPIO_LOW);
    motor_snapshot_t tSnapshot;

    if (bCurrBtnState == ptThis->bLastButtonState) { return; }
    if ((uint32_t)(wNow - ptThis->wLastButtonTick) < 50U) { return; }
    ptThis->bLastButtonState = bCurrBtnState;
    ptThis->wLastButtonTick = wNow;
    if (!bCurrBtnState) { return; }     /* 只在按下沿动作 */

    if (motor_GetSnapshot(ptThis->ptMotor, &tSnapshot) != FOC_RESULT_OK) {
        return;
    }
    if (tSnapshot.wFaults != MOTOR_FAULT_NONE) {
        MLOGF(I, "[Button] Faults 0x%lX, clearing...\r\n",
              (unsigned long)tSnapshot.wFaults);
        if (motor_ClearFault(ptThis->ptMotor) != FOC_RESULT_OK) {
            MLOG(W, "[Button] ClearFault rejected (PWM on?)\r\n");
        }
    } else if (tSnapshot.eRunState == MOTOR_STATE_IDLE) {
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
    motor_snapshot_t tSnapshot;

    if (ptThis == NULL) { return MODUS_EFAIL; }

    wEvent = mbase_EventPend(ptThis->ptBase);
    (void)wEvent;

    foc_app_HandleButton(ptThis);

    /* 物理串口接收等待：当有串口输入数据时，跨模块 Post 转发给
       TEMPLATE_CLASS 的 RingBuffer，达到 Echo 效果 */
    {
        uint8_t chBuf[64];
        int32_t nReadBytes = MDI_Read(HW.ptSerial, chBuf, sizeof(chBuf));
        if (nReadBytes > 0) {
            mbase_MessagePostToRing(TEMPLATE_CLASS, chBuf, nReadBytes);
        }
    }

    (void)foc_app_RunFSM(ptThis);
    foc_app_DrainEvents(ptThis);

    wNow = (uint32_t)get_system_ms();
    if ((uint32_t)(wNow - ptThis->wLastHeartbeatTick) >= 1000U) {
        ptThis->wLastHeartbeatTick = wNow;
        if (motor_GetSnapshot(ptThis->ptMotor, &tSnapshot) ==
            FOC_RESULT_OK) {
            extern volatile uint32_t g_wHFStepCycles;
#if defined(MOTOR_PROFILE_CYCLES)
            extern volatile uint32_t g_wTestSinfCycles;
            extern volatile uint32_t g_wSampleCurrentCycles;
            extern volatile uint32_t g_wClarkeCycles;
            extern volatile uint32_t g_wParkCycles;
            extern volatile uint32_t g_wIparkCycles;
            extern volatile uint32_t g_wModulateCycles;
            extern volatile uint32_t g_wCommitCycles;
            MLOGF(T, "[Heartbeat] motor: %s, HF: %lu, sinf: %lu, sample: %lu, clarke: %lu, park: %lu, ipark: %lu, mod: %lu, commit: %lu, angle: %.4f, Iu: %.4f, Iv: %.4f, Iw: %.4f\r\n",
                  foc_app_StateName(tSnapshot.eRunState),
                  (unsigned long)g_wHFStepCycles,
                  (unsigned long)g_wTestSinfCycles,
                  (unsigned long)g_wSampleCurrentCycles,
                  (unsigned long)g_wClarkeCycles,
                  (unsigned long)g_wParkCycles,
                  (unsigned long)g_wIparkCycles,
                  (unsigned long)g_wModulateCycles,
                  (unsigned long)g_wCommitCycles,
                  (double)foc_angle_to_turns(tSnapshot.tActiveAngle),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIu),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIv),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIw));
#else
            uint32_t rawU = haladc_GetInjected(0, 0);
            uint32_t rawV = haladc_GetInjected(1, 1);
            uint32_t rawW = haladc_GetInjected(0, 1);
            MLOGF(T, "[Heartbeat] motor: %s, HF_cycles: %lu, angle: %.4f, Iu: %.4f, Iv: %.4f, Iw: %.4f, offset: %lu/%lu/%lu, raw: %lu/%lu/%lu\r\n",
                  foc_app_StateName(tSnapshot.eRunState),
                  (unsigned long)g_wHFStepCycles,
                  (double)foc_angle_to_turns(tSnapshot.tActiveAngle),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIu),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIv),
                  (double)foc_to_float(tSnapshot.tPhaseCurrent.qIw),
                  (unsigned long)tSnapshot.tCurrentCalibration.wOffsetU,
                  (unsigned long)tSnapshot.tCurrentCalibration.wOffsetV,
                  (unsigned long)tSnapshot.tCurrentCalibration.wOffsetW,
                  (unsigned long)rawU, (unsigned long)rawV, (unsigned long)rawW);
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
    phase_test_waveform_step();

    return MODUS_SUCCESS;
}

static foc_result_t foc_app_InitControllers(void)
{
    const foc_pid_params_t tCurrentParams = {
        .tKp = {0, FOC_SCALAR(0.3f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.25f),
        .qOutputMaximum = FOC_SCALAR(0.25f),
        .qIntegratorMinimum = FOC_SCALAR(-0.18f),
        .qIntegratorMaximum = FOC_SCALAR(0.18f),
    };
    const foc_pid_params_t tOuterParams = {
        .tKp = {0, FOC_SCALAR(0.2f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.5f),
        .qOutputMaximum = FOC_SCALAR(0.5f),
        .qIntegratorMinimum = FOC_SCALAR(-0.3f),
        .qIntegratorMaximum = FOC_SCALAR(0.3f),
    };

    if (foc_pid_Init(&s_tPidId, &tCurrentParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidIq, &tCurrentParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidSpeed, &tOuterParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidPosition, &tOuterParams) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    s_tMotorConfig.tControl.tIdController =
        foc_controller_FromPid(&s_tPidId);
    s_tMotorConfig.tControl.tIqController =
        foc_controller_FromPid(&s_tPidIq);
    s_tMotorConfig.tControl.tSpeedController =
        foc_controller_FromPid(&s_tPidSpeed);
    s_tMotorConfig.tControl.tPositionController =
        foc_controller_FromPid(&s_tPidPosition);
    s_tMotorConfig.tControl.eModulation = MOTOR_MODULATION_SVPWM;
    return FOC_RESULT_OK;
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

    if (foc_app_InitControllers() != FOC_RESULT_OK ||
        foc_hal_mdi_BindDefault(&s_tMotorConfig.tHal) != FOC_RESULT_OK ||
        motor_Init(ptThis->ptMotor, &s_tMotorConfig) != FOC_RESULT_OK) {
        MLOG(E, "foc_app_Init: motor init or binding failed.\r\n");
        return MODUS_EFAIL;
    }
    s_ptMotorISR = ptThis->ptMotor;

    phase_test_waveform_init();

    phase_testA();
    phase_testB(ptThis->ptMotor);
    phase_testC(ptThis);

    return mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
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
        s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP;
        s_tMotorRunConfig.ptInitialPositionSource = NULL;
        s_tMotorRunConfig.ptTargetPositionSource = NULL;
        s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
        s_tMotorRunConfig.qOpenLoopSpeed = FOC_SCALAR(FOC_APP_OPEN_LOOP_SPEED);
        s_tMotorRunConfig.qAcceleration = FOC_SCALAR(FOC_APP_OPEN_LOOP_ACCEL);
        s_tMotorRunConfig.tVoltageReference.qD = FOC_ZERO;
        s_tMotorRunConfig.tVoltageReference.qQ = FOC_SCALAR(FOC_APP_VOLTAGE_REF_Q);
        foc_app_Start(&tFocApp);
    } else if (strncmp(args, "stop", 4) == 0) {
        foc_app_Stop(&tFocApp);
    } else if (strncmp(args, "clear", 5) == 0) {
        if (motor_ClearFault(tFocApp.ptMotor) == FOC_RESULT_OK) {
            MLOG(I, "Motor faults cleared\r\n");
        } else {
            MLOG(W, "ClearFault rejected (not in FAULT or PWM on)\r\n");
        }
    } else if (strncmp(args, "static", 6) == 0) {
        float fTurns = 0.0f;
        float fVq = 0.0f;
        if (sscanf(args + 6, "%f %f", &fTurns, &fVq) == 2) {
            extern motor_run_config_t s_tMotorRunConfig;
            s_tMotorRunConfig.qOpenLoopSpeed = FOC_ZERO;
            s_tMotorRunConfig.qAcceleration = FOC_ZERO;
            motor_impl_t *p = motor_private(tFocApp.ptMotor);
            if (p->tRt.eRunState != MOTOR_STATE_RUNNING) {
                foc_app_Start(&tFocApp);
                /* 延时几毫秒让启动和初始化重置完成 */
                for (volatile uint32_t i = 0; i < 2000000UL; i++) {}
            }
            p->qOpenLoopCommandSpeed = FOC_ZERO;
            p->tOpenLoopAngle = foc_angle_from_turns(fTurns);
            p->tRt.tThetaE = foc_angle_from_turns(fTurns);
            p->tControl.tVoltageReference.qQ = FOC_SCALAR(fVq);
            p->tControl.tVoltageReference.qD = FOC_ZERO;
            MLOGF(I, "Static lock: angle = %.3f turns, Vq = %.3f\r\n", (double)fTurns, (double)fVq);
        } else {
            MLOG(I, "Usage: motor static <turns> <vq>\r\n");
        }
    } else if (strncmp(args, "current", 7) == 0) {
        float fIq = 0.0f;
        float fSpeed = 1.0f;
        int nArgs = sscanf(args + 7, "%f %f", &fIq, &fSpeed);
        if (nArgs >= 1) {
            extern motor_run_config_t s_tMotorRunConfig;
            s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_CURRENT;
            s_tMotorRunConfig.qOpenLoopSpeed = FOC_SCALAR(fSpeed);
            s_tMotorRunConfig.qAcceleration = FOC_SCALAR(10.0f);
            s_tMotorRunConfig.tCurrentReference.qQ = FOC_SCALAR(fIq);
            s_tMotorRunConfig.tCurrentReference.qD = FOC_ZERO;

            foc_pid_Reset(&s_tPidId);
            foc_pid_Reset(&s_tPidIq);

            motor_impl_t *p = motor_private(tFocApp.ptMotor);
            if (p->tRt.eRunState != MOTOR_STATE_RUNNING) {
                foc_app_Start(&tFocApp);
                /* 延时几毫秒让启动和初始化重置完成 */
                for (volatile uint32_t i = 0; i < 2000000UL; i++) {}
            }
            p->tControl.eMode = MOTOR_CONTROL_CURRENT;
            p->qOpenLoopCommandSpeed = FOC_SCALAR(fSpeed);
            p->qOpenLoopSpeed = FOC_SCALAR(fSpeed);
            p->tControl.tCurrentReference.qQ = FOC_SCALAR(fIq);
            p->tControl.tCurrentReference.qD = FOC_ZERO;

            foc_pid_Reset(&s_tPidId);
            foc_pid_Reset(&s_tPidIq);

            MLOGF(I, "Current loop run: Iq ref = %.3f pu, Speed = %.2f Hz\r\n", (double)fIq, (double)fSpeed);
        } else {
            MLOG(I, "Usage: motor current <iq-pu> [speed-hz]\r\n");
        }
    } else if (strncmp(args, "pid", 3) == 0) {
        float fKp = 0.0f;
        float fKi = 0.0f;
        if (sscanf(args + 3, "%f %f", &fKp, &fKi) == 2) {
            (void)foc_gain_from_float(fKp, &s_tPidId.tParams.tKp);
            (void)foc_gain_from_float(fKi, &s_tPidId.tParams.tKiTs);
            (void)foc_gain_from_float(fKp, &s_tPidIq.tParams.tKp);
            (void)foc_gain_from_float(fKi, &s_tPidIq.tParams.tKiTs);
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
        MLOGF(I, " - Calib: U=%lu V=%lu W=%lu done=%d\r\n",
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetU,
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetV,
              (unsigned long)tSnapshot.tCurrentCalibration.wOffsetW,
              (int)tSnapshot.tCurrentCalibration.bIsCalibrated);
    } else {
        MLOG(I, "Usage: motor <start|stop|clear|vq <x>|static <t> <v>|current <x>|pid <p> <i>|status>\r\n");
    }
}
MODUS_SHELL_CMD(motor, cmd_motor, "Control FOC Motor (start/stop/clear/vq/static/current/pid/status)");
