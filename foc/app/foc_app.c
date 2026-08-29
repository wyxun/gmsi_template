/****************************************************************************
 * @file    foc_app.c
 * @brief   Minimal single-motor FOC application (unique runtime + lifecycle)
 *
 * 唯一 foc_runtime_t 实例；四态生命周期 IDLE/CALIBRATING/RUNNING/FAULT；
 * 20 kHz ISR 直接链接 foc_port → foc_encoder → foc_core → foc_port，
 * 1 kHz Clock 运行速度 PI 并更新 Iq 参考。硬件只在 foc_port 层访问。
 ****************************************************************************/

#include <stddef.h>
#include <string.h>

#include "foc_app.h"
#include "foc_port.h"
#include "foc_encoder.h"
#include "foc_core.h"
#include "foc_pid.h"
#include "perf_counter.h"
#include "mdi_hw.h"
#include "mdebug/util_debug.h"

#if defined(MDI_HW_HAS_I2C_ENCODER)
#include "as5600.h"
#endif

#if defined(MODUS_ENABLE) && MODUS_ENABLE
#include "modus.h"
#include "mdebug/mshell.h"
#endif

/* 校准超时：2000 拍 × 50 us = 100 ms（§4.2 不用毫秒时钟） */
#define FOC_APP_CALIBRATION_MAX_TICKS  2000U
#define FOC_APP_POLE_PAIRS             7U
#define FOC_APP_HF_PERIOD_S            0.00005f

/* 故障位定义（wFaults 低 16 位为产品故障，高 16 位保留） */
#define FOC_FAULT_CALIBRATION_TIMEOUT  (1UL << 0)
#define FOC_FAULT_CALIBRATION          (1UL << 1)
#define FOC_FAULT_CURRENT_SAMPLE       (1UL << 2)
#define FOC_FAULT_ANGLE                (1UL << 3)
#define FOC_FAULT_MATH                 (1UL << 4)
#define FOC_FAULT_DUTY_COMMIT          (1UL << 5)
#define FOC_FAULT_PWM_ENABLE           (1UL << 6)

typedef struct {
    foc_core_state_t   tCore;
    foc_core_command_t tCommand;
    foc_pid_t          tSpeedPi;
    foc_encoder_t      tEncoder;
    foc_adc_calib_t    tCalibration;
    foc_angle_t        tMechanicalZero;
    foc_scalar_t       qMechanicalSpeed;
    foc_scalar_t       qSpeedReference;
#if defined(FOC_NUMERIC_FLOAT)
    float              fElectricalAngleTurns;
#endif
    uint32_t           wFaults;
    uint16_t           hwCalibrationTicks;
    uint8_t            chPolePairs;
    uint8_t            chPendingCommand;
    foc_run_state_e    eState;
    bool               bDirectionInverted;
    bool               bPwmEnabled;
} foc_runtime_t;

static foc_runtime_t s_tFoc;

#if defined(MDI_HW_HAS_I2C_ENCODER)
static as5600_t s_tAs5600;
#endif

/* ---------------------------------------------------------------------------
 * 故障收敛：第一动作急停，再置状态（§7 不变量 4/5/6）
 * ---------------------------------------------------------------- */
static void foc_app_EnterFault(uint32_t wFault)
{
    foc_port_EmergencyStop();
    s_tFoc.bPwmEnabled = false;
    s_tFoc.wFaults |= wFault;
    s_tFoc.eState = FOC_STATE_FAULT;
}

static bool foc_app_ModeValid(foc_control_mode_e eMode)
{
    return eMode <= FOC_MODE_SPEED;
}

/* ---------------------------------------------------------------------------
 * 命令邮箱：Shell/按键写 chPendingCommand（中断守卫），ISR 消费
 * ---------------------------------------------------------------- */
static void foc_app_PostCommand(foc_command_e eCommand)
{
    uintptr_t wState = perfc_port_disable_global_interrupt();

    s_tFoc.chPendingCommand = (uint8_t)eCommand;
    perfc_port_resume_global_interrupt(wState);
}

static void foc_app_ConsumeCommand(void)
{
    foc_command_e eCommand =
        (foc_command_e)s_tFoc.chPendingCommand;

    if (eCommand == FOC_COMMAND_NONE) {
        return;
    }
    s_tFoc.chPendingCommand = (uint8_t)FOC_COMMAND_NONE;
    switch (eCommand) {
    case FOC_COMMAND_START:
        if (s_tFoc.eState == FOC_STATE_IDLE) {
            foc_port_CurrentCalibrationBegin();
            s_tFoc.hwCalibrationTicks = 0U;
            s_tFoc.eState = FOC_STATE_CALIBRATING;
        }
        break;
    case FOC_COMMAND_STOP:
        foc_port_EmergencyStop();
        s_tFoc.bPwmEnabled = false;
        s_tFoc.eState = FOC_STATE_IDLE;
        break;
    case FOC_COMMAND_CLEAR_FAULT:
        if (s_tFoc.eState == FOC_STATE_FAULT &&
            !s_tFoc.bPwmEnabled) {
            s_tFoc.wFaults = 0U;
            s_tFoc.eState = FOC_STATE_IDLE;
        }
        break;
    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * 校准：512 拍累计零偏，超时或偏移非法进入 FAULT（§3.2/§4.1）
 * ---------------------------------------------------------------- */
static void foc_app_CalibrationStep(void)
{
    foc_calibration_state_e eCalib;
    foc_result_t eResult;

    if (s_tFoc.hwCalibrationTicks < UINT16_MAX) {
        s_tFoc.hwCalibrationTicks++;
    }
    eCalib = foc_port_CurrentCalibrationStep(&s_tFoc.tCalibration);
    if (eCalib == FOC_CALIBRATION_BUSY) {
        if (s_tFoc.hwCalibrationTicks >= FOC_APP_CALIBRATION_MAX_TICKS) {
            foc_app_EnterFault(FOC_FAULT_CALIBRATION_TIMEOUT);
        }
        return;
    }
    if (eCalib != FOC_CALIBRATION_COMPLETE) {
        foc_app_EnterFault(FOC_FAULT_CALIBRATION);
        return;
    }
    /* 清 PI → 中性 duty 先提交 → 再使能 PWM（§7 不变量 7/8） */
    foc_core_Reset(&s_tFoc.tCore);
    eResult = foc_port_DutyCommit(&s_tFoc.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        foc_app_EnterFault(FOC_FAULT_DUTY_COMMIT);
        return;
    }
    eResult = foc_port_PwmEnable(true);
    if (eResult != FOC_RESULT_OK) {
        foc_app_EnterFault(FOC_FAULT_PWM_ENABLE);
        return;
    }
    s_tFoc.bPwmEnabled = true;
    s_tFoc.eState = FOC_STATE_RUNNING;
}

/* ---------------------------------------------------------------------------
 * 角度路径：AS5600 缓存 → foc_encoder_Step → 机械角换算电角
 * ---------------------------------------------------------------- */
static void foc_app_AngleStep(foc_core_input_t *ptInput)
{
#if defined(MDI_HW_HAS_I2C_ENCODER)
    as5600_sample_t tSample;
    foc_encoder_sample_t tEncSample;
    foc_encoder_output_t tEncOutput;
    uint64_t llElectrical;

    as5600_GetSample(&s_tAs5600, &tSample);
    if (!tSample.bValid) {
        ptInput->bAngleValid = false;
        return;
    }
    tEncSample.hwRawAngle = tSample.hwRawAngle;
    tEncSample.wSequence = tSample.wSequence;
    tEncSample.bMagnetOk = tSample.bMagnetOk;
    if (foc_encoder_Step(&s_tFoc.tEncoder, &tEncSample,
                         &tEncOutput) != FOC_RESULT_OK) {
        ptInput->bAngleValid = false;
        return;
    }
    s_tFoc.qMechanicalSpeed = tEncOutput.qMechanicalSpeed;
    ptInput->bAngleValid = s_tFoc.tEncoder.bValid;
    if (!ptInput->bAngleValid) {
        return;
    }
    /* 机械角 × 极对数（BAM32 模 2^32），方向反转，叠加电气零位 */
    llElectrical = (uint64_t)tEncOutput.tMechanicalAngle.wBam32 *
                   (uint64_t)s_tFoc.chPolePairs;
    s_tFoc.tCore.tElectricalAngle.wBam32 = (uint32_t)llElectrical;
    if (s_tFoc.bDirectionInverted) {
        s_tFoc.tCore.tElectricalAngle.wBam32 =
            (uint32_t)(0U - s_tFoc.tCore.tElectricalAngle.wBam32);
    }
    s_tFoc.tCore.tElectricalAngle = foc_angle_add(
        s_tFoc.tCore.tElectricalAngle, s_tFoc.tMechanicalZero);
    ptInput->tElectricalAngle = s_tFoc.tCore.tElectricalAngle;
    ptInput->qElectricalSpeed = foc_mul_wide(
        tEncOutput.qMechanicalSpeed,
        FOC_SCALAR((float)s_tFoc.chPolePairs));
    s_tFoc.tCore.qElectricalSpeed = ptInput->qElectricalSpeed;
#else
    (void)ptInput;
    ptInput->bAngleValid = false;
#endif
}

/* ---------------------------------------------------------------------------
 * RUNNING 高频链路：采样 → 角度 → 核心 → 提交（§1.3）
 * ---------------------------------------------------------------- */
static void foc_app_RunningStep(foc_core_input_t *ptInput)
{
    foc_result_t eResult;

    eResult = foc_port_CurrentSample(&s_tFoc.tCalibration, ptInput);
    if (eResult != FOC_RESULT_OK) {
        foc_app_EnterFault(FOC_FAULT_CURRENT_SAMPLE);
        return;
    }
    foc_app_AngleStep(ptInput);
    if (!ptInput->bAngleValid) {
        foc_app_EnterFault(FOC_FAULT_ANGLE);
        return;
    }
    eResult = foc_core_step(&s_tFoc.tCore, &s_tFoc.tCommand, ptInput);
    if (eResult != FOC_RESULT_OK) {
        foc_app_EnterFault(FOC_FAULT_MATH);
        return;
    }
    eResult = foc_port_DutyCommit(&s_tFoc.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        foc_app_EnterFault(FOC_FAULT_DUTY_COMMIT);
        return;
    }
#if defined(FOC_NUMERIC_FLOAT)
    /* 唯一展示变量：BAM32 角度无法按 FLOAT 注册波形（§6） */
    s_tFoc.fElectricalAngleTurns =
        foc_angle_to_turns(s_tFoc.tCore.tElectricalAngle);
#endif
}

void foc_app_HighFrequencyISR(void)
{
    foc_core_input_t tInput = {0};

    foc_app_ConsumeCommand();
    switch (s_tFoc.eState) {
    case FOC_STATE_CALIBRATING:
        foc_app_CalibrationStep();
        break;
    case FOC_STATE_RUNNING:
        foc_app_RunningStep(&tInput);
        break;
    case FOC_STATE_IDLE:
    case FOC_STATE_FAULT:
    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * 1 kHz 速度环：电速度反馈 → Speed PI → Iq 参考（中断守卫更新）
 * ---------------------------------------------------------------- */
#if defined(MODUS_ENABLE) && MODUS_ENABLE
static void foc_app_SpeedLoop(void)
{
    foc_scalar_t qSpeed;
    foc_scalar_t qIqRef;
    uintptr_t wState;

    wState = perfc_port_disable_global_interrupt();
    qSpeed = s_tFoc.tCore.qElectricalSpeed;
    perfc_port_resume_global_interrupt(wState);

    qIqRef = foc_pid_Step(&s_tFoc.tSpeedPi, s_tFoc.qSpeedReference,
                          qSpeed);
    wState = perfc_port_disable_global_interrupt();
    s_tFoc.tCommand.tCurrentReference.qQ = qIqRef;
    perfc_port_resume_global_interrupt(wState);
}
#endif

#if defined(MODUS_ENABLE) && MODUS_ENABLE
static modus_base_t s_tFocAppBase;

static int foc_app_Clock(uintptr_t wObjectAddr)
{
    (void)wObjectAddr;
#if defined(MDI_HW_HAS_I2C_ENCODER)
    (void)as5600_Update(&s_tAs5600);
#endif
    if (s_tFoc.eState == FOC_STATE_RUNNING &&
        s_tFoc.tCommand.eMode == FOC_MODE_SPEED) {
        foc_app_SpeedLoop();
    }
    return MODUS_SUCCESS;
}

static modus_base_cfg_t s_tFocAppBaseCfg = {
    .wId = FOC_APP,
    .wParent = 0,
    .FcnInterface = {
        .Clock = foc_app_Clock,
        .Run = NULL,
    },
};

/* MODUS 对象占位：runtime 为文件内静态 s_tFoc，对象只承载
 * init_info 自动注册（modus_Init → foc_app_Init）。 */
typedef struct {
    uint8_t chReserved;
} foc_app_cfg_t;

typedef struct {
    uint8_t chReserved;
} foc_app_t;

MODUS_DECLARE_OBJECT(foc_app, FocApp, .chReserved = 0U);
#endif

/* ---------------------------------------------------------------------------
 * 对外 API
 * ---------------------------------------------------------------- */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    const foc_pid_params_t tSpeedParams = {
        .tKp = {0, FOC_SCALAR(0.2f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.10f),
        .qOutputMaximum = FOC_SCALAR(0.10f),
        .qIntegratorMinimum = FOC_SCALAR(-0.10f),
        .qIntegratorMaximum = FOC_SCALAR(0.10f),
    };
    foc_encoder_params_t tEncoderParams;

    (void)wObjectAddr;
    (void)wObjectCfgAddr;

    memset(&s_tFoc, 0, sizeof(s_tFoc));
    s_tFoc.eState = FOC_STATE_IDLE;
    s_tFoc.chPolePairs = FOC_APP_POLE_PAIRS;
    s_tFoc.tCommand.eMode = FOC_MODE_VOLTAGE;
    foc_core_Reset(&s_tFoc.tCore);
    (void)foc_pid_Init(&s_tFoc.tSpeedPi, &tSpeedParams);

    foc_encoder_DefaultParams(&tEncoderParams);
    tEncoderParams.chPolePairs = s_tFoc.chPolePairs;
    tEncoderParams.qHighFrequencyPeriod =
        FOC_SCALAR(FOC_APP_HF_PERIOD_S);
    (void)foc_encoder_Init(&s_tFoc.tEncoder, &tEncoderParams);

    foc_port_Init();
    foc_port_EmergencyStop();   /* 启动即保证功率级关闭 */

#if defined(MDI_HW_HAS_I2C_ENCODER)
    (void)as5600_Init(&s_tAs5600, HW.ptI2c1);
#endif

#if defined(MODUS_ENABLE) && MODUS_ENABLE
    return mbase_Init(&s_tFocAppBase, &s_tFocAppBaseCfg);
#else
    return 0;
#endif
}

foc_result_t foc_app_Start(const foc_core_command_t *ptCommand)
{
    uintptr_t wState;

    if (ptCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_tFoc.eState != FOC_STATE_IDLE) {
        return FOC_RESULT_BUSY;
    }
    if (!foc_app_ModeValid(ptCommand->eMode)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wState = perfc_port_disable_global_interrupt();
    s_tFoc.tCommand = *ptCommand;
    perfc_port_resume_global_interrupt(wState);
    foc_app_PostCommand(FOC_COMMAND_START);
    return FOC_RESULT_OK;
}

void foc_app_Stop(void)
{
    /* 先急停再投递，软件状态在下一个 ISR 边界收敛（§4.2） */
    foc_port_EmergencyStop();
    foc_app_PostCommand(FOC_COMMAND_STOP);
}

foc_result_t foc_app_ClearFault(void)
{
    if (s_tFoc.eState != FOC_STATE_FAULT || s_tFoc.bPwmEnabled) {
        return FOC_RESULT_BUSY;
    }
    foc_app_PostCommand(FOC_COMMAND_CLEAR_FAULT);
    return FOC_RESULT_OK;
}

foc_result_t foc_app_SetVoltageReference(foc_scalar_t qD,
                                         foc_scalar_t qQ)
{
    uintptr_t wState;

    if (s_tFoc.tCommand.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wState = perfc_port_disable_global_interrupt();
    s_tFoc.tCommand.tVoltageReference.qD = qD;
    s_tFoc.tCommand.tVoltageReference.qQ = qQ;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

foc_result_t foc_app_SetCurrentReference(foc_scalar_t qD,
                                         foc_scalar_t qQ)
{
    uintptr_t wState;

    if (s_tFoc.tCommand.eMode != FOC_MODE_CURRENT) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wState = perfc_port_disable_global_interrupt();
    s_tFoc.tCommand.tCurrentReference.qD = qD;
    s_tFoc.tCommand.tCurrentReference.qQ = qQ;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

foc_result_t foc_app_SetSpeedReference(
    foc_scalar_t qMechanicalTurnPerSecond)
{
    uintptr_t wState;

    if (s_tFoc.tCommand.eMode != FOC_MODE_SPEED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wState = perfc_port_disable_global_interrupt();
    s_tFoc.qSpeedReference = qMechanicalTurnPerSecond;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

foc_result_t foc_app_GetStatus(foc_status_t *ptStatus)
{
    uintptr_t wState;

    if (ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    wState = perfc_port_disable_global_interrupt();
    ptStatus->eState = s_tFoc.eState;
    ptStatus->wFaults = s_tFoc.wFaults;
    ptStatus->tElectricalAngle = s_tFoc.tCore.tElectricalAngle;
    ptStatus->qElectricalSpeed = s_tFoc.tCore.qElectricalSpeed;
    ptStatus->tCurrent = s_tFoc.tCore.tCurrent;
    ptStatus->tVoltage = s_tFoc.tCore.tVoltage;
    ptStatus->tDuty = s_tFoc.tCore.tDuty;
    ptStatus->tCalibration = s_tFoc.tCalibration;
    ptStatus->bPwmEnabled = s_tFoc.bPwmEnabled;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

#if defined(MODUS_ENABLE) && MODUS_ENABLE
#include <stdio.h>

/* Shell：极简 motor 控制命令（Task 5 会补充完整 status 输出） */
static void foc_app_CmdMotor(const char *args)
{
    if (strncmp(args, "start", 5) == 0) {
        foc_core_command_t tCommand = {
            .eMode = FOC_MODE_VOLTAGE,
            .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.03f)},
        };
        if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
            MLOG(W, "motor start rejected (not IDLE?)\r\n");
        }
    } else if (strncmp(args, "stop", 4) == 0) {
        foc_app_Stop();
    } else if (strncmp(args, "clear", 5) == 0) {
        if (foc_app_ClearFault() != FOC_RESULT_OK) {
            MLOG(W, "clear rejected (not FAULT or PWM on)\r\n");
        }
    } else if (strncmp(args, "status", 6) == 0) {
        foc_status_t tStatus;
        if (foc_app_GetStatus(&tStatus) == FOC_RESULT_OK) {
            MLOGF(I, "state=%d faults=0x%lX pwm=%d\r\n",
                  (int)tStatus.eState,
                  (unsigned long)tStatus.wFaults,
                  (int)tStatus.bPwmEnabled);
        }
    } else if (strncmp(args, "vq", 2) == 0) {
        float fVq = 0.03f;
        (void)sscanf(args + 2, "%f", &fVq);
        (void)foc_app_SetVoltageReference(FOC_ZERO, FOC_SCALAR(fVq));
    } else {
        MLOG(I, "usage: motor start|stop|clear|status|vq <x>\r\n");
    }
}
MODUS_SHELL_CMD(motor, foc_app_CmdMotor,
                "start/stop/clear/status/vq for FOC motor");
#endif /* MODUS_ENABLE */
