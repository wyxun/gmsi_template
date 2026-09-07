#include <stdio.h>
#include <string.h>

#include "foc_app.h"
#include "foc_port.h"

/* ---------------------------------------------------------------------------
 * ops stub：记录调用序列并可按场景注入失败
 * ---------------------------------------------------------------- */
typedef struct {
    uint32_t wCalibrationTicks;     /* 校准进行到的拍数 */
    uint32_t wCalibrationLimit;     /* 多少拍后完成（默认 512） */
    bool bCalibrationFailed;        /* 强制校准失败 */
    bool bSampleFailed;             /* 强制采样失败 */
    bool bDutyCommitFailed;         /* 强制提交失败 */
    bool bPwmEnableFailed;          /* 强制使能失败 */
    bool bEncoderValid;             /* 位置反馈是否有效 */
    bool bPwmEnabled;
    uint32_t wPwmEnableCalls;
    uint32_t wPwmDisableCalls;
    uint32_t wEmergencyStopCalls;
    uint32_t wCalibrationBeginCalls;
    uint32_t wDutyCommitCalls;
    foc_duty_abc_t tLastDuty;
    foc_adc_calib_t tCalibration;
} fake_port_t;

static fake_port_t s_tFakePort;
static uint8_t s_chRingA[64];
static uint8_t s_chRingB[64];
static foc_app_t s_tAppA;
static foc_app_t s_tAppB;

/* 传感器桩：foc_sensor_ops_t 契约——sensor 只返回机械角/速（turn / turn/s），
   电角换算在 app 的 AngleStep（×Pp + 零位）。stub 固定返回机械角 0.5 turn。 */
static int32_t stub_sensor_update(void *pPriv)
{
    (void)pPriv;
    return 0;   /* 1 kHz 慢侧：无 I2C，恒成功 */
}

static foc_result_t stub_sensor_read(void *pPriv,
                                     foc_angle_t *ptMechanicalAngle,
                                     foc_scalar_t *pqMechanicalSpeed,
                                     bool *pbValid)
{
    (void)pPriv;
    if (ptMechanicalAngle == NULL || pqMechanicalSpeed == NULL ||
        pbValid == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptMechanicalAngle = foc_angle_from_turns(0.5f);
    *pqMechanicalSpeed = FOC_ZERO;
    *pbValid = s_tFakePort.bEncoderValid;
    return FOC_RESULT_OK;
}

static const foc_sensor_ops_t s_tTestSensorOps = {
    .fnUpdate    = stub_sensor_update,
    .fnRead      = stub_sensor_read,
    .fnCalibrate = NULL,   /* 标定由 app 非阻塞流程承担 */
};

static const foc_sensor_t g_tTestSensor = {
    .ptOps = &s_tTestSensorOps,
    .pPriv = NULL,
};

/* foc_app_Init 仅当 cfg 未提供 sensor 时才回退到板级默认（foc_port_SensorInit
 * + g_tFocSensor）；测试已注入 g_tTestSensor 不触发该路径，但符号仍需满足
 * 链接，故提供空实现。 */
const foc_sensor_t g_tFocSensor = {
    .ptOps = NULL,
    .pPriv = NULL,
};

int32_t foc_port_SensorInit(const foc_encoder_params_t *ptParams)
{
    (void)ptParams;
    return 0;
}

static void fake_port_reset(void);

static foc_app_cfg_t test_make_config(uint8_t *pchRing)
{
    foc_app_cfg_t tConfig = {
        .tCurrentPiParams = {
            .tKp = {0, FOC_SCALAR(0.20f)},
            .tKiTs = {0, FOC_SCALAR(0.005f)},
            .tKdOverTs = {0, FOC_ZERO},
            .qOutputMinimum = FOC_SCALAR(-0.55f),
            .qOutputMaximum = FOC_SCALAR(0.55f),
            .qIntegratorMinimum = FOC_SCALAR(-0.50f),
            .qIntegratorMaximum = FOC_SCALAR(0.50f),
        },
        .tSpeedPiParams = {
            .tKp = {0, FOC_SCALAR(0.20f)},
            .tKiTs = {0, FOC_SCALAR(0.005f)},
            .tKdOverTs = {0, FOC_ZERO},
            .qOutputMinimum = FOC_SCALAR(-0.10f),
            .qOutputMaximum = FOC_SCALAR(0.10f),
            .qIntegratorMinimum = FOC_SCALAR(-0.10f),
            .qIntegratorMaximum = FOC_SCALAR(0.10f),
        },
        .tEncoderParams = {
            .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
            .hwInvalidTimeout = 100U,
            .chPolePairs = 7U,
            .qHighFrequencyPeriod = FOC_SCALAR(0.00005f),
        },
        .tElectricalZero = {0U},
        .ptPwmOps = &g_tFocPwmOps,
        .ptAdcOps = &g_tFocAdcOps,
        .tSensor = g_tTestSensor,
        .chFeedbackSlot = 0U,
        .pchRingBuffer = pchRing,
        .hwRingSize = 64U,
        .bDirectionInverted = false,
    };

    return tConfig;
}

static int test_init_app(foc_app_t *ptApp, uint8_t *pchRing)
{
    foc_app_cfg_t tConfig = test_make_config(pchRing);

    fake_port_reset();
    return foc_app_Init((uintptr_t)ptApp, (uintptr_t)&tConfig);
}

static void fake_port_reset(void)
{
    memset(&s_tFakePort, 0, sizeof(s_tFakePort));
    s_tFakePort.wCalibrationLimit = 512U;
    s_tFakePort.bEncoderValid = true;
    s_tFakePort.tCalibration = (foc_adc_calib_t){
        .wOffsetU = 32768U,
        .wOffsetV = 32768U,
        .wOffsetW = 32768U,
        .bIsCalibrated = true,
    };
}

/* ===== ops stub 实现 ===== */

static foc_result_t stub_duty_commit(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    s_tFakePort.wDutyCommitCalls++;
    s_tFakePort.tLastDuty = *ptDuty;
    return s_tFakePort.bDutyCommitFailed ? FOC_RESULT_SAFETY
                                         : FOC_RESULT_OK;
}

static foc_result_t stub_pwm_enable(bool bEnable)
{
    if (bEnable) {
        s_tFakePort.wPwmEnableCalls++;
        s_tFakePort.bPwmEnabled = true;
    } else {
        s_tFakePort.wPwmDisableCalls++;
        s_tFakePort.bPwmEnabled = false;
    }
    return s_tFakePort.bPwmEnableFailed ? FOC_RESULT_SAFETY
                                        : FOC_RESULT_OK;
}

static void stub_emergency_stop(void)
{
    s_tFakePort.wEmergencyStopCalls++;
    s_tFakePort.bPwmEnabled = false;
}

static void stub_calibration_begin(foc_adc_calib_t *ptCalibration)
{
    if (ptCalibration != NULL) {
        ptCalibration->ullSumU = 0U;
        ptCalibration->ullSumV = 0U;
        ptCalibration->ullSumW = 0U;
        ptCalibration->hwSampleCount = 0U;
        ptCalibration->bIsCalibrated = false;
    }
    s_tFakePort.wCalibrationTicks = 0U;
    s_tFakePort.wCalibrationBeginCalls++;
    s_tFakePort.tCalibration.bIsCalibrated = false;
}

static foc_calibration_state_e stub_calibration_step(
    foc_adc_calib_t *ptCalibration)
{
    if (ptCalibration == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    if (s_tFakePort.bCalibrationFailed) {
        return FOC_CALIBRATION_FAILED;
    }
    s_tFakePort.wCalibrationTicks++;
    if (s_tFakePort.wCalibrationTicks >=
        s_tFakePort.wCalibrationLimit) {
        *ptCalibration = s_tFakePort.tCalibration;
        return FOC_CALIBRATION_COMPLETE;
    }
    return FOC_CALIBRATION_BUSY;
}

static foc_result_t stub_current_sample(
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    if (ptCalibration == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (s_tFakePort.bSampleFailed) {
        return FOC_RESULT_SAFETY;
    }
    ptInput->qIu = FOC_ZERO;
    ptInput->qIv = FOC_ZERO;
    ptInput->qIw = FOC_ZERO;
    return FOC_RESULT_OK;
}

const foc_pwm_ops_t g_tFocPwmOps = {
    .fnDutyCommit    = stub_duty_commit,
    .fnPwmEnable     = stub_pwm_enable,
    .fnEmergencyStop = stub_emergency_stop,
};

const foc_adc_ops_t g_tFocAdcOps = {
    .fnCalibrationBegin = stub_calibration_begin,
    .fnCalibrationStep  = stub_calibration_step,
    .fnCurrentSample    = stub_current_sample,
};

/* ---------------------------------------------------------------------------
 * 测试辅助
 * ---------------------------------------------------------------- */
static void run_isr(foc_app_t *ptApp, uint32_t wTicks)
{
    uint32_t wIndex = 0U;

    for (wIndex = 0U; wIndex < wTicks; wIndex++) {
        foc_app_HighFrequencyStep(ptApp);
    }
}

static int check_state(const foc_app_t *ptApp,
                       foc_run_state_e eExpected,
                       bool bExpectedPwm)
{
    foc_status_t tStatus;
    int nResult = 0;

    if (foc_app_GetStatus(ptApp, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    if (tStatus.eState != eExpected) {
        nResult++;
    }
    if (tStatus.bPwmEnabled != bExpectedPwm) {
        nResult++;
    }
    return nResult;
}

/* §7 不变量 1/2：IDLE、CALIBRATING 下 PWM 关闭，不提交工作 duty */
static int test_idle_and_calibrating_no_pwm(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
        .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (check_state(&s_tAppA, FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    /* START must arm the ADC trigger before the ISR consumes the command. */
    if (s_tFakePort.wCalibrationBeginCalls != 1U) {
        return 1;
    }
    run_isr(&s_tAppA, 10U);   /* 进入 CALIBRATING 且未完成 */
    if (check_state(&s_tAppA, FOC_STATE_CALIBRATING, false) != 0) {
        return 1;
    }
    /* 校准期间只累加零偏，不提交 duty、不使能 PWM */
    if (s_tFakePort.wDutyCommitCalls != 0U ||
        s_tFakePort.wPwmEnableCalls != 0U) {
        return 1;
    }
    return s_tFakePort.wCalibrationBeginCalls == 1U ? 0 : 1;
}

/* §7 不变量 3/8：512 拍后才提交中性 duty，随后使能 PWM → RUNNING */
static int test_calibration_completes_to_running(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 511U);
    if (check_state(&s_tAppA, FOC_STATE_CALIBRATING, false) != 0) {
        return 1;
    }
    if (s_tFakePort.wPwmEnableCalls != 0U) {
        return 1;
    }
    run_isr(&s_tAppA, 1U);    /* 第 512 拍完成 */
    if (check_state(&s_tAppA, FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    /* 中性 duty 先提交、PWM 后使能，且中性 duty = 0.5/0.5/0.5 */
    if (s_tFakePort.wDutyCommitCalls < 1U ||
        s_tFakePort.wPwmEnableCalls != 1U) {
        return 1;
    }
    return (s_tFakePort.tLastDuty.qU == FOC_HALF &&
            s_tFakePort.tLastDuty.qV == FOC_HALF &&
            s_tFakePort.tLastDuty.qW == FOC_HALF) ? 0 : 1;
}

/* §4.1：校准失败 → 先急停再 FAULT */
static int test_calibration_failure_faults(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    s_tFakePort.bCalibrationFailed = true;
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 600U);
    if (check_state(&s_tAppA, FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    return s_tFakePort.wEmergencyStopCalls >= 1U ? 0 : 1;
}

/* §4.1：校准超时（>2000 拍）→ 先急停再 FAULT */
static int test_calibration_timeout_faults(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    s_tFakePort.wCalibrationLimit = 100000U;
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 2001U);
    if (check_state(&s_tAppA, FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    return s_tFakePort.wEmergencyStopCalls >= 1U ? 0 : 1;
}

/* §7 不变量 5：运行中采样失败 → 先急停再 FAULT */
static int test_running_sample_failure_faults(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    if (check_state(&s_tAppA, FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    s_tFakePort.bSampleFailed = true;
    run_isr(&s_tAppA, 1U);
    if (check_state(&s_tAppA, FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    return s_tFakePort.wEmergencyStopCalls >= 1U ? 0 : 1;
}

/* §7 不变量 5：运行中提交失败 → 先急停再 FAULT */
static int test_running_duty_failure_faults(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    if (check_state(&s_tAppA, FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    s_tFakePort.bDutyCommitFailed = true;
    run_isr(&s_tAppA, 1U);
    if (check_state(&s_tAppA, FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    return s_tFakePort.wEmergencyStopCalls >= 1U ? 0 : 1;
}

/* §7 不变量 6：STOP 任何状态先急停再收敛到 IDLE */
static int test_stop_emergency_and_idle(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    if (check_state(&s_tAppA, FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    foc_app_Stop(&s_tAppA);
    if (s_tFakePort.wEmergencyStopCalls < 1U) {
        return 1;   /* Stop 立即急停，不等 ISR */
    }
    run_isr(&s_tAppA, 1U);
    if (check_state(&s_tAppA, FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    /* 校准超时看门狗不得在 IDLE 误触发 */
    s_tFakePort.wCalibrationLimit = 100000U;
    run_isr(&s_tAppA, 3000U);
    return check_state(&s_tAppA, FOC_STATE_IDLE, false) == 0 ? 0 : 1;
}

/* §4.1：FAULT 只能 CLEAR_FAULT 回到 IDLE，START 在非 IDLE 拒绝 */
static int test_fault_clear_and_start_reject(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    s_tFakePort.bSampleFailed = true;
    run_isr(&s_tAppA, 1U);
    if (check_state(&s_tAppA, FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_BUSY) {
        return 1;   /* FAULT 不接受 START */
    }
    if (foc_app_ClearFault(&s_tAppA) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 1U);
    if (check_state(&s_tAppA, FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    /* CLEAR_FAULT 后可以重新 START */
    s_tFakePort.bSampleFailed = false;
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    return check_state(&s_tAppA, FOC_STATE_RUNNING, true) == 0 ? 0 : 1;
}

/* §4.2：控制模式在一次 RUNNING 期间固定，引用 setter 按模式校验 */
static int test_reference_setters_mode_check(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
        .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (foc_app_SetVoltageReference(&s_tAppA,
                                    FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_OK) {
        return 1;   /* IDLE 下允许设置 */
    }
    if (foc_app_SetCurrentReference(&s_tAppA,
                                    FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;   /* VOLTAGE 模式拒绝电流参考 */
    }
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    if (foc_app_SetCurrentReference(&s_tAppA,
                                    FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    return 0;
}

/* Current mode aligns first, then drives the initialized Iq PI. */
static int test_current_mode_initializes_current_pi(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };
    foc_status_t tStatus;

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 513U);
    if (foc_app_GetStatus(&s_tAppA, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    if (tStatus.eState != FOC_STATE_RUNNING ||
        tStatus.tVoltage.qD < FOC_SCALAR(0.03f) ||
        tStatus.qElectricalSpeed != FOC_ZERO) {
        return 1;
    }
    run_isr(&s_tAppA, 20001U); /* 1 s alignment, then one ramp step */
    if (foc_app_GetStatus(&s_tAppA, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    return (tStatus.eState == FOC_STATE_RUNNING &&
            tStatus.tVoltage.qQ > FOC_ZERO &&
            tStatus.tVoltage.qD < FOC_SCALAR(0.02f)) ? 0 : 1;
}

/* 已校准编码器时，CURRENT 启动不得再次施加开环对齐电压。 */
static int test_calibrated_current_uses_encoder_angle(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };
    foc_status_t tStatus;

    foc_app_TestMarkEncoderCalibrated(&s_tAppA);
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 513U);
    if (foc_app_GetStatus(&s_tAppA, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    return (tStatus.eState == FOC_STATE_RUNNING &&
            tStatus.tVoltage.qD < FOC_SCALAR(0.02f)) ? 0 : 1;
}

/* CURRENT mode must keep the requested Iq; no speed loop may overwrite it. */
static int test_current_mode_keeps_fixed_iq(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };
    foc_scalar_t qIqBefore;
    foc_scalar_t qIqAfter;

    foc_app_TestMarkEncoderCalibrated(&s_tAppA);
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 512U);
    qIqBefore = foc_app_TestGetCurrentIqReference(&s_tAppA);
    foc_app_TestRun1kHz(&s_tAppA);
    qIqAfter = foc_app_TestGetCurrentIqReference(&s_tAppA);
    return (qIqBefore == FOC_SCALAR(0.05f) &&
            qIqAfter == FOC_SCALAR(0.05f)) ? 0 : 1;
}

/* START 必须清除上一次 CURRENT 运行残留的电流 PI 积分。 */
static int test_current_start_resets_current_pi(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };
    foc_status_t tStatus;

    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 513U);
    run_isr(&s_tAppA, 25000U);
    foc_app_Stop(&s_tAppA);
    run_isr(&s_tAppA, 1U);
    if (foc_app_Start(&s_tAppA, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(&s_tAppA, 20512U);
    if (foc_app_GetStatus(&s_tAppA, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    return tStatus.tVoltage.qD < FOC_SCALAR(0.02f) ? 0 : 1;
}

/* §4.2/§4.3：Start 只接受合法模式，NULL 参数拒绝 */
static int test_start_validation(void)
{
    foc_core_command_t tCommand = {
        .eMode = (foc_control_mode_e)99,
    };

    if (foc_app_Start(&s_tAppA, NULL) != FOC_RESULT_NULL) {
        return 1;
    }
    return foc_app_Start(&s_tAppA, &tCommand) == FOC_RESULT_INVALID_ARGUMENT
               ? 0 : 1;
}

/* New Class contract: App state must be supplied by the caller. */
static int test_object_api_is_explicit(void)
{
    foc_app_cfg_t tConfig = test_make_config(s_chRingB);
    foc_status_t tStatus;

    if (foc_app_Init((uintptr_t)&s_tAppB,
                     (uintptr_t)&tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    if (foc_app_GetStatus(&s_tAppB, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    return tStatus.eState == FOC_STATE_IDLE ? 0 : 1;
}

/* Class initialization must reject missing objects and invalid dependencies. */
static int test_init_validation(void)
{
    foc_app_cfg_t tConfig = test_make_config(s_chRingA);
    int nFailures = 0;

    if (foc_app_Init(0U, (uintptr_t)&tConfig) != FOC_RESULT_NULL) {
        nFailures++;
    }
    if (foc_app_Init((uintptr_t)&s_tAppA, 0U) != FOC_RESULT_NULL) {
        nFailures++;
    }
    tConfig.tEncoderParams.chPolePairs = 0U;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.pchRingBuffer = NULL;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.hwRingSize = 0U;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.ptPwmOps = NULL;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.ptAdcOps = NULL;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.tEncoderParams.hwInvalidTimeout = 0U;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.tEncoderParams.qHighFrequencyPeriod = FOC_ZERO;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.tEncoderParams.qSpeedFilterAlpha =
        FOC_SCALAR(1.01f);
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    tConfig = test_make_config(s_chRingA);
    tConfig.tCurrentPiParams.qOutputMinimum =
        FOC_SCALAR(0.60f);   /* >= qOutputMaximum: PID init must reject */
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    return nFailures;
}

/* Every failed init must leave the power stage disabled (PWM off). */
static int test_init_failure_keeps_pwm_off(void)
{
    foc_app_cfg_t tConfig = test_make_config(s_chRingA);
    int nFailures = 0;

    fake_port_reset();
    s_tFakePort.wEmergencyStopCalls = 0U;
    tConfig.pchRingBuffer = NULL;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    if (s_tFakePort.wEmergencyStopCalls == 0U) {
        nFailures++;   /* failed init must call the emergency stop */
    }
    if (s_tFakePort.bPwmEnabled) {
        nFailures++;   /* no PWM may stay enabled after a failed init */
    }
    fake_port_reset();
    s_tFakePort.wEmergencyStopCalls = 0U;
    tConfig = test_make_config(s_chRingA);
    tConfig.tCurrentPiParams.qOutputMinimum =
        FOC_SCALAR(0.60f);
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) != FOC_RESULT_INVALID_ARGUMENT) {
        nFailures++;
    }
    if (s_tFakePort.wEmergencyStopCalls == 0U) {
        nFailures++;   /* PID-rejected init must also stop the stage */
    }
    return nFailures;
}

int main(void)
{
    int nFailures = 0;
    int nSub = 0;

    fake_port_reset();
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_idle_and_calibrating_no_pwm();
    printf("  idle_no_pwm: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_calibration_completes_to_running();
    printf("  calib_to_running: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_calibration_failure_faults();
    printf("  calib_failure: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_calibration_timeout_faults();
    printf("  calib_timeout: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_running_sample_failure_faults();
    printf("  sample_failure: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_running_duty_failure_faults();
    printf("  duty_failure: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_stop_emergency_and_idle();
    printf("  stop_emergency: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_fault_clear_and_start_reject();
    printf("  fault_clear: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_reference_setters_mode_check();
    printf("  ref_setters: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_current_mode_initializes_current_pi();
    printf("  current_pi: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_calibrated_current_uses_encoder_angle();
    printf("  calibrated_current: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_current_mode_keeps_fixed_iq();
    printf("  current_fixed_iq: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_current_start_resets_current_pi();
    printf("  current_reset_pi: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    test_init_app(&s_tAppA, s_chRingA);
    nSub = test_start_validation();
    printf("  start_validation: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_object_api_is_explicit();
    printf("  object_api: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_init_validation();
    printf("  init_validation: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_init_failure_keeps_pwm_off();
    printf("  init_failure_pwm: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    printf("minimal lifecycle: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
