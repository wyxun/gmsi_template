#include <stdio.h>
#include <string.h>

#include "foc_app.h"
#include "foc_port.h"
#include "mdi_hw.h"

/* Host 测试：提供目标侧 extern 的硬件池符号（全部为空指针） */
const mdi_hardware_t HW = {0};

/* ---------------------------------------------------------------------------
 * fake foc_port：记录调用序列并可按场景注入失败
 * ---------------------------------------------------------------- */
typedef struct {
    uint32_t wCalibrationTicks;     /* 校准进行到的拍数 */
    uint32_t wCalibrationLimit;     /* 多少拍后完成（默认 512） */
    bool bCalibrationFailed;        /* 强制校准失败 */
    bool bSampleFailed;             /* 强制采样失败 */
    bool bDutyCommitFailed;         /* 强制提交失败 */
    bool bPwmEnableFailed;          /* 强制使能失败 */
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

static void fake_port_reset(void)
{
    memset(&s_tFakePort, 0, sizeof(s_tFakePort));
    s_tFakePort.wCalibrationLimit = 512U;
    s_tFakePort.tCalibration = (foc_adc_calib_t){
        .wOffsetU = 32768U,
        .wOffsetV = 32768U,
        .wOffsetW = 32768U,
        .bIsCalibrated = true,
    };
}

void foc_port_Init(void)
{
    fake_port_reset();
}

void foc_port_CurrentCalibrationBegin(void)
{
    s_tFakePort.wCalibrationTicks = 0U;
    s_tFakePort.wCalibrationBeginCalls++;
    s_tFakePort.tCalibration.bIsCalibrated = false;
}

foc_calibration_state_e foc_port_CurrentCalibrationStep(
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

foc_result_t foc_port_CurrentSample(const foc_adc_calib_t *ptCalibration,
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

foc_result_t foc_port_DutyCommit(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    s_tFakePort.wDutyCommitCalls++;
    s_tFakePort.tLastDuty = *ptDuty;
    return s_tFakePort.bDutyCommitFailed ? FOC_RESULT_SAFETY
                                         : FOC_RESULT_OK;
}

foc_result_t foc_port_PwmEnable(bool bEnable)
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

void foc_port_EmergencyStop(void)
{
    s_tFakePort.wEmergencyStopCalls++;
    s_tFakePort.bPwmEnabled = false;
}

/* ---------------------------------------------------------------------------
 * as5600 stub：固定有效样本，序列不递增（速度 0，不外推）
 * ---------------------------------------------------------------- */
#include "as5600.h"

int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic)
{
    (void)ptThis;
    (void)ptIic;
    return 0;
}

int32_t as5600_Update(as5600_t *ptThis)
{
    (void)ptThis;
    return 0;
}

void as5600_GetSample(const as5600_t *ptThis, as5600_sample_t *ptSample)
{
    (void)ptThis;
    ptSample->hwRawAngle = 2048U;
    ptSample->wSequence = 1U;
    ptSample->chStatus = 0U;
    ptSample->bMagnetOk = true;
    ptSample->bValid = true;
}

/* ---------------------------------------------------------------------------
 * 测试辅助
 * ---------------------------------------------------------------- */
static void run_isr(uint32_t wTicks)
{
    uint32_t wIndex;

    for (wIndex = 0U; wIndex < wTicks; wIndex++) {
        foc_app_HighFrequencyISR();
    }
}

static int check_state(foc_run_state_e eExpected, bool bExpectedPwm)
{
    foc_status_t tStatus;
    int nResult = 0;

    if (foc_app_GetStatus(&tStatus) != FOC_RESULT_OK) {
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

    if (check_state(FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(10U);   /* 进入 CALIBRATING 且未完成 */
    if (check_state(FOC_STATE_CALIBRATING, false) != 0) {
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

    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(511U);
    if (check_state(FOC_STATE_CALIBRATING, false) != 0) {
        return 1;
    }
    if (s_tFakePort.wPwmEnableCalls != 0U) {
        return 1;
    }
    run_isr(1U);    /* 第 512 拍完成 */
    if (check_state(FOC_STATE_RUNNING, true) != 0) {
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
    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(600U);
    if (check_state(FOC_STATE_FAULT, false) != 0) {
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
    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(2001U);
    if (check_state(FOC_STATE_FAULT, false) != 0) {
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

    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    if (check_state(FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    s_tFakePort.bSampleFailed = true;
    run_isr(1U);
    if (check_state(FOC_STATE_FAULT, false) != 0) {
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

    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    if (check_state(FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    s_tFakePort.bDutyCommitFailed = true;
    run_isr(1U);
    if (check_state(FOC_STATE_FAULT, false) != 0) {
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

    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    if (check_state(FOC_STATE_RUNNING, true) != 0) {
        return 1;
    }
    foc_app_Stop();
    if (s_tFakePort.wEmergencyStopCalls < 1U) {
        return 1;   /* Stop 立即急停，不等 ISR */
    }
    run_isr(1U);
    if (check_state(FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    /* 校准超时看门狗不得在 IDLE 误触发 */
    s_tFakePort.wCalibrationLimit = 100000U;
    run_isr(3000U);
    return check_state(FOC_STATE_IDLE, false) == 0 ? 0 : 1;
}

/* §4.1：FAULT 只能 CLEAR_FAULT 回到 IDLE，START 在非 IDLE 拒绝 */
static int test_fault_clear_and_start_reject(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
    };

    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    s_tFakePort.bSampleFailed = true;
    run_isr(1U);
    if (check_state(FOC_STATE_FAULT, false) != 0) {
        return 1;
    }
    if (foc_app_Start(&tCommand) != FOC_RESULT_BUSY) {
        return 1;   /* FAULT 不接受 START */
    }
    if (foc_app_ClearFault() != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(1U);
    if (check_state(FOC_STATE_IDLE, false) != 0) {
        return 1;
    }
    /* CLEAR_FAULT 后可以重新 START */
    s_tFakePort.bSampleFailed = false;
    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    return check_state(FOC_STATE_RUNNING, true) == 0 ? 0 : 1;
}

/* §4.2：控制模式在一次 RUNNING 期间固定，引用 setter 按模式校验 */
static int test_reference_setters_mode_check(void)
{
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
        .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (foc_app_SetVoltageReference(FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_OK) {
        return 1;   /* IDLE 下允许设置 */
    }
    if (foc_app_SetCurrentReference(FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;   /* VOLTAGE 模式拒绝电流参考 */
    }
    if (foc_app_Start(&tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    run_isr(512U);
    if (foc_app_SetCurrentReference(FOC_ZERO, FOC_SCALAR(0.1f)) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    return 0;
}

/* §4.2/§4.3：Start 只接受合法模式，NULL 参数拒绝 */
static int test_start_validation(void)
{
    foc_core_command_t tCommand = {
        .eMode = (foc_control_mode_e)99,
    };

    if (foc_app_Start(NULL) != FOC_RESULT_NULL) {
        return 1;
    }
    return foc_app_Start(&tCommand) == FOC_RESULT_INVALID_ARGUMENT ? 0 : 1;
}

int main(void)
{
    int nFailures = 0;

    foc_app_Init(0U, 0U);
    nFailures += test_idle_and_calibrating_no_pwm();
    foc_app_Init(0U, 0U);
    nFailures += test_calibration_completes_to_running();
    foc_app_Init(0U, 0U);
    nFailures += test_calibration_failure_faults();
    foc_app_Init(0U, 0U);
    nFailures += test_calibration_timeout_faults();
    foc_app_Init(0U, 0U);
    nFailures += test_running_sample_failure_faults();
    foc_app_Init(0U, 0U);
    nFailures += test_running_duty_failure_faults();
    foc_app_Init(0U, 0U);
    nFailures += test_stop_emergency_and_idle();
    foc_app_Init(0U, 0U);
    nFailures += test_fault_clear_and_start_reject();
    foc_app_Init(0U, 0U);
    nFailures += test_reference_setters_mode_check();
    foc_app_Init(0U, 0U);
    nFailures += test_start_validation();
    printf("minimal lifecycle: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
