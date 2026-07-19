#include "test_common.h"
#include "motor.h"

typedef struct {
    uint32_t wNow;
    unsigned wCalibrateCalls;
    unsigned wEnableCalls;
    unsigned wStopCalls;
    bool bEnabled;
    bool bFailCalibration;
    bool bFailDisable;
    bool bCalibrationWhileEnabled;
    bool bEmergencyDuringCalibration;
    bool bStopDuringCalibration;
    motor_handle_t *ptMotor;
    unsigned wSyncDepth;
    bool bDutyUnderSync;
    bool bEnableUnderSync;
} fake_fsm_hw_t;

static foc_result_t fake_set_duty(void *p, foc_scalar_t u,
                                  foc_scalar_t v, foc_scalar_t w)
{
    fake_fsm_hw_t *hw = p;
    (void)u; (void)v; (void)w;
    hw->bDutyUnderSync = hw->wSyncDepth != 0U;
    return FOC_RESULT_OK;
}

static foc_result_t fake_enable(void *p, bool enable)
{
    fake_fsm_hw_t *hw = p;
    hw->bEnableUnderSync = hw->wSyncDepth != 0U;
    hw->wEnableCalls++;
    if (!enable && hw->bFailDisable) return FOC_RESULT_INVALID_ARGUMENT;
    hw->bEnabled = enable;
    return FOC_RESULT_OK;
}

static void fake_stop(void *p)
{
    fake_fsm_hw_t *hw = p;
    hw->wStopCalls++;
    hw->bEnabled = false;
}

static foc_result_t fake_calibrate(void *p, foc_adc_calib_t *calib)
{
    fake_fsm_hw_t *hw = p;
    hw->wCalibrateCalls++;
    hw->bCalibrationWhileEnabled = hw->bEnabled;
    if (hw->bFailCalibration) return FOC_RESULT_INVALID_ARGUMENT;
    if (hw->bEmergencyDuringCalibration)
        motor_EmergencyStop(hw->ptMotor, MOTOR_FAULT_HARDWARE);
    if (hw->bStopDuringCalibration)
        (void)motor_Stop(hw->ptMotor);
    calib->bIsCalibrated = true;
    return FOC_RESULT_OK;
}

static foc_result_t fake_reconstruct(void *p, phase_current_handle_t *current)
{
    (void)p; (void)current;
    return FOC_RESULT_OK;
}

static uint32_t fake_now(void *p)
{
    return ((fake_fsm_hw_t *)p)->wNow;
}

static uintptr_t fake_enter(void *p)
{
    ((fake_fsm_hw_t *)p)->wSyncDepth++;
    return 0U;
}

static void fake_exit(void *p, uintptr_t state)
{
    (void)state;
    ((fake_fsm_hw_t *)p)->wSyncDepth--;
}

static foc_result_t fake_position_step(void *context,
    const foc_position_input_t *input, foc_position_output_t *output)
{
    (void)context; (void)input; (void)output;
    return FOC_RESULT_OK;
}

static motor_config_t config_for(fake_fsm_hw_t *hw, uint32_t delay)
{
    motor_config_t cfg = {0};
    cfg.tParams.chPolePairs = 4U;
    cfg.qHighFrequencyPeriod = FOC_SCALAR(0.001f);
    cfg.qLowFrequencyPeriod = FOC_SCALAR(0.01f);
    cfg.tPosition.chPolePairs = 4U;
    cfg.tPosition.chDirection = 1;
    cfg.eTopology = SENSING_TOPOLOGY_3P;
    cfg.tHal.tPwm = (foc_pwm_if_t){hw, fake_set_duty, fake_enable, fake_stop};
    cfg.tHal.tAdc.pContext = hw;
    cfg.tHal.tAdc.fnOffsetCalib = fake_calibrate;
    cfg.tHal.tAdc.fnReconstruct = fake_reconstruct;
    cfg.tTime = (motor_time_if_t){hw, fake_now};
    cfg.tSync = (motor_sync_if_t){hw, fake_enter, fake_exit};
    cfg.wStartupDelayMs = delay;
    return cfg;
}

static motor_run_config_t open_run(void)
{
    motor_run_config_t run = {0};
    run.eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP;
    run.qInitialAngle = FOC_SCALAR(0.1f);
    run.qOpenLoopSpeed = FOC_SCALAR(0.2f);
    run.qAcceleration = FOC_SCALAR(0.01f);
    run.tVoltageReference.qQ = FOC_SCALAR(0.3f);
    return run;
}

int test_motor_fsm(void)
{
    int nFailures = 0;
    fake_fsm_hw_t hw = {0};
    motor_config_t cfg = config_for(&hw, 10U);
    motor_run_config_t run = open_run();
    motor_handle_t motor;
    motor_handle_t invalid = {0};
    motor_snapshot_t snap;
    motor_event_t event;
    unsigned wInitialEnableCalls;

    TEST_CHECK(motor_Start(NULL, &run) == FOC_RESULT_NULL);
    TEST_CHECK(motor_Stop(NULL) == FOC_RESULT_NULL);
    TEST_CHECK(motor_ClearFault(NULL) == FOC_RESULT_NULL);
    TEST_CHECK(motor_RunFSM(NULL) == fsm_rt_err);
    TEST_CHECK(motor_Start(&invalid, &run) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_Stop(&invalid) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_ClearFault(&invalid) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_RunFSM(&invalid) == fsm_rt_err);

    TEST_CHECK(motor_Init(&motor, &cfg) == FOC_RESULT_OK);
    hw.ptMotor = &motor;
    TEST_CHECK(hw.wSyncDepth == 0U);
    wInitialEnableCalls = hw.wEnableCalls;
    run.eControlMode = (motor_control_mode_e)-1;
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_DebugReadEvent(&motor, &event));
    TEST_CHECK(event.eType == MOTOR_EVENT_COMMAND_REJECTED);
    TEST_CHECK(event.eCommand == MOTOR_COMMAND_START);
    TEST_CHECK(event.eResult == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(event.eFromState == MOTOR_STATE_IDLE);
    TEST_CHECK(event.eToState == MOTOR_STATE_IDLE);
    TEST_CHECK(event.wSequence == 1U);
    run = open_run();
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    motor_Reset(&motor);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.ePendingCommand == MOTOR_COMMAND_START);
    TEST_CHECK(!hw.bEnabled && hw.wCalibrateCalls == 0U);
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_BUSY);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_DebugReadEvent(&motor, &event));
    TEST_CHECK(event.eType == MOTOR_EVENT_COMMAND_ACCEPTED);
    TEST_CHECK(event.eCommand == MOTOR_COMMAND_START);
    TEST_CHECK(event.eResult == FOC_RESULT_OK);
    TEST_CHECK(event.wSequence == 2U);
    TEST_CHECK(motor_DebugReadEvent(&motor, &event));
    TEST_CHECK(event.eType == MOTOR_EVENT_COMMAND_REJECTED);
    TEST_CHECK(event.eCommand == MOTOR_COMMAND_START);
    TEST_CHECK(event.eResult == FOC_RESULT_BUSY);
    TEST_CHECK(event.wSequence == 3U);
    TEST_CHECK(motor_DebugReadEvent(&motor, &event));
    TEST_CHECK(event.eType == MOTOR_EVENT_STATE_CHANGED);
    TEST_CHECK(event.eFromState == MOTOR_STATE_IDLE);
    TEST_CHECK(event.eToState == MOTOR_STATE_STARTING);
    TEST_CHECK(event.wSequence == 4U);
    TEST_CHECK(hw.wCalibrateCalls == 1U && !hw.bEnabled);
    TEST_CHECK(!hw.bCalibrationWhileEnabled);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_STARTING);
    TEST_CHECK(snap.eStartupPhase == MOTOR_STARTUP_WAIT_DELAY);
    hw.wNow = 9U;
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(hw.wCalibrateCalls == 1U &&
               hw.wEnableCalls == wInitialEnableCalls);
    hw.wNow = 10U;
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(!hw.bEnabled);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(hw.bEnabled &&
               hw.wEnableCalls == wInitialEnableCalls + 1U);
    motor_Reset(&motor);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_RUNNING && snap.bPwmEnabled);
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_Stop(&motor) == FOC_RESULT_OK);
    TEST_CHECK(hw.bEnabled);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(!hw.bEnabled);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_IDLE);

    hw.bFailCalibration = true;
    hw.wStopCalls = 0U;
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
    TEST_CHECK(hw.wStopCalls == 1U);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_FAULT && !snap.bPwmEnabled);
    motor_Reset(&motor);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);

    hw.bFailCalibration = false;
    hw.bStopDuringCalibration = true;
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_IDLE && !snap.bPwmEnabled);
    hw.bStopDuringCalibration = false;
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_IDLE &&
               snap.wFaults == MOTOR_FAULT_NONE);
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_INVALID_ARGUMENT);
    hw.bEmergencyDuringCalibration = true;
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_FAULT && !snap.bPwmEnabled);
    hw.bEmergencyDuringCalibration = false;
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);
    motor_test_CorruptFSM(&motor, MOTOR_STATE_STARTING,
                          MOTOR_STARTUP_COMPLETE);
    TEST_CHECK(!motor_TestCommitTransitionTimeout(&motor));
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_STARTING);
    motor_test_CorruptFSM(&motor, MOTOR_STATE_STARTING,
                          MOTOR_STARTUP_QUALIFY_SOURCE);
    TEST_CHECK(motor_TestCommitTransitionTimeout(&motor));
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK((snap.wFaults & MOTOR_FAULT_TRANSITION_TIMEOUT) != 0U);
    {
        motor_event_t tLastEvent = {0};
        uint32_t wLastSequence = 0U;
        bool bGotEvent = false;

        while (motor_DebugReadEvent(&motor, &event)) {
            if (bGotEvent) {
                TEST_CHECK(event.wSequence > wLastSequence);
            }
            wLastSequence = event.wSequence;
            tLastEvent = event;
            bGotEvent = true;
        }
        TEST_CHECK(bGotEvent);
        TEST_CHECK(tLastEvent.eType == MOTOR_EVENT_TRANSITION_TIMEOUT);
        TEST_CHECK(tLastEvent.eFromState == MOTOR_STATE_STARTING);
        TEST_CHECK(tLastEvent.eToState == MOTOR_STATE_FAULT);
        TEST_CHECK(tLastEvent.wPreviousValue ==
                   MOTOR_STARTUP_QUALIFY_SOURCE);
        TEST_CHECK(tLastEvent.wCurrentValue == MOTOR_STARTUP_IDLE);
    }
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);

    {
        foc_position_source_if_t invalid_source = {0};
        foc_position_source_if_t a = {NULL, NULL, fake_position_step};
        foc_position_source_if_t b = {NULL, NULL, fake_position_step};
        run.ptInitialPositionSource = &invalid_source;
        TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);
        run.ptInitialPositionSource = &a;
        run.ptTargetPositionSource = &b;
        TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);
        run.ptTargetPositionSource = &a;
        TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
        a.fnStep = NULL;
        TEST_CHECK(motor_test_PositionBindingsValid(&motor));
        TEST_CHECK(motor_Stop(&motor) == FOC_RESULT_OK);
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    }

    run = open_run();
    run.eControlMode = MOTOR_CONTROL_SPEED;
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_INVALID_ARGUMENT);

    motor_test_CorruptFSM(&motor, (motor_state_e)99,
                          MOTOR_STARTUP_IDLE);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);
    motor_test_CorruptFSM(&motor, MOTOR_STATE_STARTING,
                          (motor_startup_phase_e)99);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);

    run = open_run();
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    hw.bFailDisable = true;
    TEST_CHECK(motor_Stop(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
    TEST_CHECK(motor_GetSnapshot(&motor, &snap) == FOC_RESULT_OK);
    TEST_CHECK(snap.eRunState == MOTOR_STATE_FAULT);
    hw.bFailDisable = false;
    TEST_CHECK(motor_ClearFault(&motor) == FOC_RESULT_OK);

    {
        foc_position_source_if_t source = {NULL, NULL, fake_position_step};
        run = open_run();
        run.ptInitialPositionSource = &source;
        TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
        hw.wNow += 10U;
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
        TEST_CHECK(motor_Stop(&motor) == FOC_RESULT_OK);
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);

        run = open_run();
        run.eControlMode = MOTOR_CONTROL_SPEED;
        run.ptTargetPositionSource = &source;
        TEST_CHECK(motor_Start(&motor, &run) ==
                   FOC_RESULT_INVALID_ARGUMENT);
    }

    cfg = config_for(&hw, 10U);
    hw.wNow = UINT32_MAX - 5U;
    TEST_CHECK(motor_Init(&motor, &cfg) == FOC_RESULT_OK);
    run = open_run();
    TEST_CHECK(motor_Start(&motor, &run) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    hw.wNow = 4U;
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(motor_Stop(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);

    cfg.tTime.fnGetMilliseconds = NULL;
    TEST_CHECK(motor_Init(&motor, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
    cfg.wStartupDelayMs = 0U;
    TEST_CHECK(motor_Init(&motor, &cfg) == FOC_RESULT_OK);
    cfg.tSync.fnExit = NULL;
    TEST_CHECK(motor_Init(&motor, &cfg) == FOC_RESULT_INVALID_ARGUMENT);
    return nFailures;
}
