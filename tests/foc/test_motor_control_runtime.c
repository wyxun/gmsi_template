#include "test_common.h"

#include "motor.h"

typedef struct {
    unsigned enable_calls;
    unsigned stop_calls;
    unsigned duty_calls;
    unsigned sample_calls;
    bool enabled;
} runtime_hw_t;

typedef struct {
    unsigned calls;
    foc_scalar_t output;
} runtime_controller_t;

typedef struct {
    unsigned calls;
    foc_result_t result;
    foc_position_valid_flag_e flags;
    motor_handle_t *motor;
    bool reenter_hf;
    bool stop_motor;
    foc_result_t callback_result;
} runtime_source_t;

static foc_result_t set_duty(void *context, foc_scalar_t u,
                             foc_scalar_t v, foc_scalar_t w)
{
    runtime_hw_t *hw = context;
    (void)u; (void)v; (void)w;
    hw->duty_calls++;
    return FOC_RESULT_OK;
}

static foc_result_t enable_pwm(void *context, bool enable)
{
    runtime_hw_t *hw = context;
    hw->enable_calls++;
    hw->enabled = enable;
    return FOC_RESULT_OK;
}

static void emergency(void *context)
{
    runtime_hw_t *hw = context;
    hw->stop_calls++;
    hw->enabled = false;
}

static foc_result_t calibrate(void *context, foc_adc_calib_t *calib)
{
    (void)context;
    calib->bIsCalibrated = true;
    return FOC_RESULT_OK;
}

static foc_result_t sample(void *context,
                           phase_current_handle_t *current)
{
    runtime_hw_t *hw = context;
    current->qIu = FOC_SCALAR(0.1f);
    current->qIv = FOC_SCALAR(-0.1f);
    current->qIw = FOC_ZERO;
    hw->sample_calls++;
    return FOC_RESULT_OK;
}

static foc_scalar_t controller_step(void *context, foc_scalar_t reference,
                                    foc_scalar_t feedback)
{
    runtime_controller_t *controller = context;
    (void)reference; (void)feedback;
    controller->calls++;
    return controller->output;
}

static foc_result_t source_step(void *context,
    const foc_position_input_t *input, foc_position_output_t *output)
{
    runtime_source_t *source = context;
    (void)input;
    source->calls++;
    if (source->reenter_hf) {
        source->reenter_hf = false;
        source->callback_result = motor_HighFrequencyStep(source->motor);
    }
    if (source->stop_motor) {
        source->stop_motor = false;
        source->callback_result = motor_Stop(source->motor);
    }
    if (source->result != FOC_RESULT_OK) return source->result;
    output->tElectricalAngle = foc_angle_from_turns(0.2f);
    output->tMechanicalAngle = foc_angle_from_turns(0.05f);
    output->qElectricalSpeed = FOC_SCALAR(0.4f);
    output->qMechanicalSpeed = FOC_SCALAR(0.1f);
    output->eValidFlags = source->flags;
    return FOC_RESULT_OK;
}

static motor_config_t runtime_config(runtime_hw_t *hw,
    runtime_controller_t controllers[4])
{
    motor_config_t config = {0};
    foc_controller_if_t *bindings[4] = {
        &config.tControl.tIdController,
        &config.tControl.tIqController,
        &config.tControl.tSpeedController,
        &config.tControl.tPositionController,
    };
    config.tParams.chPolePairs = 4U;
    config.tPosition.chPolePairs = 4U;
    config.tPosition.chDirection = 1;
    config.qHighFrequencyPeriod = FOC_SCALAR(0.01f);
    config.qLowFrequencyPeriod = FOC_SCALAR(0.1f);
    config.eTopology = SENSING_TOPOLOGY_3P;
    config.tHal.tPwm = (foc_pwm_if_t){hw, set_duty, enable_pwm, emergency};
    config.tHal.tAdc.pContext = hw;
    config.tHal.tAdc.fnOffsetCalib = calibrate;
    config.tHal.tAdc.fnReconstruct = sample;
    for (unsigned i = 0; i < 4U; i++) {
        bindings[i]->pContext = &controllers[i];
        bindings[i]->fnStep = controller_step;
    }
    return config;
}

static bool start_running(motor_handle_t *motor,
                          const motor_run_config_t *run)
{
    return motor_Start(motor, run) == FOC_RESULT_OK &&
           motor_RunFSM(motor) == fsm_rt_on_going &&
           motor_RunFSM(motor) == fsm_rt_on_going &&
           motor_RunFSM(motor) == fsm_rt_cpl;
}

static int test_target_mode(motor_control_mode_e mode)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = {
        .result = FOC_RESULT_OK,
        .flags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                 FOC_POSITION_VALID_ELECTRICAL_SPEED |
                 FOC_POSITION_VALID_MECHANICAL_ANGLE |
                 FOC_POSITION_VALID_MECHANICAL_SPEED,
    };
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = mode,
        .ptTargetPositionSource = &source_if,
        .qAcceleration = FOC_SCALAR(0.2f),
        .qSpeedReference = FOC_SCALAR(0.1f),
        .qPositionReference = FOC_SCALAR(0.1f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_running(&motor, &run));
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 0U && controllers[3].calls == 0U);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[0].calls == 1U && controllers[1].calls == 1U);
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 0U && controllers[3].calls == 0U);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_RUNNING &&
               snapshot.wFaults == MOTOR_FAULT_NONE && hw.enabled);
    return nFailures;
}

static int test_ramp_case(float initial, float target, float acceleration)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
        .qOpenLoopSpeed = foc_from_float(target),
        .qAcceleration = foc_from_float(acceleration),
        .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.1f)},
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;
    float previous = initial;
    float limit = acceleration * 0.01f + 0.0002f;

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_running(&motor, &run));
    motor_TestSetOpenLoopCommandSpeed(&motor, foc_from_float(initial));
    for (unsigned i = 0; i < 100U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
        TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
        float current = foc_to_float(snapshot.qElectricalSpeed);
        TEST_CHECK((current - previous <= limit) &&
                   (previous - current <= limit));
        if (target > previous) TEST_CHECK(current >= previous);
        if (target < previous) TEST_CHECK(current <= previous);
        if (i == 0U && initial != target)
            TEST_CHECK(current != target);
        previous = current;
    }
    TEST_NEAR(previous, target, 0.0003f);
    return nFailures;
}

static int test_source_failure(foc_result_t result,
                               foc_position_valid_flag_e flags)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = {.result = result, .flags = flags};
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptInitialPositionSource = &source_if,
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_running(&motor, &run));
    TEST_CHECK(motor_HighFrequencyStep(&motor) != FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(source.calls == 1U && hw.stop_calls == 1U && !hw.enabled);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_FAULT &&
               snapshot.wFaults != MOTOR_FAULT_NONE &&
               !snapshot.bPwmEnabled);
    return nFailures;
}

static int test_callback_interlock(bool stop_motor)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = {
        .result = FOC_RESULT_OK,
        .flags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                 FOC_POSITION_VALID_MECHANICAL_ANGLE |
                 FOC_POSITION_VALID_MECHANICAL_SPEED,
        .reenter_hf = !stop_motor,
        .stop_motor = stop_motor,
    };
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_CURRENT,
        .ptInitialPositionSource = &source_if,
    };
    motor_handle_t motor;
    motor_snapshot_t before;
    motor_snapshot_t after;

    source.motor = &motor;
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_running(&motor, &run));
    TEST_CHECK(motor_GetSnapshot(&motor, &before) == FOC_RESULT_OK);
    foc_result_t result = motor_HighFrequencyStep(&motor);
    TEST_CHECK(source.callback_result ==
               (stop_motor ? FOC_RESULT_OK : FOC_RESULT_BUSY));
    TEST_CHECK(result == (stop_motor ? FOC_RESULT_BUSY : FOC_RESULT_OK));
    TEST_CHECK(hw.duty_calls == (stop_motor ? 0U : 1U));
    TEST_CHECK(motor_GetSnapshot(&motor, &after) == FOC_RESULT_OK);
    if (stop_motor) {
        TEST_CHECK(after.ePendingCommand == MOTOR_COMMAND_STOP);
        TEST_CHECK(after.tElectricalAngle.qTurns ==
                   before.tElectricalAngle.qTurns);
        TEST_CHECK(after.wFaults == MOTOR_FAULT_NONE);
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl && !hw.enabled);
    }
    return nFailures;
}

static int test_direct_outer_loop(motor_control_mode_e mode)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = {
        .result = FOC_RESULT_OK,
        .flags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                 FOC_POSITION_VALID_ELECTRICAL_SPEED |
                 FOC_POSITION_VALID_MECHANICAL_ANGLE |
                 FOC_POSITION_VALID_MECHANICAL_SPEED,
    };
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = mode,
        .ptInitialPositionSource = &source_if,
        .qSpeedReference = FOC_SCALAR(0.3f),
        .qPositionReference = FOC_SCALAR(0.2f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    controllers[0].output = FOC_SCALAR(0.15f);
    controllers[1].output = FOC_SCALAR(0.25f);
    controllers[2].output = FOC_SCALAR(0.25f);
    controllers[3].output = FOC_SCALAR(0.35f);
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_running(&motor, &run));
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 1U);
    TEST_CHECK(controllers[3].calls ==
               (mode == MOTOR_CONTROL_POSITION ? 1U : 0U));
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(snapshot.tVoltage.qQ), 0.25f, 0.0002f);
    TEST_CHECK(snapshot.wFaults == MOTOR_FAULT_NONE);
    return nFailures;
}

int test_motor_control_runtime(void)
{
    int nFailures = 0;

    TEST_CHECK(motor_TestGetImplementationSize() <= 480U);
    nFailures += test_target_mode(MOTOR_CONTROL_SPEED);
    nFailures += test_target_mode(MOTOR_CONTROL_POSITION);
    nFailures += test_ramp_case(0.0f, 0.05f, 0.1f);
    nFailures += test_ramp_case(0.0f, -0.05f, 0.1f);
    nFailures += test_ramp_case(0.03f, -0.03f, 0.1f);
    nFailures += test_ramp_case(0.05f, 0.01f, 0.1f);
    nFailures += test_source_failure(FOC_RESULT_INVALID_ARGUMENT,
                                     FOC_POSITION_VALID_NONE);
    nFailures += test_source_failure(FOC_RESULT_OK,
                                     FOC_POSITION_VALID_MECHANICAL_SPEED);
    nFailures += test_callback_interlock(false);
    nFailures += test_callback_interlock(true);
    nFailures += test_direct_outer_loop(MOTOR_CONTROL_SPEED);
    nFailures += test_direct_outer_loop(MOTOR_CONTROL_POSITION);
    return nFailures;
}
