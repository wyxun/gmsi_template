#include "test_common.h"

#include "motor.h"

typedef struct {
    unsigned enable_calls;
    unsigned stop_calls;
    unsigned duty_calls;
    unsigned sample_calls;
    bool enabled;
    uint32_t now_ms;
    foc_duty_abc_t duty;
} runtime_hw_t;

typedef struct {
    unsigned calls;
    unsigned track_calls;
    foc_scalar_t output;
    foc_scalar_t reference;
    foc_scalar_t feedback;
    foc_scalar_t tracked_output;
} runtime_controller_t;

typedef struct {
    unsigned calls;
    foc_result_t result;
    foc_position_valid_flag_e flags;
    motor_handle_t *motor;
    bool reenter_hf;
    bool stop_motor;
    bool copy_timestamp;
    bool require_control_input;
    uint32_t timestamp;
    uint32_t faults;
    foc_angle_t electrical_angle;
    foc_angle_t mechanical_angle;
    foc_scalar_t electrical_speed;
    foc_scalar_t mechanical_speed;
    foc_scalar_t confidence;
    foc_result_t callback_result;
} runtime_source_t;

static foc_result_t set_duty(void *context, foc_scalar_t u,
                             foc_scalar_t v, foc_scalar_t w)
{
    runtime_hw_t *hw = context;
    hw->duty_calls++;
    hw->duty = (foc_duty_abc_t){u, v, w};
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

static uint32_t now_ms(void *context)
{
    return ((runtime_hw_t *)context)->now_ms;
}

static foc_scalar_t controller_step(void *context, foc_scalar_t reference,
                                    foc_scalar_t feedback)
{
    runtime_controller_t *controller = context;
    controller->calls++;
    controller->reference = reference;
    controller->feedback = feedback;
    return controller->output;
}

static void controller_track(void *context, foc_scalar_t output,
                             foc_scalar_t reference,
                             foc_scalar_t feedback)
{
    runtime_controller_t *controller = context;

    controller->track_calls++;
    controller->tracked_output = output;
    controller->reference = reference;
    controller->feedback = feedback;
}

static foc_result_t source_step(void *context,
    const foc_position_input_t *input, foc_position_output_t *output)
{
    runtime_source_t *source = context;
    source->calls++;
    if (source->require_control_input &&
        (input->tCurrent.qAlpha == FOC_ZERO ||
         input->tVoltage.qAlpha == FOC_ZERO)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (source->reenter_hf) {
        source->reenter_hf = false;
        source->callback_result = motor_HighFrequencyStep(source->motor);
    }
    if (source->stop_motor) {
        source->stop_motor = false;
        source->callback_result = motor_Stop(source->motor);
    }
    if (source->result != FOC_RESULT_OK) return source->result;
    output->tElectricalAngle = source->electrical_angle;
    output->tMechanicalAngle = source->mechanical_angle;
    output->qElectricalSpeed = source->electrical_speed;
    output->qMechanicalSpeed = source->mechanical_speed;
    output->qConfidence = source->confidence;
    output->eValidFlags = source->flags;
    output->wFaults = source->faults;
    output->wTimestamp = source->copy_timestamp ?
                         input->wTimestamp : source->timestamp;
    return FOC_RESULT_OK;
}

static runtime_source_t valid_source(void)
{
    runtime_source_t source = {
        .result = FOC_RESULT_OK,
        .flags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                 FOC_POSITION_VALID_ELECTRICAL_SPEED |
                 FOC_POSITION_VALID_MECHANICAL_ANGLE |
                 FOC_POSITION_VALID_MECHANICAL_SPEED,
        .copy_timestamp = true,
        .electrical_angle = {FOC_SCALAR(0.2f)},
        .mechanical_angle = {FOC_SCALAR(0.05f)},
        .electrical_speed = FOC_SCALAR(0.4f),
        .mechanical_speed = FOC_SCALAR(0.1f),
        .confidence = FOC_ONE,
    };
    return source;
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
    config.qTransitionMinimumConfidence = FOC_SCALAR(0.8f);
    config.qTransitionMinimumSpeed = FOC_SCALAR(0.05f);
    config.qTransitionMaximumAngleError = FOC_SCALAR(0.25f);
    config.hwTransitionQualificationSamples = 3U;
    config.hwTransitionBlendSamples = 4U;
    config.wTransitionTimeoutMs = 50U;
    config.eTopology = SENSING_TOPOLOGY_3P;
    config.tHal.tPwm = (foc_pwm_if_t){hw, set_duty, enable_pwm, emergency};
    config.tHal.tAdc.pContext = hw;
    config.tHal.tAdc.fnOffsetCalib = calibrate;
    config.tHal.tAdc.fnReconstruct = sample;
    config.tTime = (motor_time_if_t){hw, now_ms};
    for (unsigned i = 0; i < 4U; i++) {
        bindings[i]->pContext = &controllers[i];
        bindings[i]->fnStep = controller_step;
        bindings[i]->fnTrack = controller_track;
    }
    return config;
}

static bool start_enabled(motor_handle_t *motor,
                          const motor_run_config_t *run,
                          bool transition_expected)
{
    if (motor_Start(motor, run) != FOC_RESULT_OK ||
        motor_RunFSM(motor) != fsm_rt_on_going ||
        motor_RunFSM(motor) != fsm_rt_on_going) {
        return false;
    }
    return motor_RunFSM(motor) ==
           (transition_expected ? fsm_rt_on_going : fsm_rt_cpl);
}

static float duty_max_delta(foc_duty_abc_t from, foc_duty_abc_t to)
{
    float du = foc_to_float(to.qU) - foc_to_float(from.qU);
    float dv = foc_to_float(to.qV) - foc_to_float(from.qV);
    float dw = foc_to_float(to.qW) - foc_to_float(from.qW);
    float maximum;

    if (du < 0.0f) du = -du;
    if (dv < 0.0f) dv = -dv;
    if (dw < 0.0f) dw = -dw;
    maximum = du > dv ? du : dv;
    return maximum > dw ? maximum : dw;
}

static int test_target_mode(motor_control_mode_e mode)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = mode,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.4f),
        .qAcceleration = FOC_SCALAR(0.2f),
        .qSpeedReference = FOC_SCALAR(0.1f),
        .qPositionReference = FOC_SCALAR(0.1f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 0U && controllers[3].calls == 0U);
    for (unsigned i = 0; i < 7U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
        TEST_CHECK(motor_RunFSM(&motor) ==
                   (i == 6U ? fsm_rt_cpl : fsm_rt_on_going));
    }
    TEST_CHECK(controllers[0].calls == 7U && controllers[1].calls == 7U);
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 1U);
    TEST_CHECK(controllers[3].calls ==
               (mode == MOTOR_CONTROL_POSITION ? 1U : 0U));
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
    TEST_CHECK(start_enabled(&motor, &run, false));
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
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptInitialPositionSource = &source_if,
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    source.result = result;
    source.flags = flags;
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, false));
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
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_CURRENT,
        .ptInitialPositionSource = &source_if,
    };
    motor_handle_t motor;
    motor_snapshot_t before;
    motor_snapshot_t after;

    source.flags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                   FOC_POSITION_VALID_MECHANICAL_ANGLE |
                   FOC_POSITION_VALID_MECHANICAL_SPEED;
    source.reenter_hf = !stop_motor;
    source.stop_motor = stop_motor;
    source.motor = &motor;
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, false));
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
    runtime_source_t source = valid_source();
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
    TEST_CHECK(start_enabled(&motor, &run, false));
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

static int test_transition_qualification_and_wrap(void)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptTargetPositionSource = &source_if,
        .qInitialAngle = FOC_SCALAR(0.99f),
        .qOpenLoopSpeed = FOC_SCALAR(0.1f),
        .qAcceleration = FOC_SCALAR(1.0f),
        .tCurrentReference = {FOC_SCALAR(0.05f), FOC_SCALAR(0.10f)},
        .qSpeedReference = FOC_SCALAR(0.1f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    config.tPosition.chPolePairs = 1U;
    config.hwTransitionQualificationSamples = 2U;
    config.hwTransitionBlendSamples = 2U;
    config.qTransitionMaximumAngleError = FOC_SCALAR(0.05f);
    source.mechanical_angle = foc_angle_from_turns(0.01f);
    source.mechanical_speed = FOC_SCALAR(0.1f);
    controllers[0].output = FOC_SCALAR(0.05f);
    controllers[1].output = FOC_SCALAR(0.10f);
    controllers[2].output = FOC_SCALAR(0.10f);

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));

    source.confidence = FOC_SCALAR(0.2f);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.confidence = FOC_ONE;
    source.copy_timestamp = false;
    source.timestamp = 1U;
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.copy_timestamp = true;
    source.mechanical_speed = FOC_SCALAR(-0.1f);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.mechanical_speed = FOC_SCALAR(0.1f);
    source.mechanical_angle = foc_angle_from_turns(0.3f);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.mechanical_angle = foc_angle_from_turns(0.01f);

    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_STARTING);
    TEST_CHECK(controllers[2].calls == 0U);

    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(foc_angle_to_turns(snapshot.tElectricalAngle) < 0.02f ||
               foc_angle_to_turns(snapshot.tElectricalAngle) > 0.98f);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_on_going);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(motor_LowFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(controllers[2].calls == 1U);
    TEST_NEAR(foc_to_float(controllers[0].reference), 0.05f, 0.002f);
    TEST_NEAR(foc_to_float(controllers[1].reference), 0.10f, 0.002f);
    return nFailures;
}

static int test_transition_requires_consecutive_samples(void)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.1f),
        .qAcceleration = FOC_SCALAR(1.0f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    config.tPosition.chPolePairs = 1U;
    config.hwTransitionQualificationSamples = 2U;
    source.mechanical_angle = foc_angle_from_turns(0.01f);
    source.mechanical_speed = FOC_SCALAR(0.1f);
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));

    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.mechanical_speed = FOC_SCALAR(0.01f);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.mechanical_speed = FOC_SCALAR(0.1f);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.flags = (foc_position_valid_flag_e)(
        source.flags & ~FOC_POSITION_VALID_MECHANICAL_SPEED);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.flags = (foc_position_valid_flag_e)(
        source.flags | FOC_POSITION_VALID_MECHANICAL_SPEED);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eStartupPhase == MOTOR_STARTUP_QUALIFY_SOURCE);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eStartupPhase == MOTOR_STARTUP_BLEND_ANGLE);
    return nFailures;
}

static int test_transition_blend_uses_frozen_angle_bound(void)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.4f),
        .qAcceleration = FOC_SCALAR(10.0f),
        .tCurrentReference = {FOC_SCALAR(0.05f), FOC_SCALAR(0.10f)},
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;
    foc_duty_abc_t previous_duty = {0};
    float maximum_duty_delta = 0.0f;

    config.tPosition.chPolePairs = 1U;
    config.hwTransitionQualificationSamples = 1U;
    config.hwTransitionBlendSamples = 30U;
    config.qTransitionMaximumAngleError = FOC_SCALAR(0.05f);
    source.mechanical_angle = foc_angle_from_turns(0.04f);
    source.mechanical_speed = FOC_SCALAR(0.4f);
    controllers[0].output = FOC_SCALAR(0.05f);
    controllers[1].output = FOC_SCALAR(0.10f);

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    previous_duty = hw.duty;
    for (unsigned i = 0; i < 30U; i++) {
        float delta;

        if (i == 20U) {
            source.mechanical_angle = foc_angle_from_turns(0.96f);
        }
        TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
        delta = duty_max_delta(previous_duty, hw.duty);
        if (delta > maximum_duty_delta) maximum_duty_delta = delta;
        previous_duty = hw.duty;
    }
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_RUNNING);
    TEST_CHECK(snapshot.wFaults == MOTOR_FAULT_NONE);
    TEST_CHECK(maximum_duty_delta < 0.10f);
    TEST_CHECK(controllers[2].track_calls == 1U);
    TEST_NEAR(foc_to_float(controllers[2].tracked_output), 0.10f, 0.002f);
    TEST_NEAR(foc_to_float(controllers[0].reference), 0.05f, 0.002f);
    TEST_NEAR(foc_to_float(controllers[1].reference), 0.10f, 0.002f);
    return nFailures;
}

static int test_transition_source_receives_control_input(void)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_CURRENT,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.1f),
        .qAcceleration = FOC_SCALAR(1.0f),
        .tCurrentReference = {FOC_SCALAR(0.1f), FOC_SCALAR(0.2f)},
    };
    motor_handle_t motor;

    controllers[0].output = FOC_SCALAR(0.2f);
    controllers[1].output = FOC_SCALAR(0.3f);
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    source.require_control_input = true;
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    return nFailures;
}

static int test_transition_failure(bool timeout)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.4f),
        .qAcceleration = FOC_SCALAR(1.0f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;

    config.hwTransitionQualificationSamples = 1U;
    config.hwTransitionBlendSamples = 3U;
    config.wTransitionTimeoutMs = 5U;
    if (timeout) {
        source.confidence = FOC_ZERO;
    }
    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));
    if (timeout) {
        motor_event_t tEvent;
        motor_event_t tLastEvent = {0};
        bool bGotEvent = false;

        TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
        hw.now_ms = 6U;
        TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_err);
        while (motor_DebugReadEvent(&motor, &tEvent)) {
            tLastEvent = tEvent;
            bGotEvent = true;
        }
        TEST_CHECK(bGotEvent);
        TEST_CHECK(tLastEvent.eType == MOTOR_EVENT_TRANSITION_TIMEOUT);
        TEST_CHECK(tLastEvent.eFromState == MOTOR_STATE_STARTING);
        TEST_CHECK(tLastEvent.eToState == MOTOR_STATE_FAULT);
        TEST_CHECK(tLastEvent.wPreviousValue ==
                   MOTOR_STARTUP_QUALIFY_SOURCE);
    } else {
        TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
        source.faults = FOC_POSITION_FAULT_INVALID_DATA;
        TEST_CHECK(motor_HighFrequencyStep(&motor) != FOC_RESULT_OK);
    }
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK((snapshot.wFaults &
               (timeout ? MOTOR_FAULT_TRANSITION_TIMEOUT :
                          MOTOR_FAULT_POSITION_SOURCE)) != 0U);
    TEST_CHECK(!hw.enabled && hw.stop_calls == 1U);
    return nFailures;
}

static int test_transition_events(void)
{
    int nFailures = 0;
    runtime_hw_t hw = {0};
    runtime_controller_t controllers[4] = {{0}};
    runtime_source_t source = valid_source();
    foc_position_source_if_t source_if = {&source, NULL, source_step};
    motor_config_t config = runtime_config(&hw, controllers);
    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_SPEED,
        .ptTargetPositionSource = &source_if,
        .qOpenLoopSpeed = FOC_SCALAR(0.4f),
        .qAcceleration = FOC_SCALAR(1.0f),
        .tCurrentReference = {FOC_SCALAR(0.05f), FOC_SCALAR(0.10f)},
        .qSpeedReference = FOC_SCALAR(0.1f),
    };
    motor_handle_t motor;
    motor_snapshot_t snapshot;
    motor_event_t atEvents[4];
    unsigned uCount = 0U;

    config.tPosition.chPolePairs = 1U;
    config.hwTransitionQualificationSamples = 1U;
    config.hwTransitionBlendSamples = 2U;
    controllers[0].output = FOC_SCALAR(0.05f);
    controllers[1].output = FOC_SCALAR(0.10f);
    controllers[2].output = FOC_SCALAR(0.10f);

    TEST_CHECK(motor_Init(&motor, &config) == FOC_RESULT_OK);
    TEST_CHECK(start_enabled(&motor, &run, true));
    /* One qualification sample then two blend samples complete the
       open-to-closed transfer entirely in the high-frequency path. */
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_HighFrequencyStep(&motor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&motor) == fsm_rt_cpl);
    TEST_CHECK(motor_GetSnapshot(&motor, &snapshot) == FOC_RESULT_OK);
    TEST_CHECK(snapshot.eRunState == MOTOR_STATE_RUNNING);

    while (uCount < 4U &&
           motor_DebugReadEvent(&motor, &atEvents[uCount])) {
        uCount++;
    }
    TEST_CHECK(uCount == 4U);
    TEST_CHECK(!motor_DebugReadEvent(&motor, &atEvents[0]));
    if (uCount == 4U) {
        TEST_CHECK(atEvents[0].eType ==
                   MOTOR_EVENT_SOURCE_VALIDITY_CHANGED);
        TEST_CHECK(atEvents[0].ePositionRole == MOTOR_POSITION_ROLE_ACTIVE);
        TEST_CHECK(atEvents[0].wPreviousValue == 0U);
        TEST_CHECK((atEvents[0].wCurrentValue &
                    FOC_POSITION_VALID_ELECTRICAL_ANGLE) != 0U);
        TEST_CHECK(atEvents[1].eType ==
                   MOTOR_EVENT_SOURCE_VALIDITY_CHANGED);
        TEST_CHECK(atEvents[1].ePositionRole ==
                   MOTOR_POSITION_ROLE_CANDIDATE);
        TEST_CHECK(atEvents[1].wPreviousValue == 0U);
        TEST_CHECK((atEvents[1].wCurrentValue &
                    FOC_POSITION_VALID_ELECTRICAL_ANGLE) != 0U);
        TEST_CHECK(atEvents[2].eType == MOTOR_EVENT_TRANSITION_STARTED);
        TEST_CHECK(atEvents[2].wPreviousValue ==
                   MOTOR_STARTUP_QUALIFY_SOURCE);
        TEST_CHECK(atEvents[2].wCurrentValue == MOTOR_STARTUP_BLEND_ANGLE);
        TEST_CHECK(atEvents[3].eType == MOTOR_EVENT_TRANSITION_COMPLETED);
        TEST_CHECK(atEvents[3].wPreviousValue == MOTOR_STARTUP_BLEND_ANGLE);
        TEST_CHECK(atEvents[3].wCurrentValue == MOTOR_STARTUP_COMPLETE);
        for (unsigned uIndex = 1U; uIndex < 4U; uIndex++) {
            TEST_CHECK(atEvents[uIndex].wSequence ==
                       atEvents[uIndex - 1U].wSequence + 1U);
        }
    }
    return nFailures;
}

int test_motor_control_runtime(void)
{
    int nFailures = 0;

    TEST_CHECK(motor_TestGetImplementationSize() <=
               MOTOR_HANDLE_STORAGE_SIZE);
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
    nFailures += test_transition_qualification_and_wrap();
    nFailures += test_transition_requires_consecutive_samples();
    nFailures += test_transition_blend_uses_frozen_angle_bound();
    nFailures += test_transition_source_receives_control_input();
    nFailures += test_transition_failure(false);
    nFailures += test_transition_failure(true);
    nFailures += test_transition_events();
    return nFailures;
}
