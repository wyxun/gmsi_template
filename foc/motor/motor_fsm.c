#include "motor.h"
#include "motor_private.h"

static uint16_t command_payload(motor_command_e command,
                                foc_result_t result)
{
    return (uint16_t)((uint16_t)(uint8_t)command |
                      ((uint16_t)(uint8_t)result << 8));
}

static foc_result_t record_command_result(motor_impl_t *impl,
                                          motor_command_e command,
                                          foc_result_t result)
{
    uintptr_t state = motor_private_enter(impl);
    motor_private_AppendEvent(
        impl,
        result == FOC_RESULT_OK ? MOTOR_EVENT_COMMAND_ACCEPTED :
                                  MOTOR_EVENT_COMMAND_REJECTED,
        impl->tRt.eRunState, impl->tRt.eRunState, 0U,
        command_payload(command, result));
    motor_private_exit(impl, state);
    return result;
}

static bool tracking_controller_is_valid(
    const motor_tracking_controller_if_t *controller)
{
    return controller->pContext != NULL && controller->fnStep != NULL;
}

static foc_result_t validate_run(const motor_run_config_t *run)
{
    bool has_source;
    if (run == NULL) return FOC_RESULT_NULL;
    if (run->eControlMode < MOTOR_CONTROL_VOLTAGE_OPEN_LOOP ||
        run->eControlMode > MOTOR_CONTROL_POSITION)
        return FOC_RESULT_INVALID_ARGUMENT;
    if ((run->ptInitialPositionSource != NULL &&
         !foc_position_source_IsValid(run->ptInitialPositionSource)) ||
        (run->ptTargetPositionSource != NULL &&
         !foc_position_source_IsValid(run->ptTargetPositionSource)))
        return FOC_RESULT_INVALID_ARGUMENT;
    if (run->ptInitialPositionSource != NULL &&
        run->ptTargetPositionSource != NULL &&
        run->ptInitialPositionSource != run->ptTargetPositionSource)
        return FOC_RESULT_INVALID_ARGUMENT;
    has_source = run->ptInitialPositionSource != NULL ||
                 run->ptTargetPositionSource != NULL;
    if (run->eControlMode >= MOTOR_CONTROL_SPEED && !has_source)
        return FOC_RESULT_INVALID_ARGUMENT;
    return FOC_RESULT_OK;
}

static void save_run(motor_impl_t *impl, const motor_run_config_t *run)
{
    impl->tControl.eMode = run->eControlMode;
    impl->tControl.tVoltageReference = run->tVoltageReference;
    impl->tControl.tCurrentReference = run->tCurrentReference;
    impl->tControl.qSpeedReference = run->qSpeedReference;
    impl->tControl.qPositionReference = run->qPositionReference;
    impl->bInitialPositionSourceBound =
        run->ptInitialPositionSource != NULL;
    impl->bTargetPositionSourceBound = run->ptTargetPositionSource != NULL;
    impl->bOuterLoopActive = impl->bInitialPositionSourceBound;
    foc_open_loop_source_Init(&impl->tDefaultOpenLoopSource);
    foc_open_loop_source_SetSpeed(&impl->tDefaultOpenLoopSource, run->qOpenLoopSpeed);
    foc_open_loop_source_SetTargetSpeed(&impl->tDefaultOpenLoopSource, run->qOpenLoopSpeed);
    foc_open_loop_source_SetAcceleration(&impl->tDefaultOpenLoopSource, run->qAcceleration);
    foc_open_loop_source_SetAngle(&impl->tDefaultOpenLoopSource, foc_angle_from_scalar(run->qInitialAngle));

    if (impl->bInitialPositionSourceBound)
        impl->tPositionSource = *run->ptInitialPositionSource;
    else if (impl->bTargetPositionSourceBound)
        impl->tPositionSource = *run->ptTargetPositionSource;
    else
        impl->tPositionSource = foc_open_loop_source_GetInterface(&impl->tDefaultOpenLoopSource);

    impl->qOpenLoopCommandSpeed = run->qOpenLoopSpeed;
    impl->wPositionSampleTimestamp = 0U;
    impl->hwTransitionSampleCount = 0U;
    impl->tRt.tThetaE = foc_angle_from_scalar(run->qInitialAngle);
    impl->tCandidateAngle = (foc_angle_t){0};
    impl->qCandidateSpeed = FOC_ZERO;
    impl->qAngleError = FOC_ZERO;
    impl->qBlendFactor = FOC_ZERO;
    impl->chActiveValidFlags = 0U;
    impl->chCandidateValidFlags = 0U;
}

foc_result_t motor_Start(motor_handle_t *motor,
                         const motor_run_config_t *run)
{
    motor_impl_t *impl;
    foc_result_t result;
    uintptr_t sync_state;
    if (!motor_private_is_initialized(motor))
        return motor ? FOC_RESULT_INVALID_ARGUMENT : FOC_RESULT_NULL;
    result = validate_run(run);
    impl = motor_private(motor);
    if (result != FOC_RESULT_OK)
        return record_command_result(impl, MOTOR_COMMAND_START, result);
    if (run->eControlMode >= MOTOR_CONTROL_CURRENT &&
        (impl->tControl.tConfig.tIdController.pContext == NULL ||
         impl->tControl.tConfig.tIdController.fnStep == NULL ||
         impl->tControl.tConfig.tIqController.pContext == NULL ||
         impl->tControl.tConfig.tIqController.fnStep == NULL))
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    if (run->eControlMode >= MOTOR_CONTROL_SPEED &&
        !tracking_controller_is_valid(
            &impl->tControl.tConfig.tSpeedController))
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    if (run->eControlMode >= MOTOR_CONTROL_POSITION &&
        !tracking_controller_is_valid(
            &impl->tControl.tConfig.tPositionController))
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    if (run->ptInitialPositionSource == NULL &&
        run->ptTargetPositionSource != NULL &&
        impl->tTime.fnGetMilliseconds == NULL)
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    if (run->ptInitialPositionSource == NULL &&
        run->ptTargetPositionSource != NULL &&
        run->eControlMode >= MOTOR_CONTROL_SPEED &&
        impl->tControl.tConfig.tSpeedController.fnTrack == NULL)
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    if (run->ptInitialPositionSource == NULL &&
        run->ptTargetPositionSource != NULL &&
        run->eControlMode >= MOTOR_CONTROL_POSITION &&
        impl->tControl.tConfig.tPositionController.fnTrack == NULL)
        return record_command_result(impl, MOTOR_COMMAND_START,
                                     FOC_RESULT_INVALID_ARGUMENT);
    sync_state = motor_private_enter(impl);
    if (impl->bCommandPending) {
        motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_REJECTED,
            impl->tRt.eRunState, impl->tRt.eRunState, 0U,
            command_payload(MOTOR_COMMAND_START, FOC_RESULT_BUSY));
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_BUSY;
    }
    if (impl->tRt.eRunState != MOTOR_STATE_IDLE) {
        motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_REJECTED,
            impl->tRt.eRunState, impl->tRt.eRunState, 0U,
            command_payload(MOTOR_COMMAND_START,
                            FOC_RESULT_INVALID_ARGUMENT));
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    save_run(impl, run);
    impl->chPendingCommand = MOTOR_COMMAND_START;
    impl->bCommandPending = true;
    motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_ACCEPTED,
        impl->tRt.eRunState, impl->tRt.eRunState, 0U,
        command_payload(MOTOR_COMMAND_START, FOC_RESULT_OK));
    motor_private_exit(impl, sync_state);
    return FOC_RESULT_OK;
}

foc_result_t motor_Stop(motor_handle_t *motor)
{
    motor_impl_t *impl;
    uintptr_t sync_state;
    if (!motor_private_is_initialized(motor))
        return motor ? FOC_RESULT_INVALID_ARGUMENT : FOC_RESULT_NULL;
    impl = motor_private(motor);
    sync_state = motor_private_enter(impl);
    if (impl->bCommandPending &&
        impl->chPendingCommand == MOTOR_COMMAND_START) {
        impl->chPendingCommand = MOTOR_COMMAND_STOP;
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_OK;
    }
    if (impl->bCommandPending) {
        motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_REJECTED,
            impl->tRt.eRunState, impl->tRt.eRunState, 0U,
            command_payload(MOTOR_COMMAND_STOP, FOC_RESULT_BUSY));
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_BUSY;
    }
    if (impl->tRt.eRunState != MOTOR_STATE_STARTING &&
        impl->tRt.eRunState != MOTOR_STATE_RUNNING)
    {
        motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_REJECTED,
            impl->tRt.eRunState, impl->tRt.eRunState, 0U,
            command_payload(MOTOR_COMMAND_STOP,
                            FOC_RESULT_INVALID_ARGUMENT));
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    impl->chPendingCommand = MOTOR_COMMAND_STOP;
    impl->bCommandPending = true;
    motor_private_AppendEvent(impl, MOTOR_EVENT_COMMAND_ACCEPTED,
        impl->tRt.eRunState, impl->tRt.eRunState, 0U,
        command_payload(MOTOR_COMMAND_STOP, FOC_RESULT_OK));
    motor_private_exit(impl, sync_state);
    return FOC_RESULT_OK;
}

static fsm_rt_t stop_now(motor_handle_t *motor)
{
    motor_impl_t *impl = motor_private(motor);
    uintptr_t sync_state;
    foc_result_t result;
    sync_state = motor_private_enter(impl);
    /* PWM enable/disable callbacks must be bounded register writes and must
       not call back into motor APIs; this is the sole HAL-in-sync exception. */
    result = foc_hal_Enable(&impl->tHal.tPwm, false);
    if (result == FOC_RESULT_OK) {
        impl->bPwmEnabled = false;
        impl->tRt.eRunState = MOTOR_STATE_IDLE;
        impl->chStartupPhase = MOTOR_STARTUP_IDLE;
    }
    motor_private_exit(impl, sync_state);
    if (result != FOC_RESULT_OK) {
        motor_EmergencyStop(motor, MOTOR_FAULT_HARDWARE);
        return fsm_rt_err;
    }
    return fsm_rt_cpl;
}

static fsm_rt_t invalid_fsm(motor_handle_t *motor)
{
    motor_EmergencyStop(motor, MOTOR_FAULT_INVALID_COMMAND);
    return fsm_rt_err;
}

static bool commit_transition_timeout(motor_impl_t *impl)
{
    uintptr_t sync_state = motor_private_enter(impl);
    bool active =
        impl->tRt.eRunState == MOTOR_STATE_STARTING &&
        (impl->chStartupPhase == MOTOR_STARTUP_QUALIFY_SOURCE ||
         impl->chStartupPhase == MOTOR_STARTUP_BLEND_ANGLE) &&
        !impl->bCommandPending;

    if (active) {
        motor_private_AppendEvent(impl, MOTOR_EVENT_TRANSITION_TIMEOUT,
            impl->tRt.eRunState, MOTOR_STATE_FAULT, 0U,
            (uint16_t)((uint16_t)impl->chStartupPhase |
                       ((uint16_t)MOTOR_STARTUP_IDLE << 8)));
        impl->tRt.wFaults |= MOTOR_FAULT_TRANSITION_TIMEOUT;
        impl->tRt.eRunState = MOTOR_STATE_FAULT;
        impl->bPwmEnabled = false;
        impl->bCommandPending = false;
        impl->chPendingCommand = MOTOR_COMMAND_NONE;
    }
    motor_private_exit(impl, sync_state);
    if (active) {
        foc_hal_EmergencyStop(&impl->tHal.tPwm);
    }
    return active;
}

#if defined(MOTOR_ENABLE_TEST_HOOKS)
bool motor_TestCommitTransitionTimeout(motor_handle_t *motor)
{
    if (!motor_private_is_initialized(motor)) {
        return false;
    }
    return commit_transition_timeout(motor_private(motor));
}
#endif

static fsm_rt_t run_startup(motor_handle_t *motor, motor_impl_t *impl,
                            motor_startup_phase_e phase)
{
    uint32_t now;
    uintptr_t sync_state;
    foc_result_t result;
    switch (phase) {
        case MOTOR_STARTUP_CALIBRATE:
            result = motor_private_CalibrateCurrent(motor);
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                impl->chStartupPhase != MOTOR_STARTUP_CALIBRATE ||
                impl->bCommandPending) {
                motor_state_e current_state = impl->tRt.eRunState;
                motor_private_exit(impl, sync_state);
                return current_state == MOTOR_STATE_FAULT ?
                       fsm_rt_err : fsm_rt_on_going;
            }
            if (result != FOC_RESULT_OK || impl->bPwmEnabled) {
                motor_private_exit(impl, sync_state);
                motor_EmergencyStop(motor, MOTOR_FAULT_HARDWARE);
                return fsm_rt_err;
            }
            impl->chStartupPhase = MOTOR_STARTUP_WAIT_DELAY;
            motor_private_exit(impl, sync_state);
            now = impl->tTime.fnGetMilliseconds ?
                impl->tTime.fnGetMilliseconds(impl->tTime.pContext) : 0U;
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState == MOTOR_STATE_STARTING &&
                impl->chStartupPhase == MOTOR_STARTUP_WAIT_DELAY)
                impl->wStartupStartMs = now;
            motor_private_exit(impl, sync_state);
            return fsm_rt_on_going;
        case MOTOR_STARTUP_WAIT_DELAY:
            now = impl->tTime.fnGetMilliseconds ?
                  impl->tTime.fnGetMilliseconds(impl->tTime.pContext) : 0U;
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState == MOTOR_STATE_STARTING &&
                impl->chStartupPhase == MOTOR_STARTUP_WAIT_DELAY &&
                !impl->bCommandPending &&
                (uint32_t)(now - impl->wStartupStartMs) >=
                    impl->wStartupDelayMs)
                impl->chStartupPhase = MOTOR_STARTUP_ENABLE;
            motor_private_exit(impl, sync_state);
            return fsm_rt_on_going;
        case MOTOR_STARTUP_ENABLE:
        {
            bool transition_required;
            motor_time_if_t time;

            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                impl->chStartupPhase != MOTOR_STARTUP_ENABLE ||
                impl->bCommandPending) {
                motor_private_exit(impl, sync_state);
                return fsm_rt_on_going;
            }
            transition_required =
                !impl->bInitialPositionSourceBound &&
                impl->bTargetPositionSourceBound;
            time = impl->tTime;
            motor_private_exit(impl, sync_state);
            now = transition_required ?
                  time.fnGetMilliseconds(time.pContext) : 0U;
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                impl->chStartupPhase != MOTOR_STARTUP_ENABLE ||
                impl->bCommandPending) {
                motor_private_exit(impl, sync_state);
                return fsm_rt_on_going;
            }
            result = foc_hal_Enable(&impl->tHal.tPwm, true);
            if (result == FOC_RESULT_OK) {
                impl->bPwmEnabled = true;
                if (transition_required) {
                    impl->wStartupStartMs = now;
                    impl->chStartupPhase =
                        MOTOR_STARTUP_QUALIFY_SOURCE;
                } else {
                    impl->tRt.eRunState = MOTOR_STATE_RUNNING;
                    impl->chStartupPhase = MOTOR_STARTUP_IDLE;
                }
            }
            motor_private_exit(impl, sync_state);
            if (result != FOC_RESULT_OK) {
                motor_EmergencyStop(motor, MOTOR_FAULT_HARDWARE);
                return fsm_rt_err;
            }
            return transition_required ? fsm_rt_on_going : fsm_rt_cpl;
        }
        case MOTOR_STARTUP_QUALIFY_SOURCE:
        case MOTOR_STARTUP_BLEND_ANGLE:
        {
            motor_time_if_t time;
            uint32_t transition_start;
            uint32_t transition_timeout;
            bool timed_out;

            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                (impl->chStartupPhase != MOTOR_STARTUP_QUALIFY_SOURCE &&
                 impl->chStartupPhase != MOTOR_STARTUP_BLEND_ANGLE) ||
                impl->bCommandPending) {
                motor_state_e current_state = impl->tRt.eRunState;
                motor_private_exit(impl, sync_state);
                return current_state == MOTOR_STATE_FAULT ?
                       fsm_rt_err : fsm_rt_on_going;
            }
            time = impl->tTime;
            transition_start = impl->wStartupStartMs;
            transition_timeout = impl->wTransitionTimeoutMs;
            motor_private_exit(impl, sync_state);
            now = time.fnGetMilliseconds(time.pContext);
            timed_out =
                (uint32_t)(now - transition_start) >= transition_timeout;
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                (impl->chStartupPhase != MOTOR_STARTUP_QUALIFY_SOURCE &&
                 impl->chStartupPhase != MOTOR_STARTUP_BLEND_ANGLE) ||
                impl->bCommandPending) {
                motor_state_e current_state = impl->tRt.eRunState;
                motor_private_exit(impl, sync_state);
                return current_state == MOTOR_STATE_FAULT ?
                       fsm_rt_err : fsm_rt_on_going;
            }
            motor_private_exit(impl, sync_state);
            if (timed_out) {
                return commit_transition_timeout(impl) ?
                       fsm_rt_err : fsm_rt_on_going;
            }
            return fsm_rt_on_going;
        }
        case MOTOR_STARTUP_COMPLETE:
            sync_state = motor_private_enter(impl);
            if (impl->tRt.eRunState != MOTOR_STATE_STARTING ||
                impl->chStartupPhase != MOTOR_STARTUP_COMPLETE ||
                impl->bCommandPending) {
                motor_private_exit(impl, sync_state);
                return fsm_rt_on_going;
            }
            impl->bOuterLoopActive = true;
            impl->tRt.eRunState = MOTOR_STATE_RUNNING;
            impl->chStartupPhase = MOTOR_STARTUP_IDLE;
            motor_private_exit(impl, sync_state);
            return fsm_rt_cpl;
        case MOTOR_STARTUP_IDLE:
        default:
            return invalid_fsm(motor);
    }
}

fsm_rt_t motor_RunFSM(motor_handle_t *motor)
{
    motor_impl_t *impl;
    uintptr_t sync_state;
    motor_command_e cmd = MOTOR_COMMAND_NONE;
    motor_state_e state;
    motor_startup_phase_e phase;
    if (!motor_private_is_initialized(motor)) return fsm_rt_err;
    impl = motor_private(motor);
    sync_state = motor_private_enter(impl);
    if (impl->bCommandPending) {
        motor_state_e previous_state = impl->tRt.eRunState;
        cmd = impl->chPendingCommand;
        if (cmd == MOTOR_COMMAND_START) {
            impl->tRt.eRunState = MOTOR_STATE_STARTING;
            impl->chStartupPhase = MOTOR_STARTUP_CALIBRATE;
        } else if (cmd == MOTOR_COMMAND_STOP)
            impl->tRt.eRunState = MOTOR_STATE_STOPPING;
        impl->bCommandPending = false;
        impl->chPendingCommand = MOTOR_COMMAND_NONE;
        if (previous_state != impl->tRt.eRunState) {
            motor_private_AppendEvent(impl, MOTOR_EVENT_STATE_CHANGED,
                previous_state, impl->tRt.eRunState, 0U, 0U);
        }
    }
    state = impl->tRt.eRunState;
    phase = impl->chStartupPhase;
    motor_private_exit(impl, sync_state);
    if (cmd == MOTOR_COMMAND_STOP) return stop_now(motor);
    if (cmd != MOTOR_COMMAND_START && cmd != MOTOR_COMMAND_NONE &&
        cmd != MOTOR_COMMAND_STOP) return invalid_fsm(motor);
    switch (state) {
        case MOTOR_STATE_IDLE: return fsm_rt_cpl;
        case MOTOR_STATE_STARTING: return run_startup(motor, impl, phase);
        case MOTOR_STATE_STOPPING: return stop_now(motor);
        case MOTOR_STATE_RUNNING: return fsm_rt_cpl;
        case MOTOR_STATE_FAULT: return fsm_rt_err;
        default: return invalid_fsm(motor);
    }
}

foc_result_t motor_ClearFault(motor_handle_t *motor)
{
    motor_impl_t *impl;
    uintptr_t sync_state;
    if (!motor_private_is_initialized(motor))
        return motor ? FOC_RESULT_INVALID_ARGUMENT : FOC_RESULT_NULL;
    impl = motor_private(motor);
    sync_state = motor_private_enter(impl);
    if (impl->tRt.eRunState != MOTOR_STATE_FAULT || impl->bPwmEnabled) {
        motor_private_exit(impl, sync_state);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    impl->tRt.wFaults = MOTOR_FAULT_NONE;
    impl->tRt.eRunState = MOTOR_STATE_IDLE;
    impl->chStartupPhase = MOTOR_STARTUP_IDLE;
    impl->bCommandPending = false;
    impl->chPendingCommand = MOTOR_COMMAND_NONE;
    motor_private_exit(impl, sync_state);
    return FOC_RESULT_OK;
}

#if defined(MOTOR_ENABLE_TEST_HOOKS)
void motor_test_CorruptFSM(motor_handle_t *motor, motor_state_e state,
                           motor_startup_phase_e phase)
{
    if (motor_private_is_initialized(motor)) {
        motor_private(motor)->tRt.eRunState = state;
        motor_private(motor)->chStartupPhase = phase;
    }
}

bool motor_test_PositionBindingsValid(const motor_handle_t *motor)
{
    const motor_impl_t *impl;
    if (!motor_private_is_initialized(motor)) return false;
    impl = motor_private_const(motor);
    return (!impl->bInitialPositionSourceBound ||
            foc_position_source_IsValid(&impl->tPositionSource)) &&
           (!impl->bTargetPositionSourceBound ||
            foc_position_source_IsValid(&impl->tPositionSource));
}
#endif
