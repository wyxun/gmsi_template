/*******************************************************************************
 * @file    motor_control.c
 * @brief   Per-instance FOC loop orchestration
 ******************************************************************************/

#include "motor_control.h"
#include "motor_private.h"
#include "foc_hf_profile.h"
#include "perf_counter.h"

#include <stddef.h>

static foc_result_t motor_control_begin_step(motor_impl_t *, bool);
static void motor_control_end_step(motor_impl_t *, bool);

typedef struct {
    motor_startup_phase_e ePhase;
    foc_angle_t tStartAngle;
    foc_scalar_t qStartSpeed;
    uint16_t hwSampleCount;
    bool bChanged;
} motor_transition_update_t;

static foc_result_t motor_control_modulate(motor_control_t *ptControl)
{
    switch (ptControl->tConfig.eModulation) {
        case MOTOR_MODULATION_SPWM:
            return foc_spwm(&ptControl->tVoltageAlphaBeta,
                            &ptControl->tDuty);
        case MOTOR_MODULATION_THIRD_HARMONIC:
            return foc_third_harmonic_spwm(&ptControl->tVoltageAlphaBeta,
                                           &ptControl->tDuty);
        case MOTOR_MODULATION_SVPWM:
        default:
            return foc_svpwm(&ptControl->tVoltageAlphaBeta,
                             &ptControl->tDuty);
    }
}

void motor_SetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.tVoltageReference = (foc_dq_t){qD, qQ};
        motor_private_exit(p, s);
    }
}

void motor_SetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.tCurrentReference = (foc_dq_t){qD, qQ};
        motor_private_exit(p, s);
    }
}

void motor_SetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.qSpeedReference = qSpeed;
        motor_private_exit(p, s);
    }
}

void motor_SetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.qPositionReference = qPosition;
        motor_private_exit(p, s);
    }
}

foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    motor_control_t *ptControl;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    ptControl = &ptImpl->tControl;
    foc_result_t begin = motor_control_begin_step(ptImpl, false);
    if (begin != FOC_RESULT_OK) return begin;
    uintptr_t s = motor_private_enter(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING &&
        ptImpl->tRt.eRunState != MOTOR_STATE_STARTING) {
        motor_private_exit(ptImpl, s);
        motor_control_end_step(ptImpl, false);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    motor_control_mode_e mode = ptControl->eMode;
    foc_scalar_t position_ref = ptControl->qPositionReference;
    foc_scalar_t speed_ref = ptControl->qSpeedReference;
    foc_scalar_t id_ref = ptControl->tCurrentReference.qD;
    foc_angle_t mechanical_angle = ptImpl->tMechanicalAngle;
    foc_scalar_t mechanical_speed = ptImpl->qMechanicalSpeed;
    foc_position_valid_flag_e mechanical_valid =
        ptImpl->chMechanicalValidFlags;
    bool direct_source = ptImpl->bOuterLoopActive;
    motor_private_exit(ptImpl, s);
    if (mode >= MOTOR_CONTROL_SPEED && !direct_source) {
        motor_control_end_step(ptImpl, false);
        return FOC_RESULT_OK;
    }
    if (mode == MOTOR_CONTROL_POSITION) {
        if ((mechanical_valid & FOC_POSITION_VALID_MECHANICAL_ANGLE) == 0U) {
            goto fail;
        }
        foc_scalar_t error = foc_angle_diff(
            foc_angle_from_scalar(position_ref), mechanical_angle);
        speed_ref =
            ptControl->tConfig.tPositionController.fnStep(
                ptControl->tConfig.tPositionController.pContext,
                error, FOC_ZERO);
    }
    if (mode >= MOTOR_CONTROL_SPEED) {
        if ((mechanical_valid & FOC_POSITION_VALID_MECHANICAL_SPEED) == 0U) {
            goto fail;
        }
        foc_scalar_t iq_ref =
            ptControl->tConfig.tSpeedController.fnStep(
                ptControl->tConfig.tSpeedController.pContext,
                speed_ref, mechanical_speed);
        s = motor_private_enter(ptImpl);
        bool stopping = ptImpl->bCommandPending &&
                        ptImpl->chPendingCommand == MOTOR_COMMAND_STOP;
        if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING || stopping) {
            ptImpl->bLowFrequencyStepInProgress = false;
            motor_private_exit(ptImpl, s);
            return stopping ? FOC_RESULT_BUSY : FOC_RESULT_INVALID_ARGUMENT;
        }
        ptControl->qSpeedReference = speed_ref;
        ptControl->tCurrentReference = (foc_dq_t){id_ref, iq_ref};
        ptImpl->bLowFrequencyStepInProgress = false;
        motor_private_exit(ptImpl, s);
    } else {
        motor_control_end_step(ptImpl, false);
    }
    return FOC_RESULT_OK;
fail:
    motor_control_end_step(ptImpl, false);
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_INVALID_COMMAND);
    return FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t motor_control_begin_step(motor_impl_t *impl, bool hf)
{
    uintptr_t state = motor_private_enter(impl);
    bool *active = hf ? &impl->bHighFrequencyStepInProgress :
                        &impl->bLowFrequencyStepInProgress;
    if (*active) {
        motor_private_exit(impl, state);
        return FOC_RESULT_BUSY;
    }
    *active = true;
    motor_private_exit(impl, state);
    return FOC_RESULT_OK;
}

static void motor_control_end_step(motor_impl_t *impl, bool hf)
{
    uintptr_t state = motor_private_enter(impl);
    if (hf) impl->bHighFrequencyStepInProgress = false;
    else impl->bLowFrequencyStepInProgress = false;
    motor_private_exit(impl, state);
}

static foc_result_t motor_control_commit_hf(
    motor_impl_t *impl, const motor_control_t *work,
    foc_angle_t angle, foc_scalar_t speed,
    const foc_position_output_t *output,
    const motor_transition_update_t *transition,
    foc_scalar_t angle_error, foc_scalar_t blend_factor,
    bool direct_source, bool candidate_source,
    bool *hardware_failed)
{
    uintptr_t state = motor_private_enter(impl);
    bool stopping = impl->bCommandPending &&
                    impl->chPendingCommand == MOTOR_COMMAND_STOP;
    if ((impl->tRt.eRunState != MOTOR_STATE_RUNNING &&
         impl->tRt.eRunState != MOTOR_STATE_STARTING) ||
        impl->tRt.wFaults != MOTOR_FAULT_NONE ||
        !impl->bPwmEnabled || stopping) {
        impl->bHighFrequencyStepInProgress = false;
        motor_private_exit(impl, state);
        return stopping ? FOC_RESULT_BUSY : FOC_RESULT_INVALID_ARGUMENT;
    }
    foc_result_t result = foc_hal_SetDuty(&impl->tHal.tPwm,
        work->tDuty.qU, work->tDuty.qV, work->tDuty.qW);
    *hardware_failed = result != FOC_RESULT_OK;
    if (result == FOC_RESULT_OK) {
        impl->tRt.tThetaE = angle;
        impl->tRt.qOmegaE = speed;
        impl->qOpenLoopCommandSpeed = speed;
        impl->tCandidateAngle = output->tElectricalAngle;
        impl->qCandidateSpeed = output->qElectricalSpeed;
        impl->qAngleError = angle_error;
        impl->qBlendFactor = blend_factor;
        uint8_t active_valid = direct_source ?
            (uint8_t)output->eValidFlags :
            (uint8_t)(FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                      FOC_POSITION_VALID_ELECTRICAL_SPEED);
        uint8_t candidate_valid = candidate_source ?
            (uint8_t)output->eValidFlags : 0U;
        if (active_valid != impl->chActiveValidFlags) {
            motor_private_AppendEvent(
                impl, MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                impl->tRt.eRunState, impl->tRt.eRunState,
                MOTOR_POSITION_ROLE_ACTIVE,
                (uint16_t)((uint16_t)impl->chActiveValidFlags |
                           ((uint16_t)active_valid << 8)));
            impl->chActiveValidFlags = active_valid;
        }
        if (candidate_valid != impl->chCandidateValidFlags) {
            motor_private_AppendEvent(
                impl, MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                impl->tRt.eRunState, impl->tRt.eRunState,
                MOTOR_POSITION_ROLE_CANDIDATE,
                (uint16_t)((uint16_t)impl->chCandidateValidFlags |
                           ((uint16_t)candidate_valid << 8)));
            impl->chCandidateValidFlags = candidate_valid;
        }
        impl->tMechanicalAngle = output->tMechanicalAngle;
        impl->qMechanicalSpeed = output->qMechanicalSpeed;
        impl->chMechanicalValidFlags =
            (uint8_t)(output->eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED));
        if (transition->bChanged) {
            motor_startup_phase_e previous_phase = impl->chStartupPhase;
            impl->chStartupPhase = transition->ePhase;
            impl->tTransitionStartAngle = transition->tStartAngle;
            impl->qTransitionStartSpeed = transition->qStartSpeed;
            impl->hwTransitionSampleCount = transition->hwSampleCount;
            if (previous_phase != transition->ePhase &&
                transition->ePhase == MOTOR_STARTUP_BLEND_ANGLE) {
                motor_private_AppendEvent(
                    impl, MOTOR_EVENT_TRANSITION_STARTED,
                    impl->tRt.eRunState, impl->tRt.eRunState, 0U,
                    (uint16_t)((uint16_t)previous_phase |
                               ((uint16_t)transition->ePhase << 8)));
            } else if (previous_phase != transition->ePhase &&
                       transition->ePhase == MOTOR_STARTUP_COMPLETE) {
                motor_private_AppendEvent(
                    impl, MOTOR_EVENT_TRANSITION_COMPLETED,
                    impl->tRt.eRunState, impl->tRt.eRunState, 0U,
                    (uint16_t)((uint16_t)previous_phase |
                               ((uint16_t)transition->ePhase << 8)));
            }
        }
        impl->tControl.tCurrent = work->tCurrent;
        impl->tControl.tVoltage = work->tVoltage;
        impl->tControl.tVoltageAlphaBeta = work->tVoltageAlphaBeta;
        impl->tControl.tDuty = work->tDuty;
    }
    impl->bHighFrequencyStepInProgress = false;
    motor_private_exit(impl, state);
    return result;
}

static bool motor_control_candidate_qualified(
    const foc_position_output_t *output,
    const foc_position_qualification_t *qualification,
    foc_angle_t reference_angle,
    foc_scalar_t reference_speed,
    uint32_t timestamp)
{
    foc_position_qualification_t sample = *qualification;

    sample.tReferenceAngle = reference_angle;
    sample.qReferenceSpeed = reference_speed;
    sample.wNow = timestamp;
    return foc_position_IsQualified(output, &sample);
}

static foc_scalar_t motor_control_blend_progress(uint16_t hwSample,
                                                 uint16_t hwTotal)
{
    return foc_from_float((float)hwSample / (float)hwTotal);
}

foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    motor_control_t *ptControl;
    foc_result_t eResult;
    foc_dq_t voltage_ref, current_ref;
    foc_ab_t current_alpha_beta;
    motor_control_t work;
    motor_control_mode_e mode;
    uintptr_t s;
    motor_transition_update_t transition = {0};
    uint32_t position_timestamp;
    foc_scalar_t high_frequency_period;
    foc_scalar_t angle_error = FOC_ZERO;
    foc_scalar_t blend_factor = FOC_ZERO;
    foc_position_config_t position_config;
    foc_position_qualification_t qualification;
    uint16_t qualification_samples;
    uint16_t blend_samples;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    eResult = motor_control_begin_step(ptImpl, true);
    if (eResult != FOC_RESULT_OK) return eResult;
    s = motor_private_enter(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_STARTING &&
        ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING) {
        motor_private_exit(ptImpl, s);
        motor_control_end_step(ptImpl, true);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptControl = &ptImpl->tControl;
    work = *ptControl;
    mode = ptControl->eMode;
    voltage_ref = ptControl->tVoltageReference;
    current_ref = ptControl->tCurrentReference;
    bool direct_source = ptImpl->bInitialPositionSourceBound;
    bool candidate_source = ptImpl->bTargetPositionSourceBound;
    bool transition_required = !direct_source && candidate_source;
    foc_position_source_if_t source = ptImpl->tPositionSource;
    foc_angle_t angle = ptImpl->tRt.tThetaE;
    foc_scalar_t speed = ptImpl->qOpenLoopCommandSpeed;
    motor_startup_phase_e startup_phase = ptImpl->chStartupPhase;
    uint16_t transition_samples = ptImpl->hwTransitionSampleCount;
    foc_angle_t transition_start_angle =
        ptImpl->tTransitionStartAngle;
    foc_scalar_t transition_start_speed =
        ptImpl->qTransitionStartSpeed;
    high_frequency_period = ptImpl->qHighFrequencyPeriod;
    position_config = ptImpl->tPositionConfig;
    qualification = (foc_position_qualification_t){
        .eRequiredValid = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                          FOC_POSITION_VALID_ELECTRICAL_SPEED,
        .qMinimumConfidence = ptImpl->qTransitionMinimumConfidence,
        .qMinimumSpeed = ptImpl->qTransitionMinimumSpeed,
        .qMaximumAngleError = ptImpl->qTransitionMaximumAngleError,
        .wMaximumAge = 0U,
    };
    qualification_samples = ptImpl->hwTransitionQualificationSamples;
    blend_samples = ptImpl->hwTransitionBlendSamples;
    position_timestamp = ++ptImpl->wPositionSampleTimestamp;
    if (position_timestamp == 0U) {
        position_timestamp = ++ptImpl->wPositionSampleTimestamp;
    }
    motor_private_exit(ptImpl, s);
#if FOC_HF_PROFILE
    uint32_t wTotalCycles = 0;
    uint32_t wSampleCurrentCycles = 0;
    uint32_t wClarkeCycles = 0;
    uint32_t wParkCycles = 0;
    uint32_t wIparkCycles = 0;
    uint32_t wModulateCycles = 0;
    uint32_t wCommitCycles = 0;
    uint32_t wValidFlags = 0;

    FOC_HF_PROFILE_TOTAL_BEGIN(tTotalStart);
#endif

    FOC_HF_PROFILE_STAGE_BEGIN(tSampleStart);
    eResult = motor_private_SampleCurrent(ptMotor);
    FOC_HF_PROFILE_STAGE_END(tSampleStart, wSampleCurrentCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_SAMPLE_CURRENT;
#endif
    if (eResult != FOC_RESULT_OK) {
        motor_control_end_step(ptImpl, true);
        goto publish_exit;
    }

    FOC_HF_PROFILE_STAGE_BEGIN(tClarkeStart);
    eResult = foc_clarke(ptImpl->tCurrent.qIu,
                         ptImpl->tCurrent.qIv,
                         ptImpl->tCurrent.qIw,
                         &current_alpha_beta);
    FOC_HF_PROFILE_STAGE_END(tClarkeStart, wClarkeCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_CLARKE;
#endif
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    foc_position_output_t output = {0};
    foc_position_input_t input = {
        .tCurrent = current_alpha_beta,
        .tVoltage = work.tVoltageAlphaBeta,
        .qSamplePeriod = high_frequency_period,
        .wTimestamp = position_timestamp,
    };

    if (transition_required) {
        foc_position_output_t open_loop_output = {0};
        foc_position_source_if_t open_loop_if =
            foc_open_loop_source_GetInterface(&ptImpl->tDefaultOpenLoopSource);
        eResult = foc_position_source_Step(&open_loop_if, &input, &open_loop_output);
        if (eResult != FOC_RESULT_OK) goto position_fail;
        angle = open_loop_output.tElectricalAngle;
        speed = open_loop_output.qElectricalSpeed;

        eResult = foc_position_source_Step(&source, &input, &output);
        if (eResult != FOC_RESULT_OK) goto position_fail;
        if (output.wFaults != 0U) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto position_fail;
        }
        if ((output.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            eResult = foc_position_ApplyMechanicalConfig(
                &position_config, &output);
            if (eResult != FOC_RESULT_OK) goto position_fail;
        }
    } else {
        eResult = foc_position_source_Step(&source, &input, &output);
        if (eResult != FOC_RESULT_OK) goto position_fail;
        if (output.wFaults != 0U) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto position_fail;
        }
        if ((output.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            eResult = foc_position_ApplyMechanicalConfig(
                &position_config, &output);
            if (eResult != FOC_RESULT_OK) goto position_fail;
        }

        if ((output.eValidFlags & FOC_POSITION_VALID_ELECTRICAL_ANGLE) == 0U) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto position_fail;
        }
        angle = output.tElectricalAngle;
        if ((output.eValidFlags & FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U) {
            speed = output.qElectricalSpeed;
        }
    }
    if (transition_required &&
        startup_phase == MOTOR_STARTUP_QUALIFY_SOURCE) {
        if (motor_control_candidate_qualified(
                &output, &qualification, angle, speed,
                position_timestamp)) {
            transition_samples++;
            if (transition_samples >= qualification_samples) {
                transition.ePhase = MOTOR_STARTUP_BLEND_ANGLE;
                transition.tStartAngle = angle;
                transition.qStartSpeed = speed;
                transition.hwSampleCount = 0U;
            } else {
                transition.ePhase = startup_phase;
                transition.tStartAngle = transition_start_angle;
                transition.qStartSpeed = transition_start_speed;
                transition.hwSampleCount = transition_samples;
            }
        } else {
            transition.ePhase = startup_phase;
            transition.tStartAngle = transition_start_angle;
            transition.qStartSpeed = transition_start_speed;
            transition.hwSampleCount = 0U;
        }
        transition.bChanged = true;
    } else if (transition_required &&
               startup_phase == MOTOR_STARTUP_BLEND_ANGLE) {
        foc_position_output_t from = {
            .tElectricalAngle = transition_start_angle,
            .qElectricalSpeed = transition_start_speed,
            .qConfidence = FOC_ONE,
            .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                           FOC_POSITION_VALID_ELECTRICAL_SPEED,
            .wTimestamp = position_timestamp,
        };
        foc_position_output_t blended;

        if (!motor_control_candidate_qualified(
                &output, &qualification, transition_start_angle,
                transition_start_speed, position_timestamp)) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto position_fail;
        }
        transition_samples++;
        angle_error = foc_angle_diff(output.tElectricalAngle,
                                     transition_start_angle);
        blend_factor = motor_control_blend_progress(
            transition_samples, blend_samples);
        eResult = foc_position_Blend(
            &from, &output,
            blend_factor,
            &blended);
        if (eResult != FOC_RESULT_OK) {
            goto fail;
        }
        angle = blended.tElectricalAngle;
        speed = blended.qElectricalSpeed;
        transition.ePhase =
            transition_samples >= blend_samples ?
            MOTOR_STARTUP_COMPLETE : MOTOR_STARTUP_BLEND_ANGLE;
        transition.tStartAngle = transition_start_angle;
        transition.qStartSpeed = transition_start_speed;
        transition.hwSampleCount = transition_samples;
        transition.bChanged = true;
        if (transition.ePhase == MOTOR_STARTUP_COMPLETE &&
            mode >= MOTOR_CONTROL_SPEED) {
            foc_scalar_t speed_reference =
                work.qSpeedReference;

            if (mode >= MOTOR_CONTROL_POSITION) {
                speed_reference = output.qMechanicalSpeed;
                work.tConfig.tPositionController.fnTrack(
                    work.tConfig.tPositionController.pContext,
                    speed_reference, work.qPositionReference,
                    foc_angle_to_turns(output.tMechanicalAngle));
            }
            work.tConfig.tSpeedController.fnTrack(
                work.tConfig.tSpeedController.pContext,
                current_ref.qQ, speed_reference,
                output.qMechanicalSpeed);
        }
    }
    foc_scalar_t qSinTheta, qCosTheta;
    foc_angle_sincos(angle, &qSinTheta, &qCosTheta);

    FOC_HF_PROFILE_STAGE_BEGIN(tParkStart);
    eResult = foc_park_cached(&current_alpha_beta,
                               qSinTheta,
                               qCosTheta,
                               &work.tCurrent);
    FOC_HF_PROFILE_STAGE_END(tParkStart, wParkCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_PARK;
#endif
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    if (mode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP) {
        work.tVoltage = voltage_ref;
    } else {
        work.tVoltage.qD = work.tConfig.tIdController.fnStep(
            work.tConfig.tIdController.pContext,
            current_ref.qD, work.tCurrent.qD);
        work.tVoltage.qQ = work.tConfig.tIqController.fnStep(
            work.tConfig.tIqController.pContext,
            current_ref.qQ, work.tCurrent.qQ);
    }

    FOC_HF_PROFILE_STAGE_BEGIN(tIparkStart);
    eResult = foc_ipark_cached(&work.tVoltage,
                               qSinTheta,
                               qCosTheta,
                               &work.tVoltageAlphaBeta);
    FOC_HF_PROFILE_STAGE_END(tIparkStart, wIparkCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_IPARK;
#endif
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }

    FOC_HF_PROFILE_STAGE_BEGIN(tModulateStart);
    eResult = motor_control_modulate(&work);
    FOC_HF_PROFILE_STAGE_END(tModulateStart, wModulateCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_MODULATE;
#endif
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    bool hardware_failed = false;

    FOC_HF_PROFILE_STAGE_BEGIN(tCommitStart);
    eResult = motor_control_commit_hf(
        ptImpl, &work, angle, speed, &output, &transition,
        angle_error, blend_factor, direct_source, candidate_source,
        &hardware_failed);
    FOC_HF_PROFILE_STAGE_END(tCommitStart, wCommitCycles);
#if FOC_HF_PROFILE_LEVEL >= 2
    wValidFlags |= MOTOR_HF_PROFILE_VALID_COMMIT;
#endif
    if (hardware_failed) goto hardware_fail;
    goto publish_exit;

fail:
    motor_control_end_step(ptImpl, true);
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_INVALID_COMMAND);
    goto publish_exit;
position_fail:
    motor_control_end_step(ptImpl, true);
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_POSITION_SOURCE);
    goto publish_exit;
hardware_fail:
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);

publish_exit:
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_TOTAL_END(tTotalStart, wTotalCycles);
#if FOC_HF_PROFILE_LEVEL >= 1
    wValidFlags |= MOTOR_HF_PROFILE_VALID_TOTAL;
#endif
    ptImpl->tProfileSnapshot.wSampleSequence++;
    ptImpl->tProfileSnapshot.wTotalCycles = wTotalCycles;
    ptImpl->tProfileSnapshot.wSampleCurrentCycles = wSampleCurrentCycles;
    ptImpl->tProfileSnapshot.wClarkeCycles = wClarkeCycles;
    ptImpl->tProfileSnapshot.wParkCycles = wParkCycles;
    ptImpl->tProfileSnapshot.wIparkCycles = wIparkCycles;
    ptImpl->tProfileSnapshot.wModulateCycles = wModulateCycles;
    ptImpl->tProfileSnapshot.wCommitCycles = wCommitCycles;
    ptImpl->tProfileSnapshot.wValidFlags = wValidFlags;
    ptImpl->tProfileSnapshot.eResult = eResult;
#endif
    return eResult;
}
