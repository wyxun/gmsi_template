/*******************************************************************************
 * @file    motor_hf.c
 * @brief   高频 Fast Path 控制内核：电机 20 kHz 主控制步进循环
 *
 * 在绑定期校验并缓存 I/O、位置源、Id/Iq 控制器及调制器回调；
 * 高频 ISR（motor_HighFrequencyStep）直接调度已解析的回调，消除二次服务层
 * 转发与重复校验。
 ******************************************************************************/

#include "motor.h"
#include "motor_private.h"
#include "motor_hf_private.h"
#include "foc_modulation.h"
#include "foc_hf_profile.h"

static foc_result_t motor_hf_fault(motor_impl_t *impl,
                                   motor_fault_e fault,
                                   foc_result_t result)
{
    uintptr_t s = motor_private_enter(impl);
    impl->tRt.wFaults |= (uint32_t)fault;
    impl->tRt.eRunState = MOTOR_STATE_FAULT;
    impl->bPwmEnabled = false;
    impl->bHighFrequencyStepInProgress = false;
    motor_private_AppendEvent(impl, MOTOR_EVENT_FAULT, MOTOR_STATE_FAULT,
                              MOTOR_STATE_FAULT, 0U, (uint16_t)fault);
    motor_private_exit(impl, s);
    impl->tHfPlan.tIo.fnEmergencyStop(impl->tHfPlan.tIo.pContext);
    return result;
}

static bool motor_hf_candidate_qualified(
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

static foc_scalar_t motor_hf_blend_progress(uint16_t hwSample,
                                            uint16_t hwTotal)
{
    return foc_from_float((float)hwSample / (float)hwTotal);
}

static inline void motor_hf_post_event(motor_hf_state_t *state,
                                      motor_event_type_e type,
                                      motor_position_role_e role,
                                      uint16_t payload)
{
    if (state->chPendingCount < 4U) {
        state->aPendingEvents[state->chPendingCount++] = (motor_hf_pending_event_t){
            .hwPayload = payload,
            .chType = (uint8_t)type,
            .chRole = (uint8_t)role,
        };
    }
}

foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *impl;
    motor_hf_plan_t *plan;
    motor_hf_command_t *command;
    motor_hf_state_t *state;
    motor_hf_frame_t frame = {0};
    foc_result_t result;
    uintptr_t s;
    motor_transition_update_t transition = {0};
    foc_scalar_t angle_error = FOC_ZERO;
    foc_scalar_t blend_factor = FOC_ZERO;
#if FOC_HF_PROFILE
    uint32_t wTotalCycles = 0U, wEntryCycles = 0U, wSampleCurrentCycles = 0U;
    uint32_t wPositionCycles = 0U, wAlgoCycles = 0U, wCommitCycles = 0U;
    FOC_HF_PROFILE_TOTAL_BEGIN(tTotalStart);
#endif

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }

    impl = motor_private(ptMotor);
    plan = &impl->tHfPlan;
    command = &impl->tHfCommand;
    state = &impl->tHfState;

#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tEntryStart);
#endif
    s = motor_private_enter(impl);
    if (impl->bHighFrequencyStepInProgress) {
        motor_private_exit(impl, s);
        return FOC_RESULT_BUSY;
    }
    if ((impl->tRt.eRunState != MOTOR_STATE_STARTING &&
         impl->tRt.eRunState != MOTOR_STATE_RUNNING) ||
        impl->tRt.wFaults != MOTOR_FAULT_NONE ||
        !impl->bPwmEnabled) {
        motor_private_exit(impl, s);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    impl->bHighFrequencyStepInProgress = true;

    motor_control_mode_e mode = command->eMode;
    foc_dq_t voltage_ref = command->tVoltageReference;
    foc_dq_t current_ref = command->tCurrentReference;
    bool direct_source = impl->bInitialPositionSourceBound;
    bool candidate_source = impl->bTargetPositionSourceBound;
    bool transition_required = !direct_source && candidate_source;
    foc_angle_t angle = state->tElectricalAngle;
    foc_scalar_t speed = impl->qOpenLoopCommandSpeed;
    motor_startup_phase_e startup_phase = impl->chStartupPhase;
    uint16_t transition_samples = impl->hwTransitionSampleCount;
    foc_angle_t transition_start_angle = impl->tTransitionStartAngle;
    foc_scalar_t transition_start_speed = impl->qTransitionStartSpeed;
    foc_scalar_t high_frequency_period = impl->qHighFrequencyPeriod;
    foc_position_config_t position_config = impl->tPositionConfig;
    foc_position_qualification_t qualification = (foc_position_qualification_t){
        .eRequiredValid = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                          FOC_POSITION_VALID_ELECTRICAL_SPEED,
        .qMinimumConfidence = impl->qTransitionMinimumConfidence,
        .qMinimumSpeed = impl->qTransitionMinimumSpeed,
        .qMaximumAngleError = impl->qTransitionMaximumAngleError,
        .wMaximumAge = 0U,
    };
    uint16_t qualification_samples = impl->hwTransitionQualificationSamples;
    uint16_t blend_samples = impl->hwTransitionBlendSamples;
    uint32_t position_timestamp = ++impl->wPositionSampleTimestamp;
    if (position_timestamp == 0U) {
        position_timestamp = ++impl->wPositionSampleTimestamp;
    }
    motor_private_exit(impl, s);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tEntryStart, wEntryCycles);
#endif

    /* 1. 批量采样相电流 */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tSampleStart);
#endif
    result = plan->tIo.fnSampleCurrent(plan->tIo.pContext,
                                       &state->tPhaseCurrent);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tSampleStart, wSampleCurrentCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_HARDWARE, result);
    }

    /* 2. Clarke 变换与位置源步进 */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tPosStart);
#endif
    result = foc_clarke(state->tPhaseCurrent.qIu,
                        state->tPhaseCurrent.qIv,
                        state->tPhaseCurrent.qIw,
                        &frame.tCurrentAlphaBeta);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    frame.tPositionInput = (foc_position_input_t){
        .tCurrent = frame.tCurrentAlphaBeta,
        .tVoltage = state->tVoltageAlphaBeta,
        .qSamplePeriod = high_frequency_period,
        .wTimestamp = position_timestamp,
    };

    if (transition_required) {
        foc_position_output_t open_loop_output = {0};
        foc_position_source_if_t open_loop_if =
            foc_open_loop_source_GetInterface(&impl->tDefaultOpenLoopSource);
        result = open_loop_if.fnStep(open_loop_if.pContext,
                                     &frame.tPositionInput,
                                     &open_loop_output);
        if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
        }
        angle = open_loop_output.tElectricalAngle;
        speed = open_loop_output.qElectricalSpeed;

        result = plan->fnSourceStep(plan->pSourceContext,
                                    &frame.tPositionInput,
                                    &frame.tPositionOutput);
        if (result != FOC_RESULT_OK || frame.tPositionOutput.wFaults != 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  result != FOC_RESULT_OK ?
                                  result : FOC_RESULT_INVALID_ARGUMENT);
        }
        if ((frame.tPositionOutput.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            result = foc_position_ApplyMechanicalConfig(
                &position_config, &frame.tPositionOutput);
            if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
                FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
                return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
            }
        }
    } else {
        result = plan->fnSourceStep(plan->pSourceContext,
                                    &frame.tPositionInput,
                                    &frame.tPositionOutput);
        if (result != FOC_RESULT_OK || frame.tPositionOutput.wFaults != 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  result != FOC_RESULT_OK ?
                                  result : FOC_RESULT_INVALID_ARGUMENT);
        }
        if ((frame.tPositionOutput.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            result = foc_position_ApplyMechanicalConfig(
                &position_config, &frame.tPositionOutput);
            if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
                FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
                return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
            }
        }
        if ((frame.tPositionOutput.eValidFlags &
             FOC_POSITION_VALID_ELECTRICAL_ANGLE) == 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  FOC_RESULT_INVALID_ARGUMENT);
        }
        angle = frame.tPositionOutput.tElectricalAngle;
        if ((frame.tPositionOutput.eValidFlags &
             FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U) {
            speed = frame.tPositionOutput.qElectricalSpeed;
        }
    }

    /* 源切换管理：资格判定与混合过渡 */
    if (transition_required &&
        startup_phase == MOTOR_STARTUP_QUALIFY_SOURCE) {
        if (motor_hf_candidate_qualified(
                &frame.tPositionOutput, &qualification, angle, speed,
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

        if (!motor_hf_candidate_qualified(
                &frame.tPositionOutput, &qualification, transition_start_angle,
                transition_start_speed, position_timestamp)) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  FOC_RESULT_INVALID_ARGUMENT);
        }
        transition_samples++;
        angle_error = foc_angle_diff(frame.tPositionOutput.tElectricalAngle,
                                     transition_start_angle);
        blend_factor = motor_hf_blend_progress(transition_samples,
                                              blend_samples);
        result = foc_position_Blend(&from, &frame.tPositionOutput,
                                    blend_factor, &blended);
        if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
        }
        angle = blended.tElectricalAngle;
        speed = blended.qElectricalSpeed;
        transition.ePhase = transition_samples >= blend_samples ?
            MOTOR_STARTUP_COMPLETE : MOTOR_STARTUP_BLEND_ANGLE;
        transition.tStartAngle = transition_start_angle;
        transition.qStartSpeed = transition_start_speed;
        transition.hwSampleCount = transition_samples;
        transition.bChanged = true;
        if (transition.ePhase == MOTOR_STARTUP_COMPLETE &&
            mode >= MOTOR_CONTROL_SPEED) {
            foc_scalar_t speed_reference = command->qSpeedReference;
            if (mode >= MOTOR_CONTROL_POSITION) {
                speed_reference = frame.tPositionOutput.qMechanicalSpeed;
                impl->tControlConfig.tPositionController.fnTrack(
                    impl->tControlConfig.tPositionController.pContext,
                    speed_reference, command->qPositionReference,
                    foc_angle_to_turns(frame.tPositionOutput.tMechanicalAngle));
            }
            impl->tControlConfig.tSpeedController.fnTrack(
                impl->tControlConfig.tSpeedController.pContext,
                current_ref.qQ, speed_reference,
                frame.tPositionOutput.qMechanicalSpeed);
        }
    }
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif

    /* 3. 算法核心 (Park -> PI -> IPark -> Modulate) */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tAlgoStart);
#endif
    foc_scalar_t sin_theta, cos_theta;
    foc_angle_sincos(angle, &sin_theta, &cos_theta);
    result = foc_park_cached(&frame.tCurrentAlphaBeta, sin_theta, cos_theta,
                             &frame.tCurrent);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    if (mode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP) {
        frame.tVoltage = voltage_ref;
    } else {
        frame.tVoltage.qD = plan->tId.fnStep(plan->tId.pContext,
                                             current_ref.qD,
                                             frame.tCurrent.qD);
        frame.tVoltage.qQ = plan->tIq.fnStep(plan->tIq.pContext,
                                             current_ref.qQ,
                                             frame.tCurrent.qQ);
    }

    result = foc_ipark_cached(&frame.tVoltage, sin_theta, cos_theta,
                              &frame.tVoltageAlphaBeta);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    result = plan->fnModulate(&frame.tVoltageAlphaBeta, &frame.tDuty);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    /* 检查是否有待处理的停止命令或离线状态 */
    s = motor_private_enter(impl);
    bool stopping = impl->bCommandPending &&
                    impl->chPendingCommand == MOTOR_COMMAND_STOP;
    if (stopping ||
        (impl->tRt.eRunState != MOTOR_STATE_STARTING &&
         impl->tRt.eRunState != MOTOR_STATE_RUNNING)) {
        impl->bHighFrequencyStepInProgress = false;
        motor_private_exit(impl, s);
        return FOC_RESULT_BUSY;
    }
    motor_private_exit(impl, s);

    /* 4. 提交三相占空比 */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tCommitStart);
#endif
    result = plan->tIo.fnCommitDuty(plan->tIo.pContext, &frame.tDuty);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tCommitStart, wCommitCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_HARDWARE, result);
    }

    /* 5. 状态与事件发布（低开销路径：事件入延迟槽） */
    s = motor_private_enter(impl);
    state->tElectricalAngle = angle;
    state->qElectricalSpeed = speed;
    impl->qOpenLoopCommandSpeed = speed;
    impl->tCandidateAngle = frame.tPositionOutput.tElectricalAngle;
    impl->qCandidateSpeed = frame.tPositionOutput.qElectricalSpeed;
    impl->qAngleError = angle_error;
    impl->qBlendFactor = blend_factor;
    uint8_t active_valid = direct_source ?
        (uint8_t)frame.tPositionOutput.eValidFlags :
        (uint8_t)(FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                  FOC_POSITION_VALID_ELECTRICAL_SPEED);
    uint8_t candidate_valid = candidate_source ?
        (uint8_t)frame.tPositionOutput.eValidFlags : 0U;

    if (active_valid != impl->chActiveValidFlags) {
        motor_hf_post_event(state, MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                            MOTOR_POSITION_ROLE_ACTIVE,
                            (uint16_t)((uint16_t)impl->chActiveValidFlags |
                                       ((uint16_t)active_valid << 8)));
        impl->chActiveValidFlags = active_valid;
    }
    if (candidate_valid != impl->chCandidateValidFlags) {
        motor_hf_post_event(state, MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                            MOTOR_POSITION_ROLE_CANDIDATE,
                            (uint16_t)((uint16_t)impl->chCandidateValidFlags |
                                       ((uint16_t)candidate_valid << 8)));
        impl->chCandidateValidFlags = candidate_valid;
    }
    impl->tMechanicalAngle = frame.tPositionOutput.tMechanicalAngle;
    impl->qMechanicalSpeed = frame.tPositionOutput.qMechanicalSpeed;
    impl->chMechanicalValidFlags =
        (uint8_t)(frame.tPositionOutput.eValidFlags &
         (FOC_POSITION_VALID_MECHANICAL_ANGLE |
          FOC_POSITION_VALID_MECHANICAL_SPEED));
    if (transition.bChanged) {
        motor_startup_phase_e previous_phase = impl->chStartupPhase;
        impl->chStartupPhase = transition.ePhase;
        impl->tTransitionStartAngle = transition.tStartAngle;
        impl->qTransitionStartSpeed = transition.qStartSpeed;
        impl->hwTransitionSampleCount = transition.hwSampleCount;
        if (previous_phase != transition.ePhase &&
            transition.ePhase == MOTOR_STARTUP_BLEND_ANGLE) {
            motor_hf_post_event(state, MOTOR_EVENT_TRANSITION_STARTED, 0U,
                                (uint16_t)((uint16_t)previous_phase |
                                           ((uint16_t)transition.ePhase << 8)));
        } else if (previous_phase != transition.ePhase &&
                   transition.ePhase == MOTOR_STARTUP_COMPLETE) {
            motor_hf_post_event(state, MOTOR_EVENT_TRANSITION_COMPLETED, 0U,
                                (uint16_t)((uint16_t)previous_phase |
                                           ((uint16_t)transition.ePhase << 8)));
        }
    }
    state->tCurrent = frame.tCurrent;
    state->tVoltage = frame.tVoltage;
    state->tVoltageAlphaBeta = frame.tVoltageAlphaBeta;
    state->tDuty = frame.tDuty;
    state->tPositionOutput = frame.tPositionOutput;
    impl->bHighFrequencyStepInProgress = false;

#if FOC_HF_PROFILE
    FOC_HF_PROFILE_TOTAL_END(tTotalStart, wTotalCycles);
    impl->tProfileSnapshot = (motor_hf_profile_snapshot_t){
        .wSampleSequence = position_timestamp,
        .wTotalCycles = wTotalCycles,
        .wSampleCurrentCycles = wSampleCurrentCycles,
        .wPositionCycles = wPositionCycles,
        .wClarkeCycles = wAlgoCycles,
        .wCommitCycles = wCommitCycles,
        .wEntryCycles = wEntryCycles,
        .wValidFlags = MOTOR_HF_PROFILE_VALID_TOTAL |
                       MOTOR_HF_PROFILE_VALID_SAMPLE_CURRENT |
                       MOTOR_HF_PROFILE_VALID_POSITION |
                       MOTOR_HF_PROFILE_VALID_CLARKE |
                       MOTOR_HF_PROFILE_VALID_COMMIT |
                       MOTOR_HF_PROFILE_VALID_ENTRY,
        .eResult = FOC_RESULT_OK,
    };
#endif
    motor_private_exit(impl, s);

    return FOC_RESULT_OK;
}
