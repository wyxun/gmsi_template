/*******************************************************************************
 * @file    motor.c
 * @brief   Multi-instance motor aggregation object
 ******************************************************************************/

#include "motor.h"
#include "motor_private.h"
#include "foc_hf_profile.h"

#include <stddef.h>
#include <string.h>

#define MOTOR_DEFAULT_TRANSITION_QUALIFICATION_SAMPLES 3U
#define MOTOR_DEFAULT_TRANSITION_BLEND_SAMPLES 8U
#define MOTOR_DEFAULT_TRANSITION_TIMEOUT_MS 1000U
#define MOTOR_DEFAULT_TRANSITION_MAXIMUM_ANGLE_ERROR FOC_SCALAR(0.25f)

#if defined(MOTOR_ENABLE_TEST_HOOKS)
size_t motor_TestGetImplementationSize(void)
{
    return sizeof(motor_impl_t);
}

void motor_TestSetOpenLoopCommandSpeed(motor_handle_t *ptMotor,
                                       foc_scalar_t qSpeed)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_private(ptMotor)->qOpenLoopCommandSpeed = qSpeed;
        motor_private(ptMotor)->tDefaultOpenLoopSource.qSpeed = qSpeed;
        motor_private(ptMotor)->tHfState.qElectricalSpeed = qSpeed;
    }
}
#endif

foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig)
{
    foc_result_t eResult;
    motor_impl_t *ptImpl;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->tParams.chPolePairs == 0U ||
        ptConfig->eTopology < SENSING_TOPOLOGY_1P ||
        ptConfig->eTopology > SENSING_TOPOLOGY_3P ||
        ptConfig->qHighFrequencyPeriod <= FOC_ZERO ||
        ptConfig->qLowFrequencyPeriod <= FOC_ZERO ||
        ptConfig->tPosition.chPolePairs == 0U ||
        (ptConfig->tPosition.chDirection != 1 &&
         ptConfig->tPosition.chDirection != -1) ||
        ptConfig->qTransitionMinimumConfidence < FOC_ZERO ||
        ptConfig->qTransitionMinimumConfidence > FOC_ONE ||
        ptConfig->qTransitionMinimumSpeed < FOC_ZERO ||
        ptConfig->qTransitionMaximumAngleError < FOC_ZERO ||
        ptConfig->qTransitionMaximumAngleError > FOC_HALF) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptConfig->wStartupDelayMs != 0U &&
        ptConfig->tTime.fnGetMilliseconds == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptConfig->tSync.fnEnter == NULL) !=
        (ptConfig->tSync.fnExit == NULL)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_hal_Validate(&ptConfig->tHal);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_hal_ValidateHighFrequency(&ptConfig->tHal);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }

#if FOC_HF_PROFILE
    /* Enable the DWT cycle counter once so per-step profiling reads cost a
     * single register load instead of a get_system_ticks() call. */
    foc_hf_profile_InitCycles();
#endif

    memset(ptMotor, 0, sizeof(*ptMotor));
    ptImpl = motor_private(ptMotor);
    ptImpl->tHal = ptConfig->tHal;
    /* Fast Path 接口在 Init 期确定；plan 的其余字段保持零初始化，
       由启动期 plan resolver 填充。tHfCommand 由上面的 memset 清零。 */
    ptImpl->tHfPlan.tIo = ptConfig->tHal.tHfIo;

    /* Id Controller binding */
    if (foc_controller_IsValid(&ptConfig->tControl.tId)) {
        ptImpl->tControlConfig.tId = ptConfig->tControl.tId;
    } else if (ptConfig->tControl.tIdParams.qOutputMinimum != FOC_ZERO ||
               ptConfig->tControl.tIdParams.qOutputMaximum != FOC_ZERO) {
        eResult = foc_pid_Init(&ptImpl->tIdPid, &ptConfig->tControl.tIdParams);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ptImpl->tControlConfig.tId = foc_controller_FromPid(&ptImpl->tIdPid);
    }

    /* Iq Controller binding */
    if (foc_controller_IsValid(&ptConfig->tControl.tIq)) {
        ptImpl->tControlConfig.tIq = ptConfig->tControl.tIq;
    } else if (ptConfig->tControl.tIqParams.qOutputMinimum != FOC_ZERO ||
               ptConfig->tControl.tIqParams.qOutputMaximum != FOC_ZERO) {
        eResult = foc_pid_Init(&ptImpl->tIqPid, &ptConfig->tControl.tIqParams);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ptImpl->tControlConfig.tIq = foc_controller_FromPid(&ptImpl->tIqPid);
    }

    /* Speed Controller binding */
    if (foc_controller_IsValid(&ptConfig->tControl.tSpeed)) {
        ptImpl->tControlConfig.tSpeed = ptConfig->tControl.tSpeed;
    } else if (ptConfig->tControl.tSpeedParams.qOutputMinimum != FOC_ZERO ||
               ptConfig->tControl.tSpeedParams.qOutputMaximum != FOC_ZERO) {
        eResult = foc_pid_Init(&ptImpl->tSpeedPid, &ptConfig->tControl.tSpeedParams);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ptImpl->tControlConfig.tSpeed = foc_controller_FromPid(&ptImpl->tSpeedPid);
    }

    /* Position Controller binding */
    if (foc_controller_IsValid(&ptConfig->tControl.tPosition)) {
        ptImpl->tControlConfig.tPosition = ptConfig->tControl.tPosition;
    } else if (ptConfig->tControl.tPositionParams.qOutputMinimum != FOC_ZERO ||
               ptConfig->tControl.tPositionParams.qOutputMaximum != FOC_ZERO) {
        eResult = foc_pid_Init(&ptImpl->tPositionPid, &ptConfig->tControl.tPositionParams);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ptImpl->tControlConfig.tPosition = foc_controller_FromPid(&ptImpl->tPositionPid);
    }

    ptImpl->tControlConfig.eModulation =
        ptConfig->tControl.eModulation;
    ptImpl->tHfState.tPhaseCurrent.eTopology = ptConfig->eTopology;
    ptImpl->tTime = ptConfig->tTime;
    ptImpl->tSync = ptConfig->tSync;
    ptImpl->qHighFrequencyPeriod = ptConfig->qHighFrequencyPeriod;
    ptImpl->qTransitionMinimumConfidence =
        ptConfig->qTransitionMinimumConfidence;
    ptImpl->qTransitionMinimumSpeed =
        ptConfig->qTransitionMinimumSpeed;
    ptImpl->qTransitionMaximumAngleError =
        ptConfig->qTransitionMaximumAngleError != FOC_ZERO ?
        ptConfig->qTransitionMaximumAngleError :
        MOTOR_DEFAULT_TRANSITION_MAXIMUM_ANGLE_ERROR;
    ptImpl->tPositionConfig = ptConfig->tPosition;
    ptImpl->wStartupDelayMs = ptConfig->wStartupDelayMs;
    ptImpl->wTransitionTimeoutMs =
        ptConfig->wTransitionTimeoutMs != 0U ?
        ptConfig->wTransitionTimeoutMs :
        MOTOR_DEFAULT_TRANSITION_TIMEOUT_MS;
    ptImpl->hwTransitionQualificationSamples =
        ptConfig->hwTransitionQualificationSamples != 0U ?
        ptConfig->hwTransitionQualificationSamples :
        MOTOR_DEFAULT_TRANSITION_QUALIFICATION_SAMPLES;
    ptImpl->hwTransitionBlendSamples =
        ptConfig->hwTransitionBlendSamples != 0U ?
        ptConfig->hwTransitionBlendSamples :
        MOTOR_DEFAULT_TRANSITION_BLEND_SAMPLES;
    ptImpl->tHfState.tPhaseCurrent.tCalib.wOffsetU = 2048U;
    ptImpl->tHfState.tPhaseCurrent.tCalib.wOffsetV = 2048U;
    ptImpl->tHfState.tPhaseCurrent.tCalib.wOffsetW = 2048U;
    ptImpl->tRuntime.eRunState = MOTOR_STATE_IDLE;
    ptImpl->chStartupPhase = MOTOR_STARTUP_IDLE;
    ptImpl->wNextEventSequence = 1U;
    ptImpl->wMagic = MOTOR_IMPL_MAGIC;
    return FOC_RESULT_OK;
}

void motor_private_AppendEvent(motor_impl_t *ptImpl,
                               motor_event_type_e eType,
                               motor_state_e eFrom,
                               motor_state_e eTo,
                               uint8_t chDetail,
                               uint16_t hwPayload)
{
    uint8_t chIndex;
    motor_event_record_t *ptRecord;

    if (ptImpl->chEventCount == MOTOR_EVENT_CAPACITY) {
        ptImpl->chEventHead =
            (uint8_t)((ptImpl->chEventHead + 1U) % MOTOR_EVENT_CAPACITY);
        ptImpl->wEventOverwriteCount++;
    } else {
        ptImpl->chEventCount++;
    }
    chIndex = (uint8_t)((ptImpl->chEventHead +
                         ptImpl->chEventCount - 1U) %
                        MOTOR_EVENT_CAPACITY);
    ptRecord = &ptImpl->atEvents[chIndex];
    ptRecord->wSequence = ptImpl->wNextEventSequence++;
    if (ptImpl->wNextEventSequence == 0U) {
        ptImpl->wNextEventSequence = 1U;
    }
    ptRecord->hwPayload = hwPayload;
    ptRecord->chType = (uint8_t)eType;
    ptRecord->chMeta = MOTOR_EVENT_META(eFrom, eTo, chDetail);
}

void motor_Reset(motor_handle_t *ptMotor)
{
    uintptr_t wSyncState;
    if (!motor_private_is_initialized(ptMotor)) {
        return;
    }
    motor_impl_t *ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    if (ptImpl->tRuntime.eRunState != MOTOR_STATE_IDLE ||
        ptImpl->bPwmEnabled || ptImpl->bCommandPending) {
        motor_private_exit(ptImpl, wSyncState);
        return;
    }
    memset(&ptImpl->tRuntime, 0, sizeof(ptImpl->tRuntime));
    ptImpl->tRuntime.eRunState = MOTOR_STATE_IDLE;
    ptImpl->chStartupPhase = MOTOR_STARTUP_IDLE;
    ptImpl->chPendingCommand = MOTOR_COMMAND_NONE;
    motor_private_exit(ptImpl, wSyncState);
}

foc_result_t motor_private_SetDuty(motor_handle_t *ptMotor,
                           q_type qDutyU,
                           q_type qDutyV,
                           q_type qDutyW)
{
    foc_result_t eResult;
    motor_impl_t *ptImpl;
    uintptr_t wSyncState;
    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    if (ptImpl->tRuntime.wFaults != MOTOR_FAULT_NONE ||
        ptImpl->tRuntime.eRunState != MOTOR_STATE_RUNNING ||
        !ptImpl->bPwmEnabled) {
        motor_private_exit(ptImpl, wSyncState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    /* Duty callback is a bounded register commit and must not block or
       re-enter motor APIs. Keeping it under the lock closes the final
       state/fault validation and hardware write transaction. */
    eResult = foc_hal_SetDuty(&ptImpl->tHal.tPwm,
                              qDutyU, qDutyV, qDutyW);
    motor_private_exit(ptImpl, wSyncState);
    return eResult;
}

foc_result_t motor_private_Enable(motor_handle_t *ptMotor, bool bEnable)
{
    foc_result_t eResult;
    motor_impl_t *ptImpl;
    uintptr_t wSyncState;
    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    if (bEnable && ptImpl->tRuntime.wFaults != MOTOR_FAULT_NONE) {
        motor_private_exit(ptImpl, wSyncState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    /* Enable is a bounded atomic HAL commit, like a duty update. */
    eResult = foc_hal_Enable(&ptImpl->tHal.tPwm, bEnable);
    if (eResult != FOC_RESULT_OK) {
        motor_private_exit(ptImpl, wSyncState);
        return eResult;
    }
    ptImpl->bPwmEnabled = bEnable;
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}

foc_result_t motor_private_CalibrateCurrent(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    return foc_hal_CurrentCalibrate(&ptImpl->tHal.tAdc,
                                    &ptImpl->tHfState.tPhaseCurrent.tCalib);
}

foc_result_t motor_GetRawCurrent(motor_handle_t *ptMotor,
                                 uint32_t *pwRawU,
                                 uint32_t *pwRawV,
                                 uint32_t *pwRawW)
{
    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    return foc_hal_CurrentGetRaw(&motor_private(ptMotor)->tHal.tAdc,
                                 pwRawU, pwRawV, pwRawW);
}

foc_result_t motor_private_SampleCurrent(motor_handle_t *ptMotor)
{
    foc_result_t eResult;
    motor_impl_t *ptImpl;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    eResult = foc_hal_CurrentReconstruct(&ptImpl->tHal.tAdc,
                                         &ptImpl->tHfState.tPhaseCurrent);
    if (eResult != FOC_RESULT_OK) {
        motor_EmergencyStop(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
    }
    return eResult;
}

void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_e eFault)
{
    uintptr_t wSyncState;
    motor_state_e eFromState;
    if (!motor_private_is_initialized(ptMotor)) {
        return;
    }
    motor_impl_t *ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    eFromState = ptImpl->tRuntime.eRunState;
    ptImpl->tRuntime.wFaults |= (uint32_t)eFault;
    ptImpl->tRuntime.eRunState = MOTOR_STATE_FAULT;
    ptImpl->bPwmEnabled = false;
    ptImpl->bCommandPending = false;
    ptImpl->chPendingCommand = MOTOR_COMMAND_NONE;
    ptImpl->bDiagnosticActive = false;
    motor_private_AppendEvent(ptImpl, MOTOR_EVENT_FAULT,
                              eFromState, MOTOR_STATE_FAULT, 0U,
                              (uint16_t)ptImpl->tRuntime.wFaults);
    motor_private_exit(ptImpl, wSyncState);
    foc_hal_EmergencyStop(&ptImpl->tHal.tPwm);
}

foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot)
{
    const motor_impl_t *ptImpl;

    if (ptMotor == NULL || ptSnapshot == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_private_is_initialized(ptMotor)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private_const(ptMotor);
    uintptr_t wSyncState = motor_private_enter(ptImpl);
    *ptSnapshot = (motor_snapshot_t){
        .eRunState = ptImpl->tRuntime.eRunState,
        .wFaults = ptImpl->tRuntime.wFaults,
        .wEventSequence = ptImpl->wNextEventSequence - 1U,
        .wEventOverwriteCount = ptImpl->wEventOverwriteCount,
        .tPhaseCurrent = {ptImpl->tHfState.tPhaseCurrent.qIu,
                          ptImpl->tHfState.tPhaseCurrent.qIv,
                          ptImpl->tHfState.tPhaseCurrent.qIw},
        .tCurrentReference = ptImpl->tHfCommand.tCurrentReference,
        .tCurrent = ptImpl->tHfState.tCurrent,
        .tVoltageReference = ptImpl->tHfCommand.tVoltageReference,
        .tVoltage = ptImpl->tHfState.tVoltage,
        .tDuty = ptImpl->tHfState.tDuty,
        .qSpeedReference = ptImpl->tHfCommand.qSpeedReference,
        .qPositionReference = ptImpl->tHfCommand.qPositionReference,
        .tOpenLoopAngle = ptImpl->tDefaultOpenLoopSource.tAngle,
        .tActiveAngle = ptImpl->tHfState.tElectricalAngle,
        .tCandidateAngle = ptImpl->tCandidateAngle,
        .qActiveSpeed = ptImpl->tHfState.qElectricalSpeed,
        .qCandidateSpeed = ptImpl->qCandidateSpeed,
        .qAngleError = ptImpl->qAngleError,
        .qBlendFactor = ptImpl->qBlendFactor,
        .tElectricalAngle = ptImpl->tHfState.tElectricalAngle,
        .qElectricalSpeed = ptImpl->tHfState.qElectricalSpeed,
        .qVbus = ptImpl->tRuntime.qVbus,
        .tCurrentCalibration = ptImpl->tHfState.tPhaseCurrent.tCalib,
        .eControlMode = ptImpl->tHfCommand.eMode,
        .eActiveSourceValidFlags =
            (foc_position_valid_flag_e)ptImpl->chActiveValidFlags,
        .eCandidateSourceValidFlags =
            (foc_position_valid_flag_e)ptImpl->chCandidateValidFlags,
        .bPwmEnabled = ptImpl->bPwmEnabled,
        .eStartupPhase = (motor_startup_phase_e)ptImpl->chStartupPhase,
        .ePendingCommand = ptImpl->bCommandPending ?
                           (motor_command_e)ptImpl->chPendingCommand :
                           MOTOR_COMMAND_NONE,
    };
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetStatus(const motor_handle_t *ptMotor,
                             motor_state_e *peState,
                             uint32_t *pwFaults)
{
    const motor_impl_t *ptImpl;
    uintptr_t wSyncState;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    if (peState == NULL || pwFaults == NULL) {
        return FOC_RESULT_NULL;
    }
    ptImpl = motor_private_const(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    *peState = ptImpl->tRuntime.eRunState;
    *pwFaults = ptImpl->tRuntime.wFaults;
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetTelemetry(const motor_handle_t *ptMotor,
                                motor_telemetry_t *ptTelemetry)
{
    const motor_impl_t *ptImpl;
    uintptr_t wSyncState;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptTelemetry == NULL) {
        return FOC_RESULT_NULL;
    }
    ptImpl = motor_private_const(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    ptTelemetry->tCurrent = ptImpl->tHfState.tCurrent;
    ptTelemetry->tPhaseCurrent =
        (motor_phase_current_t){
            ptImpl->tHfState.tPhaseCurrent.qIu,
            ptImpl->tHfState.tPhaseCurrent.qIv,
            ptImpl->tHfState.tPhaseCurrent.qIw,
        };
    ptTelemetry->tActiveAngle = ptImpl->tHfState.tElectricalAngle;
    ptTelemetry->qActiveSpeed = ptImpl->tHfState.qElectricalSpeed;
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetCurrentCalibration(const motor_handle_t *ptMotor,
                                         foc_adc_calib_t *ptCalib)
{
    const motor_impl_t *ptImpl;
    uintptr_t wSyncState;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptCalib == NULL) {
        return FOC_RESULT_NULL;
    }
    ptImpl = motor_private_const(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    *ptCalib = ptImpl->tHfState.tPhaseCurrent.tCalib;
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetHighFrequencyProfileSnapshot(
    const motor_handle_t *ptMotor,
    motor_hf_profile_snapshot_t *ptSnapshot)
{
    if (ptMotor == NULL || ptSnapshot == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_private_is_initialized(ptMotor)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if FOC_HF_PROFILE
    const motor_impl_t *ptImpl = motor_private_const(ptMotor);
    uintptr_t wSyncState = motor_private_enter(ptImpl);
    *ptSnapshot = ptImpl->tProfileSnapshot;
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
#else
    (void)ptMotor;
    (void)ptSnapshot;
    return FOC_RESULT_DISABLED;
#endif
}

bool motor_DebugReadEvent(motor_handle_t *ptMotor,
                          motor_event_t *ptEvent)
{
    motor_impl_t *ptImpl;
    motor_event_record_t tRecord;
    uintptr_t wSyncState;

    if (!motor_private_is_initialized(ptMotor) || ptEvent == NULL) {
        return false;
    }
    ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    motor_private_DrainPendingEvents(ptImpl);
    if (ptImpl->chEventCount == 0U) {
        motor_private_exit(ptImpl, wSyncState);
        return false;
    }
    tRecord = ptImpl->atEvents[ptImpl->chEventHead];
    ptImpl->chEventHead =
        (uint8_t)((ptImpl->chEventHead + 1U) % MOTOR_EVENT_CAPACITY);
    ptImpl->chEventCount--;
    motor_private_exit(ptImpl, wSyncState);

    *ptEvent = (motor_event_t){
        .wSequence = tRecord.wSequence,
        .eType = (motor_event_type_e)tRecord.chType,
        .eFromState = (motor_state_e)MOTOR_EVENT_META_FROM(tRecord.chMeta),
        .eToState = (motor_state_e)MOTOR_EVENT_META_TO(tRecord.chMeta),
        .eResult = FOC_RESULT_OK,
    };
    if (ptEvent->eType == MOTOR_EVENT_FAULT) {
        ptEvent->wFaults = tRecord.hwPayload;
    } else if (ptEvent->eType == MOTOR_EVENT_COMMAND_ACCEPTED ||
               ptEvent->eType == MOTOR_EVENT_COMMAND_REJECTED) {
        ptEvent->eCommand =
            (motor_command_e)(tRecord.hwPayload & 0xFFU);
        ptEvent->eResult =
            (foc_result_t)((tRecord.hwPayload >> 8) & 0xFFU);
    } else if (ptEvent->eType ==
               MOTOR_EVENT_SOURCE_VALIDITY_CHANGED) {
        ptEvent->ePositionRole =
            (motor_position_role_e)MOTOR_EVENT_META_DETAIL(tRecord.chMeta);
        ptEvent->wPreviousValue = tRecord.hwPayload & 0xFFU;
        ptEvent->wCurrentValue = tRecord.hwPayload >> 8;
    } else if (ptEvent->eType >= MOTOR_EVENT_TRANSITION_STARTED &&
               ptEvent->eType <= MOTOR_EVENT_TRANSITION_TIMEOUT) {
        ptEvent->wPreviousValue = tRecord.hwPayload & 0xFFU;
        ptEvent->wCurrentValue = tRecord.hwPayload >> 8;
    }
    return true;
}

#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC
/*
 * Hardware bring-up diagnostic output. Not part of production builds.
 * A fixed duty is applied directly through the HAL while the lifecycle
 * FSM stays in IDLE; every call re-validates no-fault/IDLE/duty-limit
 * and the cumulative output duration never exceeds
 * MOTOR_DIAGNOSTIC_TIMEOUT_MS.
 */
#define MOTOR_DIAGNOSTIC_DUTY_LIMIT  FOC_SCALAR(0.1f)
#define MOTOR_DIAGNOSTIC_TIMEOUT_MS  2000U

foc_result_t motor_DiagnosticSetOutput(motor_handle_t *ptMotor,
                                       foc_scalar_t qDutyU,
                                       foc_scalar_t qDutyV,
                                       foc_scalar_t qDutyW)
{
    motor_impl_t *ptImpl;
    uintptr_t wSyncState;
    foc_result_t eResult;
    uint32_t wNow = 0U;
    bool bFirstActivation;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    if (foc_abs(qDutyU) > MOTOR_DIAGNOSTIC_DUTY_LIMIT ||
        foc_abs(qDutyV) > MOTOR_DIAGNOSTIC_DUTY_LIMIT ||
        foc_abs(qDutyW) > MOTOR_DIAGNOSTIC_DUTY_LIMIT) {
        return FOC_RESULT_SAFETY;
    }
    ptImpl = motor_private(ptMotor);
    if (ptImpl->tTime.fnGetMilliseconds != NULL) {
        wNow = ptImpl->tTime.fnGetMilliseconds(ptImpl->tTime.pTimeContext);
    }
    wSyncState = motor_private_enter(ptImpl);
    if (ptImpl->tRt.wFaults != MOTOR_FAULT_NONE ||
        ptImpl->tRt.eRunState != MOTOR_STATE_IDLE ||
        ptImpl->bCommandPending) {
        motor_private_exit(ptImpl, wSyncState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    bFirstActivation = !ptImpl->bDiagnosticActive;
    if (bFirstActivation) {
        ptImpl->wDiagnosticStartMs = wNow;
    } else if (ptImpl->tTime.fnGetMilliseconds != NULL &&
               (uint32_t)(wNow - ptImpl->wDiagnosticStartMs) >
                   MOTOR_DIAGNOSTIC_TIMEOUT_MS) {
        ptImpl->bDiagnosticActive = false;
        ptImpl->bPwmEnabled = false;
        motor_private_exit(ptImpl, wSyncState);
        (void)foc_hal_Enable(&ptImpl->tHal.tPwm, false);
        return FOC_RESULT_SAFETY;
    }
    ptImpl->bDiagnosticActive = true;
    motor_private_exit(ptImpl, wSyncState);
    if (bFirstActivation) {
        eResult = foc_hal_Enable(&ptImpl->tHal.tPwm, true);
        wSyncState = motor_private_enter(ptImpl);
        if (eResult != FOC_RESULT_OK) {
            ptImpl->bDiagnosticActive = false;
            motor_private_exit(ptImpl, wSyncState);
            motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);
            return eResult;
        }
        ptImpl->bPwmEnabled = true;
        motor_private_exit(ptImpl, wSyncState);
    }
    eResult = foc_hal_SetDuty(&ptImpl->tHal.tPwm,
                              qDutyU, qDutyV, qDutyW);
    if (eResult != FOC_RESULT_OK) {
        motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);
    }
    return eResult;
}

foc_result_t motor_DiagnosticStopOutput(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    uintptr_t wSyncState;
    foc_result_t eResult;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    if (!ptImpl->bDiagnosticActive) {
        motor_private_exit(ptImpl, wSyncState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl->bDiagnosticActive = false;
    ptImpl->bPwmEnabled = false;
    motor_private_exit(ptImpl, wSyncState);
    eResult = foc_hal_Enable(&ptImpl->tHal.tPwm, false);
    if (eResult != FOC_RESULT_OK) {
        motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);
    }
    return eResult;
}
#endif /* FOC_ENABLE_DIAGNOSTIC */
