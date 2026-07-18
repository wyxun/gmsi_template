/*******************************************************************************
 * @file    motor.c
 * @brief   Multi-instance motor aggregation object
 ******************************************************************************/

#include "motor.h"
#include "motor_private.h"

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
        motor_private(ptMotor)->tRt.qOmegaE = qSpeed;
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

    memset(ptMotor, 0, sizeof(*ptMotor));
    ptImpl = motor_private(ptMotor);
    ptImpl->tHal = ptConfig->tHal;
    ptImpl->tControl.tConfig.tIdController.pContext =
        ptConfig->tControl.tIdController.pContext;
    ptImpl->tControl.tConfig.tIdController.fnStep =
        ptConfig->tControl.tIdController.fnStep;
    ptImpl->tControl.tConfig.tIqController.pContext =
        ptConfig->tControl.tIqController.pContext;
    ptImpl->tControl.tConfig.tIqController.fnStep =
        ptConfig->tControl.tIqController.fnStep;
    ptImpl->tControl.tConfig.tSpeedController.pContext =
        ptConfig->tControl.tSpeedController.pContext;
    ptImpl->tControl.tConfig.tSpeedController.fnStep =
        ptConfig->tControl.tSpeedController.fnStep;
    ptImpl->tControl.tConfig.tSpeedController.fnTrack =
        ptConfig->tControl.tSpeedController.fnTrack;
    ptImpl->tControl.tConfig.tPositionController.pContext =
        ptConfig->tControl.tPositionController.pContext;
    ptImpl->tControl.tConfig.tPositionController.fnStep =
        ptConfig->tControl.tPositionController.fnStep;
    ptImpl->tControl.tConfig.tPositionController.fnTrack =
        ptConfig->tControl.tPositionController.fnTrack;
    ptImpl->tControl.tConfig.eModulation =
        ptConfig->tControl.eModulation;
    ptImpl->tCurrent.eTopology = ptConfig->eTopology;
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
    ptImpl->tCurrent.tCalib.wOffsetU = 2048U;
    ptImpl->tCurrent.tCalib.wOffsetV = 2048U;
    ptImpl->tCurrent.tCalib.wOffsetW = 2048U;
    ptImpl->tRt.eRunState = MOTOR_STATE_IDLE;
    ptImpl->eStartupPhase = MOTOR_STARTUP_IDLE;
    ptImpl->wNextEventSequence = 1U;
    ptImpl->wMagic = MOTOR_IMPL_MAGIC;
    return FOC_RESULT_OK;
}

void motor_private_AppendEvent(motor_impl_t *ptImpl,
                               motor_event_type_e eType,
                               motor_state_e eFrom,
                               motor_state_e eTo,
                               uint8_t chDetail,
                               uint32_t wPayload)
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
    ptRecord->wPayload = wPayload;
    ptRecord->chType = (uint8_t)eType;
    ptRecord->chFrom = (uint8_t)eFrom;
    ptRecord->chTo = (uint8_t)eTo;
    ptRecord->chDetail = chDetail;
}

void motor_Reset(motor_handle_t *ptMotor)
{
    uintptr_t wSyncState;
    if (!motor_private_is_initialized(ptMotor)) {
        return;
    }
    motor_impl_t *ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_IDLE ||
        ptImpl->bPwmEnabled || ptImpl->bCommandPending) {
        motor_private_exit(ptImpl, wSyncState);
        return;
    }
    memset(&ptImpl->tRt, 0, sizeof(ptImpl->tRt));
    ptImpl->tRt.eRunState = MOTOR_STATE_IDLE;
    ptImpl->eStartupPhase = MOTOR_STARTUP_IDLE;
    ptImpl->ePendingCommand = MOTOR_COMMAND_NONE;
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
    if (ptImpl->tRt.wFaults != MOTOR_FAULT_NONE ||
        ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING ||
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
    if (bEnable && ptImpl->tRt.wFaults != MOTOR_FAULT_NONE) {
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
    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    return foc_hal_CurrentCalibrate(&motor_private(ptMotor)->tHal.tAdc,
                                    &motor_private(ptMotor)->tCurrent.tCalib);
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

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_hal_CurrentReconstruct(&motor_private(ptMotor)->tHal.tAdc,
                                         &motor_private(ptMotor)->tCurrent);
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
    eFromState = ptImpl->tRt.eRunState;
    ptImpl->tRt.wFaults |= (uint32_t)eFault;
    ptImpl->tRt.eRunState = MOTOR_STATE_FAULT;
    ptImpl->bPwmEnabled = false;
    ptImpl->bCommandPending = false;
    ptImpl->ePendingCommand = MOTOR_COMMAND_NONE;
    motor_private_AppendEvent(ptImpl, MOTOR_EVENT_FAULT,
                              eFromState, MOTOR_STATE_FAULT, 0U,
                              ptImpl->tRt.wFaults);
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
        .eRunState = ptImpl->tRt.eRunState,
        .wFaults = ptImpl->tRt.wFaults,
        .wEventSequence = ptImpl->wNextEventSequence - 1U,
        .wEventOverwriteCount = ptImpl->wEventOverwriteCount,
        .tPhaseCurrent = {ptImpl->tCurrent.qIu, ptImpl->tCurrent.qIv,
                          ptImpl->tCurrent.qIw},
        .tCurrentReference = ptImpl->tControl.tCurrentReference,
        .tCurrent = ptImpl->tControl.tCurrent,
        .tVoltageReference = ptImpl->tControl.tVoltageReference,
        .tVoltage = ptImpl->tControl.tVoltage,
        .tDuty = ptImpl->tControl.tDuty,
        .qSpeedReference = ptImpl->tControl.qSpeedReference,
        .qPositionReference = ptImpl->tControl.qPositionReference,
        .tOpenLoopAngle = ptImpl->tOpenLoopAngle,
        .tActiveAngle = ptImpl->tRt.tThetaE,
        .tCandidateAngle = ptImpl->tCandidateAngle,
        .qActiveSpeed = ptImpl->tRt.qOmegaE,
        .qCandidateSpeed = ptImpl->qCandidateSpeed,
        .qAngleError = ptImpl->qAngleError,
        .qBlendFactor = ptImpl->qBlendFactor,
        .tElectricalAngle = ptImpl->tRt.tThetaE,
        .qElectricalSpeed = ptImpl->tRt.qOmegaE,
        .qVbus = ptImpl->tRt.qVbus,
        .tCurrentCalibration = ptImpl->tCurrent.tCalib,
        .eControlMode = ptImpl->tControl.eMode,
        .eActiveSourceValidFlags =
            (foc_position_valid_flag_e)ptImpl->chActiveValidFlags,
        .eCandidateSourceValidFlags =
            (foc_position_valid_flag_e)ptImpl->chCandidateValidFlags,
        .bPwmEnabled = ptImpl->bPwmEnabled,
        .eStartupPhase = ptImpl->eStartupPhase,
        .ePendingCommand = ptImpl->bCommandPending ?
                           ptImpl->ePendingCommand : MOTOR_COMMAND_NONE,
    };
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
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
        .eFromState = (motor_state_e)tRecord.chFrom,
        .eToState = (motor_state_e)tRecord.chTo,
        .eResult = FOC_RESULT_OK,
    };
    if (ptEvent->eType == MOTOR_EVENT_FAULT) {
        ptEvent->wFaults = tRecord.wPayload;
    } else if (ptEvent->eType == MOTOR_EVENT_COMMAND_ACCEPTED ||
               ptEvent->eType == MOTOR_EVENT_COMMAND_REJECTED) {
        ptEvent->eCommand =
            (motor_command_e)(tRecord.wPayload & 0xFFU);
        ptEvent->eResult =
            (foc_result_t)((tRecord.wPayload >> 8) & 0xFFU);
    } else if (ptEvent->eType ==
               MOTOR_EVENT_SOURCE_VALIDITY_CHANGED) {
        ptEvent->ePositionRole =
            (motor_position_role_e)tRecord.chDetail;
        ptEvent->wPreviousValue = tRecord.wPayload & 0xFFFFU;
        ptEvent->wCurrentValue = tRecord.wPayload >> 16;
    } else if (ptEvent->eType >= MOTOR_EVENT_TRANSITION_STARTED &&
               ptEvent->eType <= MOTOR_EVENT_TRANSITION_TIMEOUT) {
        ptEvent->wPreviousValue = tRecord.wPayload & 0xFFFFU;
        ptEvent->wCurrentValue = tRecord.wPayload >> 16;
    }
    return true;
}
