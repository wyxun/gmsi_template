/*******************************************************************************
 * @file    motor.c
 * @brief   Multi-instance motor aggregation object
 ******************************************************************************/

#include "motor.h"
#include "motor_private.h"

#include <stddef.h>
#include <string.h>

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
         ptConfig->tPosition.chDirection != -1)) {
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
    ptImpl->tParams = ptConfig->tParams;
    ptImpl->tHal = ptConfig->tHal;
    ptImpl->tControl.tConfig = ptConfig->tControl;
    ptImpl->tCurrent.eTopology = ptConfig->eTopology;
    ptImpl->tTime = ptConfig->tTime;
    ptImpl->tSync = ptConfig->tSync;
    ptImpl->qHighFrequencyPeriod = ptConfig->qHighFrequencyPeriod;
    ptImpl->qLowFrequencyPeriod = ptConfig->qLowFrequencyPeriod;
    ptImpl->tPositionConfig = ptConfig->tPosition;
    ptImpl->wStartupDelayMs = ptConfig->wStartupDelayMs;
    ptImpl->tCurrent.tCalib.wOffsetU = 2048U;
    ptImpl->tCurrent.tCalib.wOffsetV = 2048U;
    ptImpl->tCurrent.tCalib.wOffsetW = 2048U;
    ptImpl->tRt.eRunState = MOTOR_STATE_IDLE;
    ptImpl->eStartupPhase = MOTOR_STARTUP_IDLE;
    ptImpl->wMagic = MOTOR_IMPL_MAGIC;
    return FOC_RESULT_OK;
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
    if (!motor_private_is_initialized(ptMotor)) {
        return;
    }
    motor_impl_t *ptImpl = motor_private(ptMotor);
    wSyncState = motor_private_enter(ptImpl);
    ptImpl->tRt.wFaults |= (uint32_t)eFault;
    ptImpl->tRt.eRunState = MOTOR_STATE_FAULT;
    ptImpl->bPwmEnabled = false;
    ptImpl->bCommandPending = false;
    ptImpl->ePendingCommand = MOTOR_COMMAND_NONE;
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
        .tPhaseCurrent = {ptImpl->tCurrent.qIu, ptImpl->tCurrent.qIv,
                          ptImpl->tCurrent.qIw},
        .tCurrent = ptImpl->tControl.tCurrent,
        .tVoltage = ptImpl->tControl.tVoltage,
        .tDuty = ptImpl->tControl.tDuty,
        .tElectricalAngle = ptImpl->tRt.tThetaE,
        .qElectricalSpeed = ptImpl->tRt.qOmegaE,
        .qVbus = ptImpl->tRt.qVbus,
        .tCurrentCalibration = ptImpl->tCurrent.tCalib,
        .bPwmEnabled = ptImpl->bPwmEnabled,
        .eStartupPhase = ptImpl->eStartupPhase,
        .ePendingCommand = ptImpl->bCommandPending ?
                           ptImpl->ePendingCommand : MOTOR_COMMAND_NONE,
    };
    motor_private_exit(ptImpl, wSyncState);
    return FOC_RESULT_OK;
}
