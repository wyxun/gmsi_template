/****************************************************************************
 * @file    motor.c
 * @brief   Single-motor FOC domain object and hard-real-time lifecycle.
 * @author  Codex
 * @date    2026-09-08
 ****************************************************************************/

#include <stddef.h>

#include "motor.h"

#define MOTOR_CALIBRATION_MAX_TICKS 2000U

static foc_result_t motor_Activate(motor_t *ptMotor);

static bool motor_ConfigValid(const motor_cfg_t *ptConfig)
{
    if (ptConfig == NULL || ptConfig->ptAdcOps == NULL ||
        ptConfig->ptPwmOps == NULL || ptConfig->tPosition.ptOps == NULL ||
        ptConfig->ptAdcOps->fnCalibrationBegin == NULL ||
        ptConfig->ptAdcOps->fnCalibrationStep == NULL ||
        ptConfig->ptAdcOps->fnCurrentSample == NULL ||
        ptConfig->ptPwmOps->fnDutyCommit == NULL ||
        ptConfig->ptPwmOps->fnPwmEnable == NULL ||
        ptConfig->ptPwmOps->fnEmergencyStop == NULL ||
        ptConfig->tPosition.ptOps->fnInit == NULL ||
        ptConfig->tPosition.ptOps->fnRead == NULL ||
        ptConfig->tMotorParams.chPolePairs == 0U ||
        ptConfig->tControlCfg.qHighFrequencyPeriod <= FOC_ZERO ||
        ptConfig->tControlCfg.hwCalibrationTimeoutTicks == 0U) {
        return false;
    }
    return true;
}

static void motor_EnterFault(motor_t *ptMotor, uint32_t wFault)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
    ptMotor->bPwmEnabled = false;
    ptMotor->wFaults |= wFault;
    ptMotor->eLifecycle = MOTOR_STATE_FAULT;
}

static void motor_BeginCalibration(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptAdcOps->fnCalibrationBegin(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration);
    ptMotor->hwCalibrationTicks = 0U;
    ptMotor->eLifecycle = MOTOR_STATE_CALIBRATING;
}

static void motor_CalibrationStep(motor_t *ptMotor)
{
    foc_calibration_state_e eResult = FOC_CALIBRATION_BUSY;

    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->hwCalibrationTicks < UINT16_MAX) {
        ptMotor->hwCalibrationTicks++;
    }
    eResult = ptMotor->ptAdcOps->fnCalibrationStep(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration);
    if (eResult == FOC_CALIBRATION_BUSY) {
        if ((uint32_t)ptMotor->hwCalibrationTicks >=
            (uint32_t)ptMotor->hwCalibrationTimeoutTicks) {
            motor_EnterFault(ptMotor,
                             MOTOR_FAULT_CALIBRATION_TIMEOUT);
        }
        return;
    }
    if (eResult != FOC_CALIBRATION_COMPLETE) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_CALIBRATION);
        return;
    }
    ptMotor->tAdcCalibration.bIsCalibrated = true;
    ptMotor->eLifecycle = MOTOR_STATE_IDLE;
    if (ptMotor->bStartAfterCalibration) {
        ptMotor->bStartAfterCalibration = false;
        (void)motor_Activate(ptMotor);
    }
}

static foc_result_t motor_Activate(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    foc_core_Reset(&ptMotor->tCore);
    eResult = ptMotor->ptPwmOps->fnDutyCommit(
        ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return eResult;
    }
    eResult = ptMotor->ptPwmOps->fnPwmEnable(
        ptMotor->pPwmContext, true);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM_ENABLE);
        return eResult;
    }
    ptMotor->bPwmEnabled = true;
    ptMotor->eLifecycle = MOTOR_STATE_RUNNING;
    return FOC_RESULT_OK;
}

static bool motor_PositionStep(motor_t *ptMotor,
                               foc_core_input_t *ptInput)
{
    foc_result_t eResult = FOC_RESULT_OK;
    motor_position_feedback_t tFeedback = {0};

    if (ptMotor == NULL || ptInput == NULL) {
        return false;
    }
    if (ptMotor->tPosition.ptOps->fnObserve != NULL) {
        eResult = ptMotor->tPosition.ptOps->fnObserve(
            ptMotor->tPosition.pContext,
            &(foc_observer_input_t){0});
        if (eResult != FOC_RESULT_OK) {
            motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
            return false;
        }
    }
    eResult = ptMotor->tPosition.ptOps->fnRead(
        ptMotor->tPosition.pContext, &tFeedback);
    if (eResult != FOC_RESULT_OK || !tFeedback.bValid) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
        return false;
    }
    ptMotor->tPositionFeedback = tFeedback;
    ptInput->tElectricalAngle = tFeedback.tElectricalAngle;
    ptInput->qElectricalSpeed = tFeedback.qElectricalSpeed;
    ptInput->bAngleValid = true;
    return true;
}

static void motor_RunningStep(motor_t *ptMotor)
{
    foc_core_input_t tInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return;
    }
    eResult = ptMotor->ptAdcOps->fnCurrentSample(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration, &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
        return;
    }
    if (!motor_PositionStep(ptMotor, &tInput)) {
        return;
    }
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = ptMotor->ptPwmOps->fnDutyCommit(
        ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
    }
}

static bool motor_ConsumeCommand(motor_t *ptMotor)
{
    motor_command_e eCommand = MOTOR_COMMAND_NONE;

    if (ptMotor == NULL) {
        return false;
    }
    eCommand = ptMotor->tCommandSync.ePending;
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_NONE;
    if (eCommand == MOTOR_COMMAND_START) {
        if (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->wFaults == 0U &&
            ptMotor->tAdcCalibration.bIsCalibrated) {
            (void)motor_Activate(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = true;
            motor_BeginCalibration(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = true;
        } else {
            /* Ignore requests outside a safe start state. */
        }
        return true;
    }
    if (eCommand == MOTOR_COMMAND_STOP) {
        motor_Stop(ptMotor);
        return true;
    }
    if (eCommand == MOTOR_COMMAND_ADC_CALIBRATION) {
        if (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = false;
            motor_BeginCalibration(ptMotor);
        }
        return true;
    }
    return eCommand != MOTOR_COMMAND_NONE;
}

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_ConfigValid(ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptMotor = (motor_t){0};
    ptMotor->tParams = ptConfig->tMotorParams;
    ptMotor->ptAdcOps = ptConfig->ptAdcOps;
    ptMotor->pAdcContext = ptConfig->pAdcContext;
    ptMotor->ptPwmOps = ptConfig->ptPwmOps;
    ptMotor->pPwmContext = ptConfig->pPwmContext;
    ptMotor->tPosition = ptConfig->tPosition;
    ptMotor->qHighFrequencyPeriod =
        ptConfig->tControlCfg.qHighFrequencyPeriod;
    ptMotor->hwCalibrationTimeoutTicks =
        ptConfig->tControlCfg.hwCalibrationTimeoutTicks;
    ptMotor->eLifecycle = MOTOR_STATE_INITIALIZING;
    eResult = foc_pid_Init(&ptMotor->tCore.tIdPi,
                           &ptConfig->tControlCfg.tCurrentPiParams);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tCore.tIqPi,
                               &ptConfig->tControlCfg.tCurrentPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tSpeedPi,
                               &ptConfig->tControlCfg.tSpeedPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = ptMotor->tPosition.ptOps->fnInit(
            ptMotor->tPosition.pContext, &ptMotor->tParams,
            ptMotor->qHighFrequencyPeriod);
    }
    if (eResult != FOC_RESULT_OK) {
        ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
        ptMotor->eLifecycle = MOTOR_STATE_FAULT;
        return eResult;
    }
    foc_core_Reset(&ptMotor->tCore);
    return FOC_RESULT_OK;
}

void motor_HighFrequencyStep(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    if (motor_ConsumeCommand(ptMotor)) {
        return;
    }
    switch (ptMotor->eLifecycle) {
    case MOTOR_STATE_INITIALIZING:
        motor_BeginCalibration(ptMotor);
        break;
    case MOTOR_STATE_CALIBRATING:
        motor_CalibrationStep(ptMotor);
        break;
    case MOTOR_STATE_RUNNING:
        motor_RunningStep(ptMotor);
        break;
    case MOTOR_STATE_IDLE:
    case MOTOR_STATE_FAULT:
        break;
    default:
        motor_EnterFault(ptMotor, MOTOR_FAULT_STATE);
        break;
    }
}

foc_result_t motor_Start(motor_t *ptMotor,
                         const foc_core_command_t *ptCommand)
{
    if (ptMotor == NULL || ptCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCommand->eMode > FOC_MODE_SPEED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if ((ptMotor->eLifecycle != MOTOR_STATE_IDLE &&
         ptMotor->eLifecycle != MOTOR_STATE_INITIALIZING &&
         ptMotor->eLifecycle != MOTOR_STATE_CALIBRATING) ||
        ptMotor->wFaults != 0U) {
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = *ptCommand;
    if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING) {
        ptMotor->bStartAfterCalibration = true;
        motor_BeginCalibration(ptMotor);
    } else if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING) {
        ptMotor->bStartAfterCalibration = true;
    } else {
        ptMotor->tCommandSync.ePending = MOTOR_COMMAND_START;
    }
    return FOC_RESULT_OK;
}

void motor_Stop(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
    ptMotor->bPwmEnabled = false;
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_NONE;
    ptMotor->eLifecycle = ptMotor->wFaults == 0U
                              ? MOTOR_STATE_IDLE : MOTOR_STATE_FAULT;
}

void motor_ClockStep(motor_t *ptMotor)
{
    foc_scalar_t qIqReference = FOC_ZERO;

    if (ptMotor == NULL || ptMotor->eLifecycle != MOTOR_STATE_RUNNING ||
        ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return;
    }
    qIqReference = foc_pid_Step(&ptMotor->tSpeedPi,
                                ptMotor->tCommand.qSpeedReference,
                                ptMotor->tPositionFeedback.qElectricalSpeed);
    ptMotor->tCommand.tCurrentReference.qQ = qIqReference;
}

void motor_BackgroundStep(motor_t *ptMotor)
{
    if (ptMotor == NULL || ptMotor->tPosition.ptOps == NULL ||
        ptMotor->tPosition.ptOps->fnSlowUpdate == NULL) {
        return;
    }
    if (ptMotor->tPosition.ptOps->fnSlowUpdate(
            ptMotor->tPosition.pContext) != 0) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
    }
}

foc_result_t motor_ClearFault(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->eLifecycle != MOTOR_STATE_FAULT ||
        ptMotor->bPwmEnabled) {
        return FOC_RESULT_BUSY;
    }
    ptMotor->wFaults = MOTOR_FAULT_NONE;
    ptMotor->eLifecycle = MOTOR_STATE_IDLE;
    return FOC_RESULT_OK;
}

foc_result_t motor_RequestAdcCalibration(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->eLifecycle != MOTOR_STATE_IDLE ||
        ptMotor->wFaults != 0U) {
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_ADC_CALIBRATION;
    return FOC_RESULT_OK;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tCommand.eMode != FOC_MODE_VOLTAGE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.tVoltageReference.qD = qD;
    ptMotor->tCommand.tVoltageReference.qQ = qQ;
    return FOC_RESULT_OK;
}

foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tCommand.eMode != FOC_MODE_CURRENT) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.tCurrentReference.qD = qD;
    ptMotor->tCommand.tCurrentReference.qQ = qQ;
    return FOC_RESULT_OK;
}

foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qElectricalSpeed)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.qSpeedReference = qElectricalSpeed;
    return FOC_RESULT_OK;
}

foc_result_t motor_GetFeedback(const motor_t *ptMotor,
                               motor_feedback_t *ptFeedback)
{
    if (ptMotor == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFeedback->tPosition = ptMotor->tPositionFeedback;
    ptFeedback->tCurrent = ptMotor->tCore.tCurrent;
    ptFeedback->tVoltage = ptMotor->tCore.tVoltage;
    ptFeedback->tDuty = ptMotor->tCore.tDuty;
    return FOC_RESULT_OK;
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    if (ptMotor == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    ptStatus->eLifecycle = ptMotor->eLifecycle;
    ptStatus->wFaults = ptMotor->wFaults;
    ptStatus->tCommand = ptMotor->tCommand;
    ptStatus->bPwmEnabled = ptMotor->bPwmEnabled;
    return FOC_RESULT_OK;
}

foc_result_t motor_CaptureElectricalZero(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tPosition.ptOps->fnCaptureElectricalZero == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (ptMotor->eLifecycle != MOTOR_STATE_RUNNING) {
        return FOC_RESULT_BUSY;
    }
    return ptMotor->tPosition.ptOps->fnCaptureElectricalZero(
        ptMotor->tPosition.pContext);
}
