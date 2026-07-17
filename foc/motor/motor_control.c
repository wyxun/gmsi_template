/*******************************************************************************
 * @file    motor_control.c
 * @brief   Per-instance FOC loop orchestration
 ******************************************************************************/

#include "motor_control.h"

#include <stddef.h>

static bool motor_control_mode_is_valid(const motor_control_t *ptControl,
                                        motor_control_mode_t eMode)
{
    if (eMode > MOTOR_CONTROL_POSITION) {
        return false;
    }
    if (eMode >= MOTOR_CONTROL_CURRENT &&
        (!foc_controller_IsValid(&ptControl->tConfig.tIdController) ||
         !foc_controller_IsValid(&ptControl->tConfig.tIqController))) {
        return false;
    }
    if (eMode >= MOTOR_CONTROL_SPEED &&
        !foc_controller_IsValid(&ptControl->tConfig.tSpeedController)) {
        return false;
    }
    if (eMode >= MOTOR_CONTROL_POSITION &&
        !foc_controller_IsValid(&ptControl->tConfig.tPositionController)) {
        return false;
    }
    return ptControl->tConfig.eModulation <=
           MOTOR_MODULATION_THIRD_HARMONIC;
}

static void motor_control_reset_controllers(motor_control_t *ptControl)
{
    foc_controller_Reset(&ptControl->tConfig.tIdController);
    foc_controller_Reset(&ptControl->tConfig.tIqController);
    foc_controller_Reset(&ptControl->tConfig.tSpeedController);
    foc_controller_Reset(&ptControl->tConfig.tPositionController);
}

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

foc_result_t motor_ControlStart(motor_handle_t *ptMotor,
                                motor_control_mode_t eMode)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tRt.wFaults != MOTOR_FAULT_NONE ||
        !motor_control_mode_is_valid(&ptMotor->tControl, eMode)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    motor_control_reset_controllers(&ptMotor->tControl);
    ptMotor->tControl.eMode = eMode;
    ptMotor->tRt.eRunState =
        eMode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP ?
        MOTOR_STATE_OPEN_LOOP : MOTOR_STATE_CLOSE_LOOP;
    if (motor_Enable(ptMotor, true) != FOC_RESULT_OK) {
        motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

void motor_ControlStop(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    (void)motor_Enable(ptMotor, false);
    motor_control_reset_controllers(&ptMotor->tControl);
    ptMotor->tControl.tVoltage = (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptMotor->tControl.tDuty = (foc_duty_abc_t){FOC_HALF,
                                               FOC_HALF,
                                               FOC_HALF};
    ptMotor->tRt.eRunState = MOTOR_STATE_IDLE;
}

void motor_ControlSetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (ptMotor != NULL) {
        ptMotor->tControl.tVoltageReference.qD = qD;
        ptMotor->tControl.tVoltageReference.qQ = qQ;
    }
}

void motor_ControlSetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (ptMotor != NULL) {
        ptMotor->tControl.tCurrentReference.qD = qD;
        ptMotor->tControl.tCurrentReference.qQ = qQ;
    }
}

void motor_ControlSetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed)
{
    if (ptMotor != NULL) {
        ptMotor->tControl.qSpeedReference = qSpeed;
    }
}

void motor_ControlSetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition)
{
    if (ptMotor != NULL) {
        ptMotor->tControl.qPositionReference = qPosition;
    }
}

foc_result_t motor_ControlLowFrequencyStep(motor_handle_t *ptMotor)
{
    motor_control_t *ptControl;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    ptControl = &ptMotor->tControl;
    if (ptMotor->tRt.eRunState != MOTOR_STATE_CLOSE_LOOP) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptControl->eMode == MOTOR_CONTROL_POSITION) {
        ptControl->qSpeedReference = foc_controller_Step(
            &ptControl->tConfig.tPositionController,
            ptControl->qPositionReference,
            ptMotor->tRt.tThetaE.qTurns);
    }
    if (ptControl->eMode >= MOTOR_CONTROL_SPEED) {
        ptControl->tCurrentReference.qQ = foc_controller_Step(
            &ptControl->tConfig.tSpeedController,
            ptControl->qSpeedReference,
            ptMotor->tRt.qOmegaE);
    }
    return FOC_RESULT_OK;
}

foc_result_t motor_ControlHighFrequencyStep(motor_handle_t *ptMotor)
{
    motor_control_t *ptControl;
    foc_result_t eResult;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tRt.eRunState != MOTOR_STATE_OPEN_LOOP &&
        ptMotor->tRt.eRunState != MOTOR_STATE_CLOSE_LOOP) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptControl = &ptMotor->tControl;
    eResult = motor_SampleCurrent(ptMotor);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_clarke(ptMotor->tCurrent.qIu,
                         ptMotor->tCurrent.qIv,
                         ptMotor->tCurrent.qIw,
                         &ptControl->tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    eResult = foc_park(&ptControl->tCurrentAlphaBeta,
                       ptMotor->tRt.tThetaE,
                       &ptControl->tCurrent);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    if (ptControl->eMode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP) {
        ptControl->tVoltage = ptControl->tVoltageReference;
    } else {
        ptControl->tVoltage.qD = foc_controller_Step(
            &ptControl->tConfig.tIdController,
            ptControl->tCurrentReference.qD,
            ptControl->tCurrent.qD);
        ptControl->tVoltage.qQ = foc_controller_Step(
            &ptControl->tConfig.tIqController,
            ptControl->tCurrentReference.qQ,
            ptControl->tCurrent.qQ);
    }
    eResult = foc_ipark(&ptControl->tVoltage,
                        ptMotor->tRt.tThetaE,
                        &ptControl->tVoltageAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    eResult = motor_control_modulate(ptControl);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    eResult = motor_SetDuty(ptMotor, ptControl->tDuty.qU,
                            ptControl->tDuty.qV,
                            ptControl->tDuty.qW);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    ptMotor->tRt.qId = ptControl->tCurrent.qD;
    ptMotor->tRt.qIq = ptControl->tCurrent.qQ;
    return FOC_RESULT_OK;

fail:
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_INVALID_COMMAND);
    return eResult;
}
