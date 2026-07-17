/*******************************************************************************
 * @file    foc_optimization.c
 * @brief   Multi-instance motor optimization adapted from SguanFOC v3.1.0
 ******************************************************************************/

#include "foc_optimization.h"

#include <stddef.h>

#include "foc_math.h"

foc_result_t foc_mtpa_Calculate(foc_scalar_t qFlux,
                                foc_scalar_t qLd,
                                foc_scalar_t qLq,
                                foc_scalar_t qIq,
                                foc_scalar_t *pqId)
{
    foc_scalar_t qDelta;
    foc_scalar_t qFluxSquare;
    foc_scalar_t qCurrentTerm;
    foc_scalar_t qRoot;
    foc_scalar_t qDenominator;
    foc_result_t eResult;

    if (pqId == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qFlux < FOC_ZERO || qLd < FOC_ZERO || qLq < FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    qDelta = foc_sub_sat(qLq, qLd);
    if (qDelta <= FOC_ZERO) {
        *pqId = FOC_ZERO;
        return FOC_RESULT_OK;
    }
    qFluxSquare = foc_mul_wide(qFlux, qFlux);
    qCurrentTerm = foc_mul_wide(qDelta, qDelta);
    qCurrentTerm = foc_mul_wide(qCurrentTerm, foc_mul_wide(qIq, qIq));
    qCurrentTerm = foc_add_sat(qCurrentTerm, qCurrentTerm);
    qCurrentTerm = foc_add_sat(qCurrentTerm, qCurrentTerm);
    qRoot = foc_sqrt(foc_add_sat(qFluxSquare, qCurrentTerm));
    qDenominator = foc_add_sat(qDelta, qDelta);
    eResult = foc_div_checked(foc_sub_sat(qFlux, qRoot),
                              qDenominator, pqId);
    if (eResult == FOC_RESULT_OK && *pqId > FOC_ZERO) {
        *pqId = FOC_ZERO;
    }
    return eResult;
}

foc_result_t foc_field_weakening_Init(
    foc_field_weakening_t *ptWeakening,
    const foc_field_weakening_params_t *ptParams)
{
    foc_result_t eResult;

    if (ptWeakening == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qBaseSpeed <= FOC_ZERO ||
        ptParams->qVoltageLimit <= FOC_ZERO ||
        ptParams->qMinimumId >= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_pid_Init(&ptWeakening->tVoltagePid,
                           &ptParams->tVoltagePid);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    ptWeakening->tParams = *ptParams;
    return FOC_RESULT_OK;
}

void foc_field_weakening_Reset(foc_field_weakening_t *ptWeakening)
{
    if (ptWeakening != NULL) {
        foc_pid_Reset(&ptWeakening->tVoltagePid);
    }
}

foc_scalar_t foc_field_weakening_Step(
    foc_field_weakening_t *ptWeakening,
    foc_scalar_t qElectricalSpeed,
    const foc_dq_t *ptVoltage)
{
    foc_scalar_t qMagnitude;
    foc_scalar_t qId;

    if (ptWeakening == NULL || ptVoltage == NULL) {
        return FOC_ZERO;
    }
    if (foc_abs(qElectricalSpeed) < ptWeakening->tParams.qBaseSpeed) {
        foc_field_weakening_Reset(ptWeakening);
        return FOC_ZERO;
    }
    qMagnitude = foc_sqrt(
        foc_add_sat(foc_mul_wide(ptVoltage->qD, ptVoltage->qD),
                    foc_mul_wide(ptVoltage->qQ, ptVoltage->qQ)));
    qId = foc_pid_Step(&ptWeakening->tVoltagePid,
                       ptWeakening->tParams.qVoltageLimit, qMagnitude);
    return foc_sat(qId, ptWeakening->tParams.qMinimumId, FOC_ZERO);
}

static foc_scalar_t deadtime_phase(foc_scalar_t qDuty,
                                   foc_scalar_t qCurrent,
                                   const foc_deadtime_params_t *ptParams)
{
    if (qCurrent > ptParams->qCurrentThreshold) {
        qDuty = foc_add_sat(qDuty, ptParams->qCompensation);
    } else if (qCurrent < -ptParams->qCurrentThreshold) {
        qDuty = foc_sub_sat(qDuty, ptParams->qCompensation);
    }
    return foc_sat(qDuty, FOC_ZERO, FOC_ONE);
}

foc_result_t foc_deadtime_Compensate(
    const foc_deadtime_params_t *ptParams,
    const foc_abc_t *ptCurrent,
    foc_duty_abc_t *ptDuty)
{
    if (ptParams == NULL || ptCurrent == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qCompensation < FOC_ZERO ||
        ptParams->qCurrentThreshold < FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptDuty->qU = deadtime_phase(ptDuty->qU, ptCurrent->qA, ptParams);
    ptDuty->qV = deadtime_phase(ptDuty->qV, ptCurrent->qB, ptParams);
    ptDuty->qW = deadtime_phase(ptDuty->qW, ptCurrent->qC, ptParams);
    return FOC_RESULT_OK;
}

foc_result_t foc_phase_delay_Compensate(
    foc_angle_t tAngle,
    foc_scalar_t qElectricalSpeed,
    const foc_gain_t *ptDelayTurnsPerSpeed,
    foc_scalar_t qDirectionOffset,
    foc_angle_t *ptCompensated)
{
    foc_scalar_t qCorrection;

    if (ptDelayTurnsPerSpeed == NULL || ptCompensated == NULL) {
        return FOC_RESULT_NULL;
    }
    qCorrection = foc_gain_apply(ptDelayTurnsPerSpeed, qElectricalSpeed);
    if (qElectricalSpeed > FOC_ZERO) {
        qCorrection = foc_add_sat(qCorrection, qDirectionOffset);
    } else if (qElectricalSpeed < FOC_ZERO) {
        qCorrection = foc_sub_sat(qCorrection, qDirectionOffset);
    }
    ptCompensated->qTurns = foc_add_sat(tAngle.qTurns, qCorrection);
    *ptCompensated = foc_angle_wrap(*ptCompensated);
    return FOC_RESULT_OK;
}
