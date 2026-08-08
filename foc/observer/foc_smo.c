/*******************************************************************************
 * @file    foc_smo.c
 * @brief   Bounded SMO adapted from SguanFOC v3.1.0
 ******************************************************************************/

#include "foc_smo.h"

#include <stddef.h>
#include <string.h>

static foc_scalar_t smo_axis_step(foc_smo_t *ptSmo,
                                  foc_scalar_t qMeasuredCurrent,
                                  foc_scalar_t qVoltage,
                                  foc_scalar_t *pqEstimatedCurrent,
                                  foc_scalar_t *pqBemf)
{
    foc_scalar_t qResidual;
    foc_scalar_t qError;
    foc_scalar_t qSwitch;

    qResidual = foc_sub_sat(
        foc_sub_sat(qVoltage,
                    foc_mul_pu(ptSmo->tParams.qResistance,
                               *pqEstimatedCurrent)),
        *pqBemf);
    *pqEstimatedCurrent = foc_add_sat(
        *pqEstimatedCurrent,
        foc_mul_pu(ptSmo->tParams.qModelGain,
                   foc_sat(qResidual, FOC_NEG_ONE, FOC_ONE)));
    qError = foc_sub_sat(*pqEstimatedCurrent, qMeasuredCurrent);
    qSwitch = foc_gain_apply(&ptSmo->tBoundaryInverse, qError);
    qSwitch = foc_mul_pu(ptSmo->tParams.qSlidingGain,
                         foc_sat(qSwitch, FOC_NEG_ONE, FOC_ONE));
    *pqBemf = foc_add_sat(
        *pqBemf,
        foc_mul_pu(ptSmo->tParams.qEmfFilterAlpha,
                   foc_sub_sat(qSwitch, *pqBemf)));
    return qError;
}

static foc_scalar_t observer_vector_magnitude(const foc_ab_t *ptVector)
{
    foc_scalar_t qA = foc_abs(ptVector->qAlpha);
    foc_scalar_t qB = foc_abs(ptVector->qBeta);
    foc_scalar_t qMaximum = qA > qB ? qA : qB;
    foc_scalar_t qMinimum = qA > qB ? qB : qA;
    return foc_add_sat(qMaximum, foc_mul_pu(qMinimum, FOC_HALF));
}

foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const foc_smo_params_t *ptParams)
{
    if (ptSmo == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qModelGain <= FOC_ZERO ||
        ptParams->qModelGain > FOC_ONE ||
        ptParams->qResistance < FOC_ZERO ||
        ptParams->qSlidingGain <= FOC_ZERO ||
        ptParams->qEmfFilterAlpha <= FOC_ZERO ||
        ptParams->qEmfFilterAlpha > FOC_ONE ||
        ptParams->qBoundaryInverse <= FOC_ZERO ||
        ptParams->qMinimumBemf <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptSmo, 0, sizeof(*ptSmo));
    ptSmo->tParams = *ptParams;
    return foc_gain_from_scalar(ptParams->qBoundaryInverse,
                                &ptSmo->tBoundaryInverse);
}

void foc_smo_Reset(foc_smo_t *ptSmo)
{
    foc_smo_params_t tParams;
    foc_gain_t tBoundaryInverse;

    if (ptSmo != NULL) {
        tParams = ptSmo->tParams;
        tBoundaryInverse = ptSmo->tBoundaryInverse;
        memset(ptSmo, 0, sizeof(*ptSmo));
        ptSmo->tParams = tParams;
        ptSmo->tBoundaryInverse = tBoundaryInverse;
    }
}

foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_position_input_t *ptInput,
                          foc_position_output_t *ptOutput)
{
    foc_scalar_t qMagnitude;
    foc_angle_t tNewAngle;
    bool bValid;

    if (ptSmo == NULL || ptInput == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    (void)smo_axis_step(ptSmo, ptInput->tCurrent.qAlpha,
                        ptInput->tVoltage.qAlpha,
                        &ptSmo->tEstimatedCurrent.qAlpha,
                        &ptSmo->tBemf.qAlpha);
    (void)smo_axis_step(ptSmo, ptInput->tCurrent.qBeta,
                        ptInput->tVoltage.qBeta,
                        &ptSmo->tEstimatedCurrent.qBeta,
                        &ptSmo->tBemf.qBeta);
    qMagnitude = observer_vector_magnitude(&ptSmo->tBemf);
    bValid = qMagnitude >= ptSmo->tParams.qMinimumBemf;
    tNewAngle = foc_angle_atan2(
        foc_sub_sat(FOC_ZERO, ptSmo->tBemf.qAlpha),
        ptSmo->tBemf.qBeta);
    if (bValid && ptSmo->bHasAngle) {
        ptSmo->qSpeed = foc_angle_diff(tNewAngle, ptSmo->tAngle);
    }
    if (bValid) {
        ptSmo->tAngle = tNewAngle;
        ptSmo->bHasAngle = true;
    }
    *ptOutput = (foc_position_output_t){
        .tElectricalAngle = ptSmo->tAngle,
        .qElectricalSpeed = ptSmo->qSpeed,
        .qConfidence = foc_sat(qMagnitude, FOC_ZERO, FOC_ONE),
        .eValidFlags = bValid && ptSmo->bHasAngle
            ? (FOC_POSITION_VALID_ELECTRICAL_ANGLE |
               FOC_POSITION_VALID_ELECTRICAL_SPEED)
            : FOC_POSITION_VALID_NONE,
    };
    return FOC_RESULT_OK;
}

static void smo_interface_reset(void *pSourceContext)
{
    foc_smo_Reset((foc_smo_t *)pSourceContext);
}

static foc_result_t smo_interface_step(
    void *pSourceContext,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput)
{
    foc_result_t eResult = foc_smo_Step((foc_smo_t *)pSourceContext, ptInput,
                                        ptOutput);
    if (eResult == FOC_RESULT_OK) {
        ptOutput->wTimestamp = ptInput->wTimestamp;
    }
    return eResult;
}

foc_position_source_if_t foc_smo_PositionSourceInterface(foc_smo_t *ptSmo)
{
    foc_position_source_if_t tInterface = {
        .pSourceContext = ptSmo,
        .fnReset = smo_interface_reset,
        .fnStep = smo_interface_step,
    };
    return tInterface;
}
