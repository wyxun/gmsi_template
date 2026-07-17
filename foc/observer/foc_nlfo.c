/*******************************************************************************
 * @file    foc_nlfo.c
 * @brief   NLFO adapted from SguanFOC v3.1.0 for normalized fixed/float math
 *
 * Physical parameter conversion must use Ls = (Ld + Lq) / 2.  The reference
 * expression `Ld + Lq / 2` is intentionally not propagated.
 ******************************************************************************/

#include "foc_nlfo.h"

#include <stddef.h>
#include <string.h>

static foc_scalar_t nlfo_magnitude(const foc_ab_t *ptVector)
{
    foc_scalar_t qA = foc_abs(ptVector->qAlpha);
    foc_scalar_t qB = foc_abs(ptVector->qBeta);
    foc_scalar_t qMaximum = qA > qB ? qA : qB;
    foc_scalar_t qMinimum = qA > qB ? qB : qA;
    return foc_add_sat(qMaximum, foc_mul_pu(qMinimum, FOC_HALF));
}

foc_result_t foc_nlfo_Init(foc_nlfo_t *ptNlfo,
                           const foc_nlfo_params_t *ptParams)
{
    foc_scalar_t qFluxInverse;

    if (ptNlfo == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qIntegratorGain <= FOC_ZERO ||
        ptParams->qIntegratorGain > FOC_ONE ||
        ptParams->qResistance < FOC_ZERO ||
        ptParams->qAverageInductance < FOC_ZERO ||
        ptParams->qFlux <= FOC_ZERO ||
        ptParams->qCorrectionGain <= FOC_ZERO ||
        ptParams->qMinimumFluxRatio <= FOC_ZERO ||
        ptParams->qMinimumFluxRatio > FOC_ONE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptNlfo, 0, sizeof(*ptNlfo));
    ptNlfo->tParams = *ptParams;
    ptNlfo->qFluxSquared = foc_mul_wide(ptParams->qFlux,
                                        ptParams->qFlux);
    if (foc_div_checked(FOC_ONE, ptParams->qFlux,
                        &qFluxInverse) != FOC_RESULT_OK) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    return foc_gain_from_scalar(qFluxInverse, &ptNlfo->tFluxInverse);
}

void foc_nlfo_Reset(foc_nlfo_t *ptNlfo)
{
    foc_nlfo_params_t tParams;
    foc_gain_t tFluxInverse;
    foc_scalar_t qFluxSquared;

    if (ptNlfo != NULL) {
        tParams = ptNlfo->tParams;
        tFluxInverse = ptNlfo->tFluxInverse;
        qFluxSquared = ptNlfo->qFluxSquared;
        memset(ptNlfo, 0, sizeof(*ptNlfo));
        ptNlfo->tParams = tParams;
        ptNlfo->tFluxInverse = tFluxInverse;
        ptNlfo->qFluxSquared = qFluxSquared;
    }
}

foc_result_t foc_nlfo_Step(foc_nlfo_t *ptNlfo,
                           const foc_observer_input_t *ptInput,
                           foc_observer_output_t *ptOutput)
{
    foc_ab_t tCurrentFlux;
    foc_ab_t tRotorFlux;
    foc_ab_t tDerivative;
    foc_scalar_t qMagnitudeSquared;
    foc_scalar_t qFluxError;
    foc_scalar_t qMagnitude;
    foc_scalar_t qFluxRatio;
    foc_angle_t tNewAngle;
    bool bValid;

    if (ptNlfo == NULL || ptInput == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    tCurrentFlux.qAlpha = foc_mul_pu(
        ptNlfo->tParams.qAverageInductance,
        ptInput->tCurrent.qAlpha);
    tCurrentFlux.qBeta = foc_mul_pu(
        ptNlfo->tParams.qAverageInductance,
        ptInput->tCurrent.qBeta);
    tRotorFlux.qAlpha = foc_sub_sat(ptNlfo->tStatorFlux.qAlpha,
                                    tCurrentFlux.qAlpha);
    tRotorFlux.qBeta = foc_sub_sat(ptNlfo->tStatorFlux.qBeta,
                                   tCurrentFlux.qBeta);
    qMagnitudeSquared = foc_add_sat(
        foc_mul_pu(tRotorFlux.qAlpha, tRotorFlux.qAlpha),
        foc_mul_pu(tRotorFlux.qBeta, tRotorFlux.qBeta));
    qFluxError = foc_sub_sat(ptNlfo->qFluxSquared,
                             qMagnitudeSquared);
    tDerivative.qAlpha = foc_add_sat(
        foc_sub_sat(ptInput->tVoltage.qAlpha,
                    foc_mul_pu(ptNlfo->tParams.qResistance,
                               ptInput->tCurrent.qAlpha)),
        foc_mul_pu(ptNlfo->tParams.qCorrectionGain,
                   foc_mul_pu(tRotorFlux.qAlpha, qFluxError)));
    tDerivative.qBeta = foc_add_sat(
        foc_sub_sat(ptInput->tVoltage.qBeta,
                    foc_mul_pu(ptNlfo->tParams.qResistance,
                               ptInput->tCurrent.qBeta)),
        foc_mul_pu(ptNlfo->tParams.qCorrectionGain,
                   foc_mul_pu(tRotorFlux.qBeta, qFluxError)));
    ptNlfo->tStatorFlux.qAlpha = foc_add_sat(
        ptNlfo->tStatorFlux.qAlpha,
        foc_mul_pu(ptNlfo->tParams.qIntegratorGain,
                   foc_sat(tDerivative.qAlpha, FOC_NEG_ONE, FOC_ONE)));
    ptNlfo->tStatorFlux.qBeta = foc_add_sat(
        ptNlfo->tStatorFlux.qBeta,
        foc_mul_pu(ptNlfo->tParams.qIntegratorGain,
                   foc_sat(tDerivative.qBeta, FOC_NEG_ONE, FOC_ONE)));

    tRotorFlux.qAlpha = foc_sub_sat(ptNlfo->tStatorFlux.qAlpha,
                                    tCurrentFlux.qAlpha);
    tRotorFlux.qBeta = foc_sub_sat(ptNlfo->tStatorFlux.qBeta,
                                   tCurrentFlux.qBeta);
    qMagnitude = nlfo_magnitude(&tRotorFlux);
    qFluxRatio = foc_gain_apply(&ptNlfo->tFluxInverse, qMagnitude);
    bValid = qFluxRatio >= ptNlfo->tParams.qMinimumFluxRatio;
    tNewAngle = foc_angle_atan2(
        foc_sat(foc_gain_apply(&ptNlfo->tFluxInverse,
                               tRotorFlux.qBeta),
                FOC_NEG_ONE, FOC_ONE),
        foc_sat(foc_gain_apply(&ptNlfo->tFluxInverse,
                               tRotorFlux.qAlpha),
                FOC_NEG_ONE, FOC_ONE));
    if (bValid && ptNlfo->bHasAngle) {
        ptNlfo->qSpeed = foc_angle_diff(tNewAngle, ptNlfo->tAngle);
    }
    if (bValid) {
        ptNlfo->tAngle = tNewAngle;
        ptNlfo->bHasAngle = true;
    }
    *ptOutput = (foc_observer_output_t){
        .tAngle = ptNlfo->tAngle,
        .qSpeed = ptNlfo->qSpeed,
        .qConfidence = foc_sat(qFluxRatio, FOC_ZERO, FOC_ONE),
        .bValid = bValid && ptNlfo->bHasAngle,
    };
    return FOC_RESULT_OK;
}

static void nlfo_interface_reset(void *pContext)
{
    foc_nlfo_Reset((foc_nlfo_t *)pContext);
}

static foc_result_t nlfo_interface_step(
    void *pContext,
    const foc_observer_input_t *ptInput,
    foc_observer_output_t *ptOutput)
{
    return foc_nlfo_Step((foc_nlfo_t *)pContext, ptInput, ptOutput);
}

foc_observer_if_t foc_nlfo_ObserverInterface(foc_nlfo_t *ptNlfo)
{
    foc_observer_if_t tInterface = {
        .pContext = ptNlfo,
        .fnReset = nlfo_interface_reset,
        .fnStep = nlfo_interface_step,
    };
    return tInterface;
}
