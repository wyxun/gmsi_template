/*******************************************************************************
 * @file    foc_hfi.c
 * @brief   Architecture-independent HFI signal generation and demodulation
 ******************************************************************************/

#include "foc_hfi.h"

#include <stddef.h>

foc_result_t foc_hfi_Init(foc_hfi_t *ptHfi,
                          const foc_hfi_params_t *ptParams)
{
    if (ptHfi == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qPhaseStep <= FOC_ZERO ||
        ptParams->qPhaseStep >= FOC_ONE ||
        ptParams->qInjectionAmplitude <= FOC_ZERO ||
        ptParams->qInjectionAmplitude > FOC_ONE ||
        ptParams->qHighPassAlpha <= FOC_ZERO ||
        ptParams->qHighPassAlpha > FOC_ONE ||
        ptParams->qDemodAlpha <= FOC_ZERO ||
        ptParams->qDemodAlpha > FOC_ONE ||
        ptParams->qMinimumResponse <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (!foc_gain_IsValid(&ptParams->tDemodGain)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptHfi->tParams = *ptParams;
    foc_hfi_Reset(ptHfi);
    return FOC_RESULT_OK;
}

void foc_hfi_Reset(foc_hfi_t *ptHfi)
{
    if (ptHfi != NULL) {
        ptHfi->tPhase = foc_angle_from_scalar(FOC_ZERO);
        ptHfi->qPreviousCurrentD = FOC_ZERO;
        ptHfi->qPreviousCarrier = FOC_ZERO;
        ptHfi->qHighPass = FOC_ZERO;
        ptHfi->qDemodulated = FOC_ZERO;
        ptHfi->bHasPreviousCarrier = false;
    }
}

foc_result_t foc_hfi_Step(foc_hfi_t *ptHfi,
                          foc_scalar_t qCurrentD,
                          foc_hfi_output_t *ptOutput)
{
    foc_scalar_t qCarrier;
    foc_scalar_t qHighPassInput;
    foc_scalar_t qDemodInput;

    if (ptHfi == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    qCarrier = foc_angle_cos(ptHfi->tPhase);
    qHighPassInput = foc_add_sat(
        ptHfi->qHighPass,
        foc_sub_sat(qCurrentD, ptHfi->qPreviousCurrentD));
    ptHfi->qHighPass = foc_mul_pu(
        ptHfi->tParams.qHighPassAlpha, qHighPassInput);
    if (ptHfi->bHasPreviousCarrier) {
        /* The current ADC sample responds to the injection returned by the
         * previous call, not the carrier being prepared below. */
        qDemodInput = foc_gain_apply(
            &ptHfi->tParams.tDemodGain,
            foc_mul_pu(ptHfi->qHighPass, ptHfi->qPreviousCarrier));
        ptHfi->qDemodulated = foc_add_sat(
            ptHfi->qDemodulated,
            foc_mul_pu(
                ptHfi->tParams.qDemodAlpha,
                foc_sub_sat(qDemodInput, ptHfi->qDemodulated)));
    }
    ptHfi->qPreviousCurrentD = qCurrentD;
    ptOutput->qInjectionD = foc_mul_pu(
        ptHfi->tParams.qInjectionAmplitude, qCarrier);
    ptOutput->qPositionError = ptHfi->qDemodulated;
    ptOutput->qResponse = foc_abs(ptHfi->qDemodulated);
    ptOutput->bValid = ptHfi->bHasPreviousCarrier &&
        ptOutput->qResponse >= ptHfi->tParams.qMinimumResponse;
    ptHfi->qPreviousCarrier = qCarrier;
    ptHfi->bHasPreviousCarrier = true;
    ptHfi->tPhase.qTurns = foc_add_sat(
        ptHfi->tPhase.qTurns, ptHfi->tParams.qPhaseStep);
    ptHfi->tPhase = foc_angle_wrap(ptHfi->tPhase);
    return FOC_RESULT_OK;
}
