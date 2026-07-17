/*******************************************************************************
 * @file    foc_nsd.c
 * @brief   Safe non-blocking NSD sequence adapted from SguanFOC v3.1.0
 ******************************************************************************/

#include "foc_nsd.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#if FOC_ENABLE_EXPERIMENTAL_NSD
static void nsd_output(const foc_nsd_t *ptNsd,
                       foc_nsd_output_t *ptOutput)
{
    ptOutput->qVoltageD = FOC_ZERO;
    if (ptNsd->eState == FOC_NSD_POSITIVE) {
        ptOutput->qVoltageD = ptNsd->tParams.qTestVoltage;
    } else if (ptNsd->eState == FOC_NSD_NEGATIVE) {
        ptOutput->qVoltageD = foc_sub_sat(
            FOC_ZERO, ptNsd->tParams.qTestVoltage);
    }
    ptOutput->bReversePolarity = ptNsd->bReversePolarity;
    ptOutput->bComplete = ptNsd->bComplete;
    ptOutput->eState = ptNsd->eState;
}

static foc_result_t nsd_abort(foc_nsd_t *ptNsd,
                              foc_nsd_output_t *ptOutput)
{
    ptNsd->eState = FOC_NSD_ABORTED;
    ptNsd->bComplete = false;
    foc_experiment_EmergencyStop(&ptNsd->tParams.tSafety);
    nsd_output(ptNsd, ptOutput);
    return FOC_RESULT_SAFETY;
}
#endif

foc_result_t foc_nsd_Init(foc_nsd_t *ptNsd,
                          const foc_nsd_params_t *ptParams)
{
#if !FOC_ENABLE_EXPERIMENTAL_NSD
    (void)ptNsd;
    (void)ptParams;
    return FOC_RESULT_DISABLED;
#else
    foc_result_t eResult;

    if (ptNsd == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = foc_experiment_ValidateSafety(&ptParams->tSafety);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptParams->qTestVoltage <= FOC_ZERO ||
        ptParams->qTestVoltage > FOC_ONE ||
        ptParams->wSettleSamples == 0U ||
        ptParams->wPositiveSamples == 0U ||
        ptParams->wZeroSamples == 0U ||
        ptParams->wNegativeSamples == 0U ||
        ptParams->wPositiveSamples >
            (uint32_t)(INT32_MAX / FOC_Q_SCALE) ||
        ptParams->wNegativeSamples >
            (uint32_t)(INT32_MAX / FOC_Q_SCALE)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptNsd, 0, sizeof(*ptNsd));
    ptNsd->tParams = *ptParams;
    return FOC_RESULT_OK;
#endif
}

foc_result_t foc_nsd_Start(foc_nsd_t *ptNsd,
                           const foc_experiment_guard_t *ptGuard)
{
#if !FOC_ENABLE_EXPERIMENTAL_NSD
    (void)ptNsd;
    (void)ptGuard;
    return FOC_RESULT_DISABLED;
#else
    if (ptNsd == NULL || ptGuard == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptGuard->bMotorStopped ||
        !foc_experiment_IsSafe(&ptNsd->tParams.tSafety, ptGuard)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptNsd->eState = FOC_NSD_SETTLE;
    ptNsd->wStateSamples = 0U;
    ptNsd->wTotalSamples = 0U;
    ptNsd->qPositiveResponse = FOC_ZERO;
    ptNsd->qNegativeResponse = FOC_ZERO;
    ptNsd->bReversePolarity = false;
    ptNsd->bComplete = false;
    return FOC_RESULT_OK;
#endif
}

foc_result_t foc_nsd_Step(foc_nsd_t *ptNsd,
                          const foc_experiment_guard_t *ptGuard,
                          foc_scalar_t qDemodulatedResponse,
                          foc_nsd_output_t *ptOutput)
{
#if !FOC_ENABLE_EXPERIMENTAL_NSD
    (void)ptNsd;
    (void)ptGuard;
    (void)qDemodulatedResponse;
    if (ptOutput != NULL) {
        memset(ptOutput, 0, sizeof(*ptOutput));
        ptOutput->eState = FOC_NSD_IDLE;
    }
    return FOC_RESULT_DISABLED;
#else
    uint32_t wDuration = 0U;

    if (ptNsd == NULL || ptGuard == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_experiment_IsSafe(&ptNsd->tParams.tSafety, ptGuard)) {
        return nsd_abort(ptNsd, ptOutput);
    }
    if (ptNsd->eState == FOC_NSD_COMPLETE) {
        nsd_output(ptNsd, ptOutput);
        return FOC_RESULT_OK;
    }
    if (ptNsd->eState == FOC_NSD_IDLE ||
        ptNsd->eState == FOC_NSD_ABORTED) {
        nsd_output(ptNsd, ptOutput);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptNsd->wTotalSamples++;
    if (ptNsd->wTotalSamples > ptNsd->tParams.tSafety.wTimeoutSamples) {
        return nsd_abort(ptNsd, ptOutput);
    }
    ptNsd->wStateSamples++;
    switch (ptNsd->eState) {
    case FOC_NSD_SETTLE:
        wDuration = ptNsd->tParams.wSettleSamples;
        break;
    case FOC_NSD_POSITIVE:
        ptNsd->qPositiveResponse = foc_add_sat(
            ptNsd->qPositiveResponse, foc_abs(qDemodulatedResponse));
        wDuration = ptNsd->tParams.wPositiveSamples;
        break;
    case FOC_NSD_ZERO:
        wDuration = ptNsd->tParams.wZeroSamples;
        break;
    case FOC_NSD_NEGATIVE:
        ptNsd->qNegativeResponse = foc_add_sat(
            ptNsd->qNegativeResponse, foc_abs(qDemodulatedResponse));
        wDuration = ptNsd->tParams.wNegativeSamples;
        break;
    default:
        break;
    }
    if (ptNsd->wStateSamples >= wDuration) {
        ptNsd->wStateSamples = 0U;
        ptNsd->eState = (foc_nsd_state_t)(ptNsd->eState + 1);
        if (ptNsd->eState == FOC_NSD_COMPLETE) {
            foc_scalar_t qPositiveAverage;
            foc_scalar_t qNegativeAverage;
#if defined(FOC_NUMERIC_FLOAT)
            foc_scalar_t qPositiveCount =
                (foc_scalar_t)ptNsd->tParams.wPositiveSamples;
            foc_scalar_t qNegativeCount =
                (foc_scalar_t)ptNsd->tParams.wNegativeSamples;
#else
            foc_scalar_t qPositiveCount = (foc_scalar_t)(
                ptNsd->tParams.wPositiveSamples * FOC_Q_SCALE);
            foc_scalar_t qNegativeCount = (foc_scalar_t)(
                ptNsd->tParams.wNegativeSamples * FOC_Q_SCALE);
#endif
            if (foc_div_checked(ptNsd->qPositiveResponse,
                                qPositiveCount,
                                &qPositiveAverage) != FOC_RESULT_OK ||
                foc_div_checked(ptNsd->qNegativeResponse,
                                qNegativeCount,
                                &qNegativeAverage) != FOC_RESULT_OK) {
                return nsd_abort(ptNsd, ptOutput);
            }
            ptNsd->bReversePolarity =
                qNegativeAverage > qPositiveAverage;
            ptNsd->bComplete = true;
        }
    }
    nsd_output(ptNsd, ptOutput);
    return FOC_RESULT_OK;
#endif
}
