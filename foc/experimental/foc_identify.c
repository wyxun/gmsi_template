/*******************************************************************************
 * @file    foc_identify.c
 * @brief   Safe non-blocking parameter identification from SguanFOC v3.1.0
 ******************************************************************************/

#include "foc_identify.h"

#include <stddef.h>
#include <string.h>

#if FOC_ENABLE_EXPERIMENTAL_IDENTIFY
static void identify_output(const foc_identify_t *ptIdentify,
                            foc_identify_output_t *ptOutput)
{
    ptOutput->tVoltage.qD = FOC_ZERO;
    ptOutput->tVoltage.qQ = FOC_ZERO;
    if (ptIdentify->eState == FOC_IDENTIFY_RS_HALF) {
        ptOutput->tVoltage.qD = ptIdentify->tParams.qHalfVoltage;
    } else if (ptIdentify->eState == FOC_IDENTIFY_RS_FULL ||
               ptIdentify->eState == FOC_IDENTIFY_LD_RISE) {
        ptOutput->tVoltage.qD = ptIdentify->tParams.qFullVoltage;
    } else if (ptIdentify->eState == FOC_IDENTIFY_LQ_RISE) {
        ptOutput->tVoltage.qQ = ptIdentify->tParams.qFullVoltage;
    } else if (ptIdentify->eState == FOC_IDENTIFY_FLUX_SETTLE) {
        ptOutput->tVoltage.qQ = foc_sat(
            ptIdentify->tParams.qFluxVoltage, FOC_ZERO, FOC_ONE);
    }
    ptOutput->qResistance = ptIdentify->qResistance;
    ptOutput->qInductanceD = ptIdentify->qInductanceD;
    ptOutput->qInductanceQ = ptIdentify->qInductanceQ;
    ptOutput->qFlux = ptIdentify->qFlux;
    ptOutput->bComplete = ptIdentify->bComplete;
    ptOutput->eState = ptIdentify->eState;
}

static foc_result_t identify_abort(foc_identify_t *ptIdentify,
                                   foc_identify_output_t *ptOutput)
{
    ptIdentify->eState = FOC_IDENTIFY_ABORTED;
    ptIdentify->bComplete = false;
    foc_experiment_EmergencyStop(&ptIdentify->tParams.tSafety);
    identify_output(ptIdentify, ptOutput);
    return FOC_RESULT_SAFETY;
}
#endif

foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_params_t *ptParams)
{
#if !FOC_ENABLE_EXPERIMENTAL_IDENTIFY
    (void)ptIdentify;
    (void)ptParams;
    return FOC_RESULT_DISABLED;
#else
    foc_result_t eResult;

    if (ptIdentify == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = foc_experiment_ValidateSafety(&ptParams->tSafety);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptParams->qHalfVoltage <= FOC_ZERO ||
        ptParams->qFullVoltage <= ptParams->qHalfVoltage ||
        ptParams->qFullVoltage > FOC_ONE ||
        (int)ptParams->eMode < (int)FOC_IDENTIFY_RS_LD_LQ ||
        (int)ptParams->eMode > (int)FOC_IDENTIFY_FLUX ||
        ptParams->qCurrentRiseRatio <= FOC_ZERO ||
        ptParams->qCurrentRiseRatio > FOC_ONE ||
        ptParams->qResetCurrentThreshold < FOC_ZERO ||
        ptParams->qInductanceTimeStep <= FOC_ZERO ||
        ptParams->wSettleSamples == 0U ||
        ptParams->wResetSamples == 0U ||
        ptParams->wMaximumRiseSamples == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptParams->eMode == FOC_IDENTIFY_FLUX &&
        (ptParams->qFluxVoltage <= FOC_ZERO ||
         ptParams->qFluxVoltage > FOC_ONE ||
         ptParams->qMinimumFluxSpeed <= FOC_ZERO ||
         ptParams->qKnownResistance < FOC_ZERO)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptIdentify, 0, sizeof(*ptIdentify));
    ptIdentify->tParams = *ptParams;
    return FOC_RESULT_OK;
#endif
}

foc_result_t foc_identify_Start(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard)
{
#if !FOC_ENABLE_EXPERIMENTAL_IDENTIFY
    (void)ptIdentify;
    (void)ptGuard;
    return FOC_RESULT_DISABLED;
#else
    if (ptIdentify == NULL || ptGuard == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptGuard->bMotorStopped ||
        !foc_experiment_IsSafe(&ptIdentify->tParams.tSafety, ptGuard)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptIdentify->eState =
        ptIdentify->tParams.eMode == FOC_IDENTIFY_FLUX ?
            FOC_IDENTIFY_FLUX_SETTLE : FOC_IDENTIFY_RS_HALF;
    ptIdentify->wStateSamples = 0U;
    ptIdentify->wTotalSamples = 0U;
    ptIdentify->qHalfCurrent = FOC_ZERO;
    ptIdentify->qFullCurrent = FOC_ZERO;
    ptIdentify->qRiseTime = FOC_ZERO;
    ptIdentify->qResistance = ptIdentify->tParams.qKnownResistance;
    ptIdentify->qInductanceD = FOC_ZERO;
    ptIdentify->qInductanceQ = FOC_ZERO;
    ptIdentify->qFlux = FOC_ZERO;
    ptIdentify->bComplete = false;
    return FOC_RESULT_OK;
#endif
}

#if FOC_ENABLE_EXPERIMENTAL_IDENTIFY
static foc_result_t identify_resistance(foc_identify_t *ptIdentify)
{
    return foc_div_checked(
        foc_sub_sat(ptIdentify->tParams.qFullVoltage,
                    ptIdentify->tParams.qHalfVoltage),
        foc_sub_sat(ptIdentify->qFullCurrent,
                    ptIdentify->qHalfCurrent),
        &ptIdentify->qResistance);
}
#endif

foc_result_t foc_identify_Step(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard,
    const foc_identify_input_t *ptInput,
    foc_identify_output_t *ptOutput)
{
#if !FOC_ENABLE_EXPERIMENTAL_IDENTIFY
    (void)ptIdentify;
    (void)ptGuard;
    (void)ptInput;
    if (ptOutput != NULL) {
        memset(ptOutput, 0, sizeof(*ptOutput));
        ptOutput->eState = FOC_IDENTIFY_IDLE;
    }
    return FOC_RESULT_DISABLED;
#else
    foc_scalar_t qThreshold;
    foc_scalar_t qNumerator;

    if (ptIdentify == NULL || ptGuard == NULL || ptInput == NULL ||
        ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_experiment_IsSafe(&ptIdentify->tParams.tSafety, ptGuard)) {
        return identify_abort(ptIdentify, ptOutput);
    }
    if (ptIdentify->eState == FOC_IDENTIFY_COMPLETE) {
        identify_output(ptIdentify, ptOutput);
        return FOC_RESULT_OK;
    }
    if (ptIdentify->eState == FOC_IDENTIFY_IDLE ||
        ptIdentify->eState == FOC_IDENTIFY_ABORTED) {
        identify_output(ptIdentify, ptOutput);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptIdentify->wTotalSamples++;
    if (ptIdentify->wTotalSamples >
        ptIdentify->tParams.tSafety.wTimeoutSamples) {
        return identify_abort(ptIdentify, ptOutput);
    }
    ptIdentify->wStateSamples++;
    switch (ptIdentify->eState) {
    case FOC_IDENTIFY_RS_HALF:
        if (ptIdentify->wStateSamples >=
            ptIdentify->tParams.wSettleSamples) {
            ptIdentify->qHalfCurrent = ptInput->qCurrentD;
            ptIdentify->wStateSamples = 0U;
            ptIdentify->eState = FOC_IDENTIFY_RS_FULL;
        }
        break;
    case FOC_IDENTIFY_RS_FULL:
        if (ptIdentify->wStateSamples >=
            ptIdentify->tParams.wSettleSamples) {
            ptIdentify->qFullCurrent = ptInput->qCurrentD;
            if (identify_resistance(ptIdentify) != FOC_RESULT_OK) {
                return identify_abort(ptIdentify, ptOutput);
            }
            ptIdentify->wStateSamples = 0U;
            ptIdentify->qRiseTime = FOC_ZERO;
            ptIdentify->eState = FOC_IDENTIFY_LD_ZERO;
        }
        break;
    case FOC_IDENTIFY_LD_ZERO:
        if (ptIdentify->wStateSamples >=
                ptIdentify->tParams.wResetSamples &&
            foc_abs(ptInput->qCurrentD) <=
                ptIdentify->tParams.qResetCurrentThreshold) {
            ptIdentify->wStateSamples = 0U;
            ptIdentify->eState = FOC_IDENTIFY_LD_RISE;
        } else if (ptIdentify->wStateSamples >=
                   ptIdentify->tParams.wMaximumRiseSamples) {
            return identify_abort(ptIdentify, ptOutput);
        }
        break;
    case FOC_IDENTIFY_LD_RISE:
        ptIdentify->qRiseTime = foc_add_sat(
            ptIdentify->qRiseTime,
            ptIdentify->tParams.qInductanceTimeStep);
        qThreshold = foc_mul_pu(
            ptIdentify->qFullCurrent,
            ptIdentify->tParams.qCurrentRiseRatio);
        if (foc_abs(ptInput->qCurrentD) >= foc_abs(qThreshold)) {
            ptIdentify->qInductanceD = foc_mul_wide(
                ptIdentify->qResistance, ptIdentify->qRiseTime);
            ptIdentify->wStateSamples = 0U;
            ptIdentify->qRiseTime = FOC_ZERO;
            ptIdentify->eState = FOC_IDENTIFY_LQ_ZERO;
        } else if (ptIdentify->wStateSamples >=
                   ptIdentify->tParams.wMaximumRiseSamples) {
            return identify_abort(ptIdentify, ptOutput);
        }
        break;
    case FOC_IDENTIFY_LQ_ZERO:
        if (ptIdentify->wStateSamples >=
                ptIdentify->tParams.wResetSamples &&
            foc_abs(ptInput->qCurrentQ) <=
                ptIdentify->tParams.qResetCurrentThreshold) {
            ptIdentify->wStateSamples = 0U;
            ptIdentify->eState = FOC_IDENTIFY_LQ_RISE;
        } else if (ptIdentify->wStateSamples >=
                   ptIdentify->tParams.wMaximumRiseSamples) {
            return identify_abort(ptIdentify, ptOutput);
        }
        break;
    case FOC_IDENTIFY_LQ_RISE:
        ptIdentify->qRiseTime = foc_add_sat(
            ptIdentify->qRiseTime,
            ptIdentify->tParams.qInductanceTimeStep);
        qThreshold = foc_mul_pu(
            ptIdentify->qFullCurrent,
            ptIdentify->tParams.qCurrentRiseRatio);
        if (foc_abs(ptInput->qCurrentQ) >= foc_abs(qThreshold)) {
            ptIdentify->qInductanceQ = foc_mul_wide(
                ptIdentify->qResistance, ptIdentify->qRiseTime);
            ptIdentify->eState = FOC_IDENTIFY_COMPLETE;
            ptIdentify->bComplete = true;
        } else if (ptIdentify->wStateSamples >=
                   ptIdentify->tParams.wMaximumRiseSamples) {
            return identify_abort(ptIdentify, ptOutput);
        }
        break;
    case FOC_IDENTIFY_FLUX_SETTLE:
        if (ptIdentify->wStateSamples >=
            ptIdentify->tParams.wSettleSamples) {
            if (foc_abs(ptInput->qElectricalSpeed) <
                ptIdentify->tParams.qMinimumFluxSpeed) {
                return identify_abort(ptIdentify, ptOutput);
            }
            qNumerator = foc_sub_sat(
                ptIdentify->tParams.qFluxVoltage,
                foc_mul_wide(ptIdentify->qResistance,
                             ptInput->qCurrentQ));
            if (foc_div_checked(qNumerator,
                                ptInput->qElectricalSpeed,
                                &ptIdentify->qFlux) != FOC_RESULT_OK) {
                return identify_abort(ptIdentify, ptOutput);
            }
            ptIdentify->eState = FOC_IDENTIFY_COMPLETE;
            ptIdentify->bComplete = true;
        }
        break;
    default:
        break;
    }
    identify_output(ptIdentify, ptOutput);
    return FOC_RESULT_OK;
#endif
}
