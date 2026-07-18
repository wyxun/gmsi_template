/*******************************************************************************
 * @file    foc_observer_selector.c
 * @brief   Observer qualification, cancellation, and bumpless blending
 ******************************************************************************/

#include "foc_observer_selector.h"

#include <stddef.h>
#include <string.h>

static foc_scalar_t selector_reciprocal(uint16_t hwValue)
{
#if defined(FOC_NUMERIC_FLOAT)
    return FOC_ONE / (foc_scalar_t)hwValue;
#else
    return (foc_scalar_t)(FOC_Q_SCALE / (int32_t)hwValue);
#endif
}

static bool selector_target_is_qualified(
    const foc_observer_selector_t *ptSelector)
{
    foc_scalar_t qAngleError = foc_abs(foc_angle_diff(
        ptSelector->tTargetOutput.tElectricalAngle,
        ptSelector->tActiveOutput.tElectricalAngle));
    const foc_position_valid_flag_e eRequired =
        FOC_POSITION_VALID_ELECTRICAL_ANGLE |
        FOC_POSITION_VALID_ELECTRICAL_SPEED;
    return ptSelector->tTargetOutput.wFaults == 0U &&
           (ptSelector->tTargetOutput.eValidFlags & eRequired) ==
               eRequired &&
           ptSelector->tTargetOutput.qConfidence >=
               ptSelector->tParams.qMinimumConfidence &&
           foc_abs(ptSelector->tTargetOutput.qElectricalSpeed) >=
               ptSelector->tParams.qMinimumSpeed &&
           qAngleError <= ptSelector->tParams.qMaximumAngleError;
}

foc_result_t foc_observer_selector_Init(
    foc_observer_selector_t *ptSelector,
    const foc_observer_selector_params_t *ptParams,
    const foc_position_source_if_t *ptInitial)
{
    if (ptSelector == NULL || ptParams == NULL ||
        !foc_position_source_IsValid(ptInitial)) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qMinimumConfidence < FOC_ZERO ||
        ptParams->qMinimumConfidence > FOC_ONE ||
        ptParams->qMinimumSpeed < FOC_ZERO ||
        ptParams->qMaximumAngleError <= FOC_ZERO ||
        ptParams->qMaximumAngleError > FOC_HALF ||
        ptParams->hwStableSamples == 0U ||
        ptParams->hwBlendSamples == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptSelector, 0, sizeof(*ptSelector));
    ptSelector->tParams = *ptParams;
    ptSelector->ptActive = ptInitial;
    ptSelector->qBlendIncrement =
        selector_reciprocal(ptParams->hwBlendSamples);
    foc_position_source_Reset(ptInitial);
    return FOC_RESULT_OK;
}

foc_result_t foc_observer_selector_Request(
    foc_observer_selector_t *ptSelector,
    const foc_position_source_if_t *ptTarget)
{
    if (ptSelector == NULL || !foc_position_source_IsValid(ptTarget)) {
        return FOC_RESULT_NULL;
    }
    if (ptTarget == ptSelector->ptActive) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptSelector->ptTarget = ptTarget;
    ptSelector->hwStableCount = 0U;
    ptSelector->hwBlendCount = 0U;
    ptSelector->qBlendProgress = FOC_ZERO;
    ptSelector->bBlending = false;
    foc_position_source_Reset(ptTarget);
    return FOC_RESULT_OK;
}

void foc_observer_selector_Cancel(foc_observer_selector_t *ptSelector)
{
    if (ptSelector != NULL) {
        ptSelector->ptTarget = NULL;
        ptSelector->hwStableCount = 0U;
        ptSelector->hwBlendCount = 0U;
        ptSelector->qBlendProgress = FOC_ZERO;
        ptSelector->bBlending = false;
    }
}

foc_result_t foc_observer_selector_Step(
    foc_observer_selector_t *ptSelector,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput)
{
    foc_result_t eResult;

    if (ptSelector == NULL || ptInput == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = foc_position_source_Step(ptSelector->ptActive, ptInput,
                                &ptSelector->tActiveOutput);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptSelector->ptTarget == NULL) {
        *ptOutput = ptSelector->tActiveOutput;
        return FOC_RESULT_OK;
    }
    eResult = foc_position_source_Step(ptSelector->ptTarget, ptInput,
                                &ptSelector->tTargetOutput);
    if (eResult != FOC_RESULT_OK) {
        foc_observer_selector_Cancel(ptSelector);
        *ptOutput = ptSelector->tActiveOutput;
        return FOC_RESULT_OK;
    }
    if (!selector_target_is_qualified(ptSelector)) {
        if (ptSelector->bBlending) {
            foc_observer_selector_Cancel(ptSelector);
        } else {
            ptSelector->hwStableCount = 0U;
        }
        *ptOutput = ptSelector->tActiveOutput;
        return FOC_RESULT_OK;
    }
    if (!ptSelector->bBlending) {
        ptSelector->hwStableCount++;
        if (ptSelector->hwStableCount <
            ptSelector->tParams.hwStableSamples) {
            *ptOutput = ptSelector->tActiveOutput;
            return FOC_RESULT_OK;
        }
        ptSelector->bBlending = true;
    }
    ptSelector->hwBlendCount++;
    ptSelector->qBlendProgress = foc_sat(
        foc_add_sat(ptSelector->qBlendProgress,
                    ptSelector->qBlendIncrement),
        FOC_ZERO, FOC_ONE);
    eResult = foc_position_Blend(&ptSelector->tActiveOutput,
                                 &ptSelector->tTargetOutput,
                                 ptSelector->qBlendProgress, ptOutput);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (ptSelector->hwBlendCount >= ptSelector->tParams.hwBlendSamples) {
        ptSelector->ptActive = ptSelector->ptTarget;
        ptSelector->tActiveOutput = ptSelector->tTargetOutput;
        foc_observer_selector_Cancel(ptSelector);
        *ptOutput = ptSelector->tActiveOutput;
    }
    return FOC_RESULT_OK;
}
