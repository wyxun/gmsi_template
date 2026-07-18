/*******************************************************************************
 * @file    foc_observer_selector.h
 * @brief   Qualified and blended observer source switching
 ******************************************************************************/

#ifndef FOC_OBSERVER_SELECTOR_H
#define FOC_OBSERVER_SELECTOR_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qMinimumConfidence;
    foc_scalar_t qMinimumSpeed;
    foc_scalar_t qMaximumAngleError;
    uint16_t hwStableSamples;
    uint16_t hwBlendSamples;
} foc_observer_selector_params_t;

typedef struct {
    foc_observer_selector_params_t tParams;
    const foc_position_source_if_t *ptActive;
    const foc_position_source_if_t *ptTarget;
    foc_position_output_t tActiveOutput;
    foc_position_output_t tTargetOutput;
    foc_scalar_t qBlendProgress;
    foc_scalar_t qBlendIncrement;
    uint16_t hwStableCount;
    uint16_t hwBlendCount;
    bool bBlending;
} foc_observer_selector_t;

foc_result_t foc_observer_selector_Init(
    foc_observer_selector_t *ptSelector,
    const foc_observer_selector_params_t *ptParams,
    const foc_position_source_if_t *ptInitial);
foc_result_t foc_observer_selector_Request(
    foc_observer_selector_t *ptSelector,
    const foc_position_source_if_t *ptTarget);
void foc_observer_selector_Cancel(foc_observer_selector_t *ptSelector);
foc_result_t foc_observer_selector_Step(
    foc_observer_selector_t *ptSelector,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput);

#endif /* FOC_OBSERVER_SELECTOR_H */
