/*******************************************************************************
 * @file    foc_observer.h
 * @brief   Common multi-instance position and speed observer interface
 ******************************************************************************/

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include "foc_core.h"

typedef struct {
    foc_ab_t tCurrent;
    foc_ab_t tVoltage;
    foc_scalar_t qEstimatedSpeed;
    uint8_t chHallCode;
} foc_observer_input_t;

typedef struct {
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    foc_scalar_t qConfidence;
    bool bValid;
} foc_observer_output_t;

typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_observer_input_t *ptInput,
                           foc_observer_output_t *ptOutput);
} foc_observer_if_t;

bool foc_observer_IsValid(const foc_observer_if_t *ptObserver);
void foc_observer_Reset(const foc_observer_if_t *ptObserver);
foc_result_t foc_observer_Step(const foc_observer_if_t *ptObserver,
                               const foc_observer_input_t *ptInput,
                               foc_observer_output_t *ptOutput);

#endif /* FOC_OBSERVER_H */
