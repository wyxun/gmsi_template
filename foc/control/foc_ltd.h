/*******************************************************************************
 * @file    foc_ltd.h
 * @brief   Multi-instance linear tracking differentiator
 ******************************************************************************/

#ifndef FOC_LTD_H
#define FOC_LTD_H

#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qMaximumVelocity;
    foc_scalar_t qMaximumAcceleration;
} foc_ltd_params_t;

typedef struct {
    foc_ltd_params_t tParams;
    foc_scalar_t qPosition;
    foc_scalar_t qVelocity;
} foc_ltd_t;

foc_result_t foc_ltd_Init(foc_ltd_t *ptLtd,
                          const foc_ltd_params_t *ptParams,
                          foc_scalar_t qInitialPosition);
void foc_ltd_Reset(foc_ltd_t *ptLtd, foc_scalar_t qPosition);
foc_scalar_t foc_ltd_Step(foc_ltd_t *ptLtd, foc_scalar_t qTarget);

#endif /* FOC_LTD_H */
