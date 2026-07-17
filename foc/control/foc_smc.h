/*******************************************************************************
 * @file    foc_smc.h
 * @brief   Multi-instance sliding-mode controller
 ******************************************************************************/

#ifndef FOC_SMC_H
#define FOC_SMC_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tDerivative;
    foc_gain_t tSurface;
    foc_gain_t tDiscontinuous;
    foc_gain_t tReach;
    foc_gain_t tPlant;
    foc_gain_t tIntegrator;
    foc_scalar_t qOutputMinimum;
    foc_scalar_t qOutputMaximum;
} foc_smc_params_t;

typedef struct {
    foc_smc_params_t tParams;
    foc_scalar_t qPreviousError;
    foc_scalar_t qIntegrator;
} foc_smc_t;

foc_result_t foc_smc_Init(foc_smc_t *ptSmc,
                          const foc_smc_params_t *ptParams);
void foc_smc_Reset(foc_smc_t *ptSmc);
foc_scalar_t foc_smc_Step(foc_smc_t *ptSmc,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_SMC_H */
