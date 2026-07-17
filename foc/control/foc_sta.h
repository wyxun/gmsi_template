/*******************************************************************************
 * @file    foc_sta.h
 * @brief   Multi-instance super-twisting controller
 ******************************************************************************/

#ifndef FOC_STA_H
#define FOC_STA_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tK1;
    foc_gain_t tK2Ts;
    foc_gain_t tBoundaryInverse;
    foc_scalar_t qIntegratorMinimum;
    foc_scalar_t qIntegratorMaximum;
    foc_scalar_t qOutputMinimum;
    foc_scalar_t qOutputMaximum;
} foc_sta_params_t;

typedef struct {
    foc_sta_params_t tParams;
    foc_scalar_t qIntegrator;
} foc_sta_t;

foc_result_t foc_sta_Init(foc_sta_t *ptSta,
                          const foc_sta_params_t *ptParams);
void foc_sta_Reset(foc_sta_t *ptSta);
foc_scalar_t foc_sta_Step(foc_sta_t *ptSta,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_STA_H */
