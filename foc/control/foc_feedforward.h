/*******************************************************************************
 * @file    foc_feedforward.h
 * @brief   PMSM current-loop decoupling feedforward
 ******************************************************************************/

#ifndef FOC_FEEDFORWARD_H
#define FOC_FEEDFORWARD_H

#include "foc_core.h"

typedef struct {
    foc_gain_t tLq;
    foc_gain_t tLd;
    foc_gain_t tFlux;
    foc_scalar_t qOutputLimit;
} foc_feedforward_params_t;

typedef struct {
    foc_feedforward_params_t tParams;
} foc_feedforward_t;

foc_result_t foc_feedforward_Init(
    foc_feedforward_t *ptFeedforward,
    const foc_feedforward_params_t *ptParams);
foc_result_t foc_feedforward_Pmsm(
    const foc_feedforward_t *ptFeedforward,
    foc_scalar_t qElectricalSpeed,
    foc_scalar_t qId,
    foc_scalar_t qIq,
    foc_dq_t *ptVoltage);

#endif /* FOC_FEEDFORWARD_H */
