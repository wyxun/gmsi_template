/*******************************************************************************
 * @file    foc_pid.h
 * @brief   Multi-instance PID controller with conditional anti-windup
 ******************************************************************************/

#ifndef FOC_PID_H
#define FOC_PID_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tKp;
    foc_gain_t tKiTs;
    foc_gain_t tKdOverTs;
    foc_scalar_t qOutputMinimum;
    foc_scalar_t qOutputMaximum;
    foc_scalar_t qIntegratorMinimum;
    foc_scalar_t qIntegratorMaximum;
} foc_pid_params_t;

typedef struct {
    foc_pid_params_t tParams;
    foc_scalar_t qIntegrator;
    foc_scalar_t qPreviousError;
} foc_pid_t;

foc_result_t foc_pid_Init(foc_pid_t *ptPid,
                          const foc_pid_params_t *ptParams);
void foc_pid_Reset(foc_pid_t *ptPid);
foc_scalar_t foc_pid_Step(foc_pid_t *ptPid,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_PID_H */
