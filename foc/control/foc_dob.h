/*******************************************************************************
 * @file    foc_dob.h
 * @brief   Multi-instance normalized disturbance observer
 ******************************************************************************/

#ifndef FOC_DOB_H
#define FOC_DOB_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tTorqueGain;
    foc_gain_t tModelGain;
    foc_gain_t tK1;
    foc_gain_t tK2Ts;
    foc_gain_t tBoundaryInverse;
    foc_scalar_t qDisturbanceMinimum;
    foc_scalar_t qDisturbanceMaximum;
    foc_scalar_t qSpeedMinimum;
    foc_scalar_t qSpeedMaximum;
} foc_dob_params_t;

typedef struct {
    foc_scalar_t qEstimatedSpeed;
    foc_scalar_t qDisturbance;
} foc_dob_output_t;

typedef struct {
    foc_dob_params_t tParams;
    foc_scalar_t qEstimatedSpeed;
    foc_scalar_t qDisturbance;
} foc_dob_t;

foc_result_t foc_dob_Init(foc_dob_t *ptDob,
                          const foc_dob_params_t *ptParams);
void foc_dob_Reset(foc_dob_t *ptDob);
foc_result_t foc_dob_Step(foc_dob_t *ptDob,
                          foc_scalar_t qCurrentQ,
                          foc_scalar_t qMeasuredSpeed,
                          foc_dob_output_t *ptOutput);

#endif /* FOC_DOB_H */
