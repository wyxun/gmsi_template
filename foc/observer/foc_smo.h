/*******************************************************************************
 * @file    foc_smo.h
 * @brief   Normalized stationary-frame sliding-mode observer
 ******************************************************************************/

#ifndef FOC_SMO_H
#define FOC_SMO_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qModelGain;
    foc_scalar_t qResistance;
    foc_scalar_t qSlidingGain;
    foc_scalar_t qBoundaryInverse;
    foc_scalar_t qEmfFilterAlpha;
    foc_scalar_t qMinimumBemf;
} foc_smo_params_t;

typedef struct {
    foc_smo_params_t tParams;
    foc_gain_t tBoundaryInverse;
    foc_ab_t tEstimatedCurrent;
    foc_ab_t tBemf;
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    bool bHasAngle;
} foc_smo_t;

foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const foc_smo_params_t *ptParams);
void foc_smo_Reset(foc_smo_t *ptSmo);
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_position_input_t *ptInput,
                          foc_position_output_t *ptOutput);
foc_position_source_if_t foc_smo_PositionSourceInterface(foc_smo_t *ptSmo);

#endif /* FOC_SMO_H */
