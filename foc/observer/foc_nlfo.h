/*******************************************************************************
 * @file    foc_nlfo.h
 * @brief   Normalized nonlinear flux observer
 ******************************************************************************/

#ifndef FOC_NLFO_H
#define FOC_NLFO_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qIntegratorGain;
    foc_scalar_t qResistance;
    foc_scalar_t qAverageInductance;
    foc_scalar_t qFlux;
    foc_scalar_t qCorrectionGain;
    foc_scalar_t qMinimumFluxRatio;
} foc_nlfo_params_t;

typedef struct {
    foc_nlfo_params_t tParams;
    foc_gain_t tFluxInverse;
    foc_scalar_t qFluxSquared;
    foc_ab_t tStatorFlux;
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    bool bHasAngle;
} foc_nlfo_t;

foc_result_t foc_nlfo_Init(foc_nlfo_t *ptNlfo,
                           const foc_nlfo_params_t *ptParams);
void foc_nlfo_Reset(foc_nlfo_t *ptNlfo);
foc_result_t foc_nlfo_Step(foc_nlfo_t *ptNlfo,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
foc_position_source_if_t foc_nlfo_PositionSourceInterface(foc_nlfo_t *ptNlfo);

#endif /* FOC_NLFO_H */
