/*******************************************************************************
 * @file    foc_ladrc.h
 * @brief   Multi-instance discrete linear active disturbance rejection control
 ******************************************************************************/

#ifndef FOC_LADRC_H
#define FOC_LADRC_H

#include "foc_controller.h"

typedef struct {
    foc_gain_t tTrackingPosition;
    foc_gain_t tTrackingVelocity;
    foc_gain_t tTrackingIntegrator;
    foc_gain_t tObserverBeta1;
    foc_gain_t tObserverBeta2;
    foc_gain_t tObserverBeta3;
    foc_gain_t tPlantGain;
    foc_gain_t tKp;
    foc_gain_t tKd;
    foc_gain_t tPlantInverse;
    foc_scalar_t qOutputMinimum;
    foc_scalar_t qOutputMaximum;
} foc_ladrc_params_t;

typedef struct {
    foc_ladrc_params_t tParams;
    foc_scalar_t qTrackingPosition;
    foc_scalar_t qTrackingVelocity;
    foc_scalar_t qObserverPosition;
    foc_scalar_t qObserverVelocity;
    foc_scalar_t qObserverDisturbance;
    foc_scalar_t qOutput;
} foc_ladrc_t;

foc_result_t foc_ladrc_Init(foc_ladrc_t *ptLadrc,
                            const foc_ladrc_params_t *ptParams);
void foc_ladrc_Reset(foc_ladrc_t *ptLadrc);
foc_scalar_t foc_ladrc_Step(foc_ladrc_t *ptLadrc,
                            foc_scalar_t qReference,
                            foc_scalar_t qFeedback);
foc_controller_if_t foc_ladrc_ControllerInterface(foc_ladrc_t *ptLadrc);

#endif /* FOC_LADRC_H */
