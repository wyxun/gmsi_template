/*******************************************************************************
 * @file    foc_hall.h
 * @brief   Safe six-step Hall angle and speed observer
 ******************************************************************************/

#ifndef FOC_HALL_H
#define FOC_HALL_H

#include "foc_observer.h"

typedef struct {
    uint8_t achSectorByCode[8];
    uint16_t hwInvalidTimeout;
    foc_scalar_t qSpeedFilterAlpha;
} foc_hall_params_t;

typedef struct {
    foc_hall_params_t tParams;
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    foc_scalar_t qConfidence;
    uint16_t hwTicksSinceEdge;
    uint16_t hwInvalidSamples;
    uint8_t chPreviousSector;
    bool bInitialized;
    bool bValid;
} foc_hall_t;

void foc_hall_DefaultParams(foc_hall_params_t *ptParams);
foc_result_t foc_hall_Init(foc_hall_t *ptHall,
                           const foc_hall_params_t *ptParams);
void foc_hall_Reset(foc_hall_t *ptHall);
foc_result_t foc_hall_Step(foc_hall_t *ptHall,
                           uint8_t chHallCode,
                           foc_observer_output_t *ptOutput);
foc_observer_if_t foc_hall_ObserverInterface(foc_hall_t *ptHall);

#endif /* FOC_HALL_H */
