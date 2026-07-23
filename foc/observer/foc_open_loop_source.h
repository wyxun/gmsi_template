/*******************************************************************************
 * @file    foc_open_loop_source.h
 * @brief   Open-loop angle and speed generator implementing foc_position_source_if_t
 ******************************************************************************/

#ifndef FOC_OPEN_LOOP_SOURCE_H
#define FOC_OPEN_LOOP_SOURCE_H

#include "foc_angle.h"
#include "foc_numeric.h"
#include "motor_position.h"

typedef struct {
    foc_angle_t  tAngle;
    foc_scalar_t qSpeed;
    foc_scalar_t qTargetSpeed;
    foc_scalar_t qAcceleration;
} foc_open_loop_source_t;

foc_result_t foc_open_loop_source_Init(foc_open_loop_source_t *ptSource);

foc_result_t foc_open_loop_source_SetSpeed(foc_open_loop_source_t *ptSource,
                                            foc_scalar_t qSpeed);

foc_result_t foc_open_loop_source_SetTargetSpeed(foc_open_loop_source_t *ptSource,
                                                  foc_scalar_t qTargetSpeed);

foc_result_t foc_open_loop_source_SetAcceleration(foc_open_loop_source_t *ptSource,
                                                   foc_scalar_t qAcceleration);

foc_result_t foc_open_loop_source_SetAngle(foc_open_loop_source_t *ptSource,
                                            foc_angle_t tAngle);

foc_position_source_if_t foc_open_loop_source_GetInterface(foc_open_loop_source_t *ptSource);

#endif /* FOC_OPEN_LOOP_SOURCE_H */
