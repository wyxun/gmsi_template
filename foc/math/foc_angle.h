/*******************************************************************************
 * @file    foc_angle.h
 * @brief   Normalized electrical-angle API, one turn equals 1.0
 ******************************************************************************/

#ifndef FOC_ANGLE_H
#define FOC_ANGLE_H

#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qTurns;
} foc_angle_t;

foc_angle_t foc_angle_from_turns(float fTurns);
foc_angle_t foc_angle_from_scalar(foc_scalar_t qTurns);
float foc_angle_to_turns(foc_angle_t tAngle);
foc_angle_t foc_angle_wrap(foc_angle_t tAngle);
foc_scalar_t foc_angle_diff(foc_angle_t tTarget,
                            foc_angle_t tActual);
foc_scalar_t foc_angle_sin(foc_angle_t tAngle);
foc_scalar_t foc_angle_cos(foc_angle_t tAngle);
foc_angle_t foc_angle_atan2(foc_scalar_t qY,
                            foc_scalar_t qX);

#endif /* FOC_ANGLE_H */
