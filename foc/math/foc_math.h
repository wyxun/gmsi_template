/*******************************************************************************
 * @file    foc_math.h
 * @brief   Compatibility math functions using normalized-turn angles
 ******************************************************************************/

#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_angle.h"
#include "foc_math_types.h"

q_type foc_sin(q_type qTurns);
q_type foc_cos(q_type qTurns);
q_type foc_atan2(q_type qY, q_type qX);
q_type foc_sqrt(q_type qValue);

#endif /* FOC_MATH_H */
