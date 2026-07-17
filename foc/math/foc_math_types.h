/*******************************************************************************
 * @file    foc_math_types.h
 * @brief   Compatibility aliases for the unified FOC numeric backend
 ******************************************************************************/

#ifndef FOC_MATH_TYPES_H
#define FOC_MATH_TYPES_H

#include "foc_numeric.h"

typedef foc_scalar_t q_type;

#define Q_SHIFT FOC_Q_FRACTION_BITS
#define _Q(value) FOC_SCALAR(value)
#define Q_ZERO FOC_ZERO
#define Q_HALF FOC_HALF
#define Q_ONE FOC_ONE

#define _D(value) ((double)foc_to_float(value))
#define _D_ANG(value) ((double)foc_to_float(value) * 360.0)

#endif /* FOC_MATH_TYPES_H */
