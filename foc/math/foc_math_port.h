/*******************************************************************************
 * @file    foc_math_port.h
 * @brief   Compatibility arithmetic wrappers for the numeric backend
 ******************************************************************************/

#ifndef FOC_MATH_PORT_H
#define FOC_MATH_PORT_H

#include "foc_numeric.h"

/* New high-frequency code chooses foc_mul_pu() or foc_mul_wide() explicitly. */
#define M_MUL(a, b) foc_mul_wide((a), (b))
#define M_MAD(a, b, c) foc_add_sat(foc_mul_wide((a), (b)), (c))
#define M_MSB(a, b, c) foc_sub_sat((c), foc_mul_wide((a), (b)))
#define M_SAT(value, minimum, maximum) \
    foc_sat((value), (minimum), (maximum))

#endif /* FOC_MATH_PORT_H */
