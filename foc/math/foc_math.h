/*******************************************************************************
 * @file    foc_math.h
 * @brief   FOC 数学函数接口声明
 ******************************************************************************/

#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include "foc_math_types.h"
#include "foc_math_port.h"

q_type foc_sin(q_type angle);
q_type foc_cos(q_type angle);
q_type foc_atan2(q_type y, q_type x);
q_type foc_sqrt(q_type x);

#endif /* __FOC_MATH_H__ */
