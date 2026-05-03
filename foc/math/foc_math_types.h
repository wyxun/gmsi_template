/*******************************************************************************
 * @file    foc_math_types.h
 * @brief   FOC 数值类型定义
 ******************************************************************************/

#ifndef __FOC_MATH_TYPES_H__
#define __FOC_MATH_TYPES_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "foc_config.h"

#if FOC_USE_FPU_HARDWARE

    typedef float   q_type;

    #define _Q(v)       ((q_type)(v))
    #define Q_ONE       (1.0f)
    #define Q_ZERO      (0.0f)
    #define Q_HALF      (0.5f)

    #define _D(v)       ((double)(v))
    #define _D_ANG(v)   ((double)(v) * 180.0 / 3.1415926535)

#else

    typedef int32_t q_type;

    #define Q_SHIFT     15
    #define _Q(v)       ((q_type)((v) * (1 << Q_SHIFT)))
    #define Q_ONE       ((q_type)(1 << Q_SHIFT))
    #define Q_ZERO      ((q_type)0)
    #define Q_HALF      ((q_type)(1 << (Q_SHIFT - 1)))

    #define _D(v)       ((double)(v) / (double)(1 << Q_SHIFT))
    #define _D_ANG(v)   ((double)(v) * 180.0 / (double)(1 << Q_SHIFT))

#endif /* FOC_USE_FPU_HARDWARE */

#endif /* __FOC_MATH_TYPES_H__ */
