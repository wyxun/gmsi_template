/*******************************************************************************
 * @file    foc_math_port.h
 * @brief   平台运算宏 — 硬件加速隔离层
 ******************************************************************************/

#ifndef __FOC_MATH_PORT_H__
#define __FOC_MATH_PORT_H__

#include "foc_math_types.h"

#if FOC_USE_FPU_HARDWARE

    #define M_MUL(a, b)         ((a) * (b))
    #define M_MAD(a, b, c)      ((a) * (b) + (c))
    #define M_MSB(a, b, c)      ((c) - (a) * (b))
    #define M_SAT(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#else

    #define M_MUL(a, b)         ((q_type)(((int64_t)(a) * (b)) >> Q_SHIFT))
    #define M_MAD(a, b, c)      ((q_type)((((int64_t)(a) * (b)) >> Q_SHIFT) + (c)))
    #define M_MSB(a, b, c)      ((q_type)((c) - (((int64_t)(a) * (b)) >> Q_SHIFT)))
    #define M_SAT(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#endif /* FOC_USE_FPU_HARDWARE */

#endif /* __FOC_MATH_PORT_H__ */
