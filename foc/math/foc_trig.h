#ifndef FOC_TRIG_H
#define FOC_TRIG_H

#include "foc_numeric.h"
#include "foc_angle.h"

#define FOC_TRIG_BACKEND_LIBM    0
#define FOC_TRIG_BACKEND_LUT     1
#define FOC_TRIG_BACKEND_CORDIC  2

/* 依据 target.mk 或 外部 Makefile 决定默认后端，host 默认 LUT */
#ifndef FOC_TRIG_BACKEND
#define FOC_TRIG_BACKEND FOC_TRIG_BACKEND_LUT
#endif

typedef struct {
    void (*fnSinCosTurns)(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
    foc_scalar_t (*fnAtan2Turns)(foc_scalar_t qY, foc_scalar_t qX);
} foc_trig_backend_t;

#if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
extern const float s_afSinQuarter[513];
float lut_sin_turns(float fTurns);
#endif

#endif /* FOC_TRIG_H */
