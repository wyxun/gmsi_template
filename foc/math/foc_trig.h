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
    void (*fnSinCosBam32)(uint32_t wBam32, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
    foc_angle_t (*fnAtan2)(foc_scalar_t qY, foc_scalar_t qX);
} foc_trig_backend_t;

#if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
extern const float s_afSinQuarter[513];
float lut_sin_turns(float fTurns);
foc_scalar_t lut_sin_bam32(uint32_t wBam32);
void lut_sincos_bam32(uint32_t wBam32, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
#endif

#endif /* FOC_TRIG_H */
