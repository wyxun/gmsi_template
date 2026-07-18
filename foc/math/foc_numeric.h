/*******************************************************************************
 * @file    foc_numeric.h
 * @brief   Architecture-independent scalar backend for FOC algorithms
 ******************************************************************************/

#ifndef FOC_NUMERIC_H
#define FOC_NUMERIC_H

#include <stdbool.h>
#include <stdint.h>

#if defined(FOC_NUMERIC_FLOAT) && defined(FOC_NUMERIC_FIXED)
#error "Select only one FOC numeric backend"
#elif defined(FOC_NUMERIC_FLOAT)
typedef float foc_scalar_t;
#elif defined(FOC_NUMERIC_FIXED)
typedef int32_t foc_scalar_t;
#else
#error "Select FOC_NUMERIC_FLOAT or FOC_NUMERIC_FIXED"
#endif

typedef enum {
    FOC_RESULT_OK = 0,
    FOC_RESULT_NULL,
    FOC_RESULT_INVALID_ARGUMENT,
    FOC_RESULT_OUT_OF_RANGE,
    FOC_RESULT_DIVIDE_BY_ZERO,
    FOC_RESULT_DISABLED,
    FOC_RESULT_SAFETY,
    FOC_RESULT_BUSY,
} foc_result_t;

typedef struct {
    int16_t nInteger;
    foc_scalar_t qFraction;
} foc_gain_t;

#define FOC_Q_FRACTION_BITS 15
#define FOC_Q_SCALE         32768

#if defined(FOC_NUMERIC_FLOAT)
#define FOC_SCALAR(value) ((foc_scalar_t)(value))
#define FOC_ZERO          ((foc_scalar_t)0.0f)
#define FOC_HALF          ((foc_scalar_t)0.5f)
#define FOC_ONE           ((foc_scalar_t)1.0f)
#define FOC_NEG_ONE       ((foc_scalar_t)-1.0f)
#else
#define FOC_SCALAR(value)                                                   \
    ((foc_scalar_t)(((value) >= 0.0f)                                      \
                        ? ((value) * (float)FOC_Q_SCALE + 0.5f)            \
                        : ((value) * (float)FOC_Q_SCALE - 0.5f)))
#define FOC_ZERO          ((foc_scalar_t)0)
#define FOC_HALF          ((foc_scalar_t)(FOC_Q_SCALE / 2))
#define FOC_ONE           ((foc_scalar_t)FOC_Q_SCALE)
#define FOC_NEG_ONE       ((foc_scalar_t)-FOC_Q_SCALE)
#endif

foc_scalar_t foc_from_float(float fValue);
float foc_to_float(foc_scalar_t qValue);
foc_scalar_t foc_add_sat(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_sub_sat(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_mul_pu(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_mul_wide(foc_scalar_t qA, foc_scalar_t qB);
foc_result_t foc_div_checked(foc_scalar_t qNumerator,
                             foc_scalar_t qDenominator,
                             foc_scalar_t *pqResult);
foc_scalar_t foc_sat(foc_scalar_t qValue,
                     foc_scalar_t qMinimum,
                     foc_scalar_t qMaximum);
foc_scalar_t foc_abs(foc_scalar_t qValue);
foc_result_t foc_gain_Init(foc_gain_t *ptGain,
                           int16_t nInteger,
                           foc_scalar_t qFraction);
foc_result_t foc_gain_from_float(float fGain, foc_gain_t *ptGain);
foc_result_t foc_gain_from_scalar(foc_scalar_t qGain, foc_gain_t *ptGain);
bool foc_gain_IsValid(const foc_gain_t *ptGain);
foc_scalar_t foc_gain_apply(const foc_gain_t *ptGain,
                            foc_scalar_t qValue);

#endif /* FOC_NUMERIC_H */
