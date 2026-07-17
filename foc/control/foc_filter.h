/*******************************************************************************
 * @file    foc_filter.h
 * @brief   Multi-instance first-order and biquad filters
 ******************************************************************************/

#ifndef FOC_FILTER_H
#define FOC_FILTER_H

#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qAlpha;
    foc_scalar_t qState;
} foc_lpf1_t;

typedef struct {
    foc_gain_t tB0;
    foc_gain_t tB1;
    foc_gain_t tB2;
    foc_gain_t tA1;
    foc_gain_t tA2;
} foc_biquad_coeffs_t;

typedef struct {
    foc_biquad_coeffs_t tCoefficients;
    foc_scalar_t qState1;
    foc_scalar_t qState2;
} foc_biquad_t;

typedef enum {
    FOC_FILTER_BUTTERWORTH = 0,
    FOC_FILTER_CHEBYSHEV_I_0P1_DB,
    FOC_FILTER_BESSEL,
} foc_filter_response_t;

foc_result_t foc_lpf1_Init(foc_lpf1_t *ptFilter,
                           foc_scalar_t qAlpha,
                           foc_scalar_t qInitialValue);
void foc_lpf1_Reset(foc_lpf1_t *ptFilter, foc_scalar_t qValue);
foc_scalar_t foc_lpf1_Step(foc_lpf1_t *ptFilter,
                           foc_scalar_t qInput);

foc_result_t foc_biquad_Init(foc_biquad_t *ptFilter,
                             const foc_biquad_coeffs_t *ptCoefficients);
foc_result_t foc_biquad_LowPassInit(foc_biquad_t *ptFilter,
                                    foc_filter_response_t eResponse,
                                    uint32_t wSampleFrequencyHz,
                                    uint32_t wCutoffFrequencyHz);
void foc_biquad_Reset(foc_biquad_t *ptFilter);
foc_scalar_t foc_biquad_Step(foc_biquad_t *ptFilter,
                             foc_scalar_t qInput);

#endif /* FOC_FILTER_H */
