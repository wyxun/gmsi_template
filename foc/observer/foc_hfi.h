/*******************************************************************************
 * @file    foc_hfi.h
 * @brief   High-frequency injection and synchronous response demodulation
 ******************************************************************************/

#ifndef FOC_HFI_H
#define FOC_HFI_H

#include <stdbool.h>

#include "foc_math.h"

typedef struct {
    foc_scalar_t qPhaseStep;
    foc_scalar_t qInjectionAmplitude;
    foc_scalar_t qHighPassAlpha;
    foc_scalar_t qDemodAlpha;
    foc_gain_t tDemodGain;
    foc_scalar_t qMinimumResponse;
} foc_hfi_params_t;

typedef struct {
    foc_scalar_t qInjectionD;
    foc_scalar_t qPositionError;
    foc_scalar_t qResponse;
    bool bValid;
} foc_hfi_output_t;

typedef struct {
    foc_hfi_params_t tParams;
    foc_angle_t tPhase;
    foc_scalar_t qPreviousCurrentD;
    foc_scalar_t qPreviousCarrier;
    foc_scalar_t qHighPass;
    foc_scalar_t qDemodulated;
    bool bHasPreviousCarrier;
} foc_hfi_t;

foc_result_t foc_hfi_Init(foc_hfi_t *ptHfi,
                          const foc_hfi_params_t *ptParams);
void foc_hfi_Reset(foc_hfi_t *ptHfi);
foc_result_t foc_hfi_Step(foc_hfi_t *ptHfi,
                          foc_scalar_t qCurrentD,
                          foc_hfi_output_t *ptOutput);

#endif /* FOC_HFI_H */
