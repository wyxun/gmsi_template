/*******************************************************************************
 * @file    foc_modulation.h
 * @brief   Architecture-independent three-phase modulation
 ******************************************************************************/

#ifndef FOC_MODULATION_H
#define FOC_MODULATION_H

#include "foc_core.h"

typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_duty_abc_t;

foc_result_t foc_svpwm(const foc_ab_t *ptVoltage,
                       foc_duty_abc_t *ptDuty);
foc_result_t foc_spwm(const foc_ab_t *ptVoltage,
                      foc_duty_abc_t *ptDuty);
foc_result_t foc_third_harmonic_spwm(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptDuty);

#endif /* FOC_MODULATION_H */
