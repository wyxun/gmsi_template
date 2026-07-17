/*******************************************************************************
 * @file    foc_modulation.c
 * @brief   Architecture-independent three-phase modulation
 ******************************************************************************/

#include "foc_modulation.h"

#include <stddef.h>

static void modulation_phase_voltage(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptPhase)
{
    const foc_scalar_t qHalf = FOC_HALF;
    const foc_scalar_t qSqrtThreeHalf = FOC_SCALAR(0.8660254038f);
    foc_scalar_t qHalfAlpha = foc_mul_pu(ptVoltage->qAlpha, qHalf);
    foc_scalar_t qBetaTerm = foc_mul_pu(ptVoltage->qBeta,
                                        qSqrtThreeHalf);

    ptPhase->qU = ptVoltage->qAlpha;
    ptPhase->qV = foc_sub_sat(qBetaTerm, qHalfAlpha);
    ptPhase->qW = foc_sub_sat(-qBetaTerm, qHalfAlpha);
}

static void modulation_clamp_duty(foc_duty_abc_t *ptDuty)
{
    ptDuty->qU = foc_sat(ptDuty->qU, FOC_ZERO, FOC_ONE);
    ptDuty->qV = foc_sat(ptDuty->qV, FOC_ZERO, FOC_ONE);
    ptDuty->qW = foc_sat(ptDuty->qW, FOC_ZERO, FOC_ONE);
}

static foc_result_t modulation_common_mode(const foc_ab_t *ptVoltage,
                                           foc_duty_abc_t *ptDuty)
{
    foc_duty_abc_t tPhase;
    foc_scalar_t qMaximum;
    foc_scalar_t qMinimum;
    foc_scalar_t qOffset;

    if (ptVoltage == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    modulation_phase_voltage(ptVoltage, &tPhase);
    qMaximum = tPhase.qU;
    qMinimum = tPhase.qU;
    if (tPhase.qV > qMaximum) qMaximum = tPhase.qV;
    if (tPhase.qW > qMaximum) qMaximum = tPhase.qW;
    if (tPhase.qV < qMinimum) qMinimum = tPhase.qV;
    if (tPhase.qW < qMinimum) qMinimum = tPhase.qW;

    qOffset = -foc_mul_pu(foc_add_sat(qMaximum, qMinimum), FOC_HALF);
    ptDuty->qU = foc_add_sat(foc_add_sat(tPhase.qU, qOffset), FOC_HALF);
    ptDuty->qV = foc_add_sat(foc_add_sat(tPhase.qV, qOffset), FOC_HALF);
    ptDuty->qW = foc_add_sat(foc_add_sat(tPhase.qW, qOffset), FOC_HALF);
    modulation_clamp_duty(ptDuty);
    return FOC_RESULT_OK;
}

foc_result_t foc_svpwm(const foc_ab_t *ptVoltage,
                       foc_duty_abc_t *ptDuty)
{
    return modulation_common_mode(ptVoltage, ptDuty);
}

foc_result_t foc_spwm(const foc_ab_t *ptVoltage,
                      foc_duty_abc_t *ptDuty)
{
    foc_duty_abc_t tPhase;

    if (ptVoltage == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    modulation_phase_voltage(ptVoltage, &tPhase);
    ptDuty->qU = foc_add_sat(tPhase.qU, FOC_HALF);
    ptDuty->qV = foc_add_sat(tPhase.qV, FOC_HALF);
    ptDuty->qW = foc_add_sat(tPhase.qW, FOC_HALF);
    modulation_clamp_duty(ptDuty);
    return FOC_RESULT_OK;
}

foc_result_t foc_third_harmonic_spwm(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptDuty)
{
    /* Min/max common-mode injection is equivalent to optimal third harmonic. */
    return modulation_common_mode(ptVoltage, ptDuty);
}
