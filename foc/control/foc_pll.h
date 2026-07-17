/*******************************************************************************
 * @file    foc_pll.h
 * @brief   Multi-instance normalized-angle phase-locked loop
 ******************************************************************************/

#ifndef FOC_PLL_H
#define FOC_PLL_H

#include "foc_angle.h"
#include "foc_pid.h"

typedef struct {
    foc_gain_t tKp;
    foc_gain_t tKi;
    foc_scalar_t qMaximumSpeed;
    foc_scalar_t qLockError;
    foc_scalar_t qUnlockError;
    uint16_t hwLockSamples;
} foc_pll_params_t;

typedef struct {
    foc_pll_params_t tParams;
    foc_pid_t tLoopFilter;
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    uint16_t hwLockCounter;
    bool bIsLocked;
} foc_pll_t;

foc_result_t foc_pll_Init(foc_pll_t *ptPll,
                          const foc_pll_params_t *ptParams);
void foc_pll_Reset(foc_pll_t *ptPll, foc_angle_t tInitialAngle);
foc_result_t foc_pll_Step(foc_pll_t *ptPll,
                          foc_angle_t tMeasuredAngle);

#endif /* FOC_PLL_H */
