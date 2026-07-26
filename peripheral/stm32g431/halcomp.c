/**
 * @file  halcomp.c
 * @brief COMP1/2/4 — overcurrent trip, reference from DAC3_CH1/CH2
 *
 * COMP1: PA1(+) vs DAC3_CH1(−) — U-phase
 * COMP2: PA7(+) vs DAC3_CH2(−) — V-phase
 * COMP4: PB0(+) vs DAC3_CH2(−) — W-phase
 */

#include "halcomp.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_comp.h"

void halcomp_Init(void)
{
    LL_COMP_InitTypeDef init = {0};

    /* ---- COMP1 ---- */
    init.InputPlus            = LL_COMP_INPUT_PLUS_IO1;
    init.InputMinus           = LL_COMP_INPUT_MINUS_DAC3_CH1;
    init.InputHysteresis      = LL_COMP_HYSTERESIS_NONE;
    init.OutputPolarity       = LL_COMP_OUTPUTPOL_NONINVERTED;
    init.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
    LL_COMP_Init(COMP1, &init);
    /* Wait for voltage scaler stabilization */
    for (volatile uint32_t i = 0; i < SystemCoreClock / 1000U; i++) {}
    LL_COMP_Enable(COMP1);

    /* ---- COMP2: PA7(+) vs DAC3_CH2(-) ---- */
    init.InputPlus            = LL_COMP_INPUT_PLUS_IO2;
    init.InputMinus           = LL_COMP_INPUT_MINUS_DAC3_CH2;
    LL_COMP_Init(COMP2, &init);
    for (volatile uint32_t i = 0; i < SystemCoreClock / 1000U; i++) {}
    LL_COMP_Enable(COMP2);

    /* ---- COMP4: PB0(+) vs DAC3_CH2(-) ---- */
    init.InputPlus            = LL_COMP_INPUT_PLUS_IO1;
    init.InputMinus           = LL_COMP_INPUT_MINUS_DAC3_CH2;
    LL_COMP_Init(COMP4, &init);
    for (volatile uint32_t i = 0; i < SystemCoreClock / 1000U; i++) {}
    LL_COMP_Enable(COMP4);
}

uint32_t halcomp_GetOutput(uint32_t wIdx)
{
    switch (wIdx) {
    case HALCOMP_IDX_COMP1: return LL_COMP_ReadOutputLevel(COMP1);
    case HALCOMP_IDX_COMP2: return LL_COMP_ReadOutputLevel(COMP2);
    case HALCOMP_IDX_COMP4: return LL_COMP_ReadOutputLevel(COMP4);
    default:                return 0;
    }
}
