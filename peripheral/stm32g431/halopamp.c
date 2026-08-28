/**
 * @file  halopamp.c
 * @brief OPAMP1/2/3 — PGA x16, 3-shunt current amplification
 *
 * Mapping:
 *   OPAMP1: PA1(VINP) / PA2(VOUT) / PA3(VINM0) — U-phase
 *   OPAMP2: PA7(VINP) / PA6(VOUT) / PA5(VINM0) — V-phase
 *   OPAMP3: PB0(VINP) / PB1(VOUT) / PB2(VINM0) — W-phase (→ internal ADC2)
 */

#include "halopamp.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_bus.h"

static void opamp_gpio_init(void)
{
    /* OPAMP1: PA1/PA2/PA3 */
    /* OPAMP2: PA5/PA6/PA7 */
    /* OPAMP3: PB0/PB1/PB2 */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
             | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void halopamp_Init(void)
{
    LL_OPAMP_InitTypeDef init = {0};

    opamp_gpio_init();

    /* Common config */
    init.PowerMode        = LL_OPAMP_POWERMODE_NORMALSPEED;
    init.FunctionalMode   = LL_OPAMP_MODE_PGA_IO0_BIAS;
    init.InputNonInverting= LL_OPAMP_INPUT_NONINVERT_IO0;

    /* OPAMP1 — U-phase */
    LL_OPAMP_Init(OPAMP1, &init);
    LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputsMuxMode(OPAMP1, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP1, LL_OPAMP_INTERNAL_OUTPUT_DISABLED);
    LL_OPAMP_SetTrimmingMode(OPAMP1, LL_OPAMP_TRIMMING_FACTORY);
    LL_OPAMP_Enable(OPAMP1);

    /* OPAMP2 — V-phase (internal output to ADC2, 与 W 相 VOPAMP3 对称) */
    LL_OPAMP_Init(OPAMP2, &init);
    LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputsMuxMode(OPAMP2, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUTPUT_ENABLED);
    LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);
    LL_OPAMP_Enable(OPAMP2);

    /* OPAMP3 — W-phase (internal output to ADC2) */
    LL_OPAMP_Init(OPAMP3, &init);
    LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
    LL_OPAMP_SetInputsMuxMode(OPAMP3, LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUTPUT_ENABLED);
    LL_OPAMP_SetTrimmingMode(OPAMP3, LL_OPAMP_TRIMMING_FACTORY);
    LL_OPAMP_Enable(OPAMP3);
}
