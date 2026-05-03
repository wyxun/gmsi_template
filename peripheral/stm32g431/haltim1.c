/**
 * @file  haltim1.c
 * @brief TIM1 initialization — 3-phase PWM for FOC
 *
 * Pin mapping:
 *   PA8  — TIM1_CH1  (AF6) — U high-side
 *   PC13 — TIM1_CH1N (AF4) — U low-side
 *   PA9  — TIM1_CH2  (AF6) — V high-side
 *   PA12 — TIM1_CH2N (AF6) — V low-side
 *   PA10 — TIM1_CH3  (AF6) — W high-side
 *   PB15 — TIM1_CH3N (AF4) — W low-side
 *   CH4 = PWM2 mode → TRGO = OC4REF → ADC injected trigger
 *
 * PWM: 20 kHz center-aligned, dead-time, complementary outputs.
 * Break inputs from COMP1/2/4 for overcurrent protection.
 */

#include "haltim1.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"

/* 170 MHz / 170 = 1 MHz timer clock → 1 MHz / 50 = 20 kHz PWM */
#define TIM1_PRESCALER      169U
#define TIM1_PERIOD         1000U   /* ARR → 20 kHz center-aligned */
#define TIM1_DEAD_TIME      50U     /* ~500 ns @ 1 MHz timer clock */
#define TIM1_CH4_CCR        ((TIM1_PERIOD / 2U) - 10U) /* trigger just before center */

static uint32_t s_wDutyU, s_wDutyV, s_wDutyW; /* cache for SetDuty */

void haltim1_Init(void)
{
    /* ---- Clock enable ---- */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /* ---- GPIO: PWM outputs ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;

    /* CH1 (PA8), CH2 (PA9), CH3 (PA10), CH2N (PA12) */
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Pin = GPIO_PIN_13;             /* CH1N (PC13) */
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Pin = GPIO_PIN_15;             /* CH3N (PB15) */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ---- TIM1 base ---- */
    LL_TIM_SetPrescaler(TIM1, TIM1_PRESCALER);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_DOWN);
    LL_TIM_SetAutoReload(TIM1, TIM1_PERIOD);
    LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV2);
    LL_TIM_SetRepetitionCounter(TIM1, 1);
    LL_TIM_DisableARRPreload(TIM1);

    /* ---- CH1, CH2, CH3: PWM1, complementary ---- */
    LL_TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = LL_TIM_OCMODE_PWM1;
    oc.OCState      = LL_TIM_OCSTATE_DISABLE;
    oc.OCNState     = LL_TIM_OCSTATE_DISABLE;
    oc.CompareValue = TIM1_PERIOD / 2;
    oc.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    oc.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    oc.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc);

    /* ---- CH4: PWM2 for ADC trigger ---- */
    oc.OCMode       = LL_TIM_OCMODE_PWM2;
    oc.OCState      = LL_TIM_OCSTATE_DISABLE;
    oc.CompareValue = TIM1_CH4_CCR;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &oc);

    /* TRGO2 = OC4REF (ADC injected trigger) */
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);

    /* ---- Break inputs from COMP1/2/4 ---- */
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP1, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP1);
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP2, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP2);
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP4, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP4);

    /* ---- BDTR ---- */
    LL_TIM_OC_SetDeadTime(TIM1, TIM1_DEAD_TIME);
    LL_TIM_ConfigBRK(TIM1, LL_TIM_BREAK_POLARITY_HIGH, LL_TIM_BREAK_FILTER_FDIV1_N8, LL_TIM_BREAK_AFMODE_INPUT);
    LL_TIM_EnableMasterSlaveMode(TIM1);

    /* Start counter (outputs disabled until haltim1_Start) */
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_EnableCounter(TIM1);
}

void haltim1_SetDuty(float fU, float fV, float fW)
{
    /* Clamp and scale float [0..1] to [0..TIM1_PERIOD] */
    if (fU < 0.0f) fU = 0.0f; else if (fU > 1.0f) fU = 1.0f;
    if (fV < 0.0f) fV = 0.0f; else if (fV > 1.0f) fV = 1.0f;
    if (fW < 0.0f) fW = 0.0f; else if (fW > 1.0f) fW = 1.0f;

    s_wDutyU = (uint32_t)(fU * (float)TIM1_PERIOD);
    s_wDutyV = (uint32_t)(fV * (float)TIM1_PERIOD);
    s_wDutyW = (uint32_t)(fW * (float)TIM1_PERIOD);

    LL_TIM_OC_SetCompareCH1(TIM1, s_wDutyU);
    LL_TIM_OC_SetCompareCH2(TIM1, s_wDutyV);
    LL_TIM_OC_SetCompareCH3(TIM1, s_wDutyW);
}

void haltim1_Start(void)
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM1);
}

void haltim1_Stop(void)
{
    LL_TIM_DisableAllOutputs(TIM1);
}
