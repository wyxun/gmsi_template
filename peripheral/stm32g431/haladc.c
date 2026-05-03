/**
 * @file  haladc.c
 * @brief ADC1/ADC2 init — regular + injected, TIM1_CH4 trigger
 *
 * ADC1 Regular:   CH1(busV), CH5(temp), CH11(pot)
 * ADC1 Injected:  CH3(U-phase via OPAMP1), CH12(W-phase via OPAMP3)
 * ADC2 Injected:  OPAMP3 internal (V-phase via OPAMP2 internal), CH3(V-phase)
 *
 * Trigger: TIM1_CH4 rising edge for injected groups.
 */

#include "haladc.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"

void haladc_Init(void)
{
    /* ---- Clock ---- */
    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /* ---- Analog GPIO ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_3;         /* PA0=ADC1_IN1, PA2=ADC1_IN3 */
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_14;                      /* PB14=ADC1_IN5 */
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_12;                      /* PB12=ADC1_IN11 */
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_1;                       /* PB1=ADC1_IN12 */
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_6;                       /* PA6=ADC2_IN3 */
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- ADC1 ---- */
    /* Common: async clock, div1 */
    LL_ADC_SetCommonClock(ADC12_COMMON, LL_ADC_CLOCK_ASYNC_DIV1);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetLowPowerMode(ADC1, 0);

    /* Regular: software trigger, 3-rank scan, single conversion */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_PRESERVED);

    /* Regular ranks */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_11);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

    /* Injected: TIM1_CH4 rising edge, 2-rank */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);

    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_12);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);

    /* Enable regulator, wait */
    LL_ADC_EnableInternalRegulator(ADC1);
    for (volatile uint32_t i = 0; i < 10000; i++) {}

    /* Calibration */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}

    /* Enable */
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}

    /* ---- ADC2 (injected only) ---- */
    LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetLowPowerMode(ADC2, 0);

    /* Regular disabled */
    LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE);

    /* Injected: TIM1_CH4 rising edge, 2-rank */
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
    LL_ADC_INJ_SetSequencerLength(ADC2, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_INJ_SetTrigAuto(ADC2, LL_ADC_INJ_TRIG_INDEPENDENT);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);

    /* V-phase: internal OPAMP3 connection + CH3 */
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP3_ADC2);
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_3);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_6CYCLES_5);

    LL_ADC_EnableInternalRegulator(ADC2);
    for (volatile uint32_t i = 0; i < 10000; i++) {}

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) {}
}

void haladc_StartRegular(void)
{
    LL_ADC_REG_StartConversion(ADC1);
}

uint32_t haladc_GetRegular(uint32_t wChannel)
{
    /* Wait for end of sequence */
    while (!LL_ADC_IsActiveFlag_EOS(ADC1)) {}
    (void)ADC1->DR; /* clear EOS */
    /* Read from the specified rank's data register */
    return LL_ADC_REG_ReadConversionData12(ADC1);
}

uint32_t haladc_GetInjected(uint32_t wAdc, uint32_t wRank)
{
    if (wAdc == HALADC_ADC1) {
        if (wRank == 0) return LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        else            return LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    } else {
        if (wRank == 0) return LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        else            return LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
    }
}
