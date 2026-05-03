/**
 * @file  haladc.h
 * @brief ADC1/ADC2 — 3-shunt current sensing + voltage/temp monitoring
 */

#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>

#define HALADC_ADC1         0
#define HALADC_ADC2         1

/* Regular channel indices */
#define HALADC_REG_BUS_VOLTAGE   0   /* ADC1_IN1  (PA0) */
#define HALADC_REG_TEMPERATURE   1   /* ADC1_IN5  (PB14) */
#define HALADC_REG_POTENTIOMETER 2   /* ADC1_IN11 (PB12) */

/* Injected channel indices */
#define HALADC_INJ_U        0   /* ADC1_IN3  (PA2, OPAMP1 out) */
#define HALADC_INJ_V        1   /* ADC2_IN3  (PA6, OPAMP2 out) */
#define HALADC_INJ_W        1   /* ADC1_IN12 (PB1, OPAMP3 out) */

void haladc_Init(void);
void haladc_StartRegular(void);
uint32_t haladc_GetRegular(uint32_t wChannel);
uint32_t haladc_GetInjected(uint32_t wAdc, uint32_t wRank);

#endif /* __HALADC_H__ */
