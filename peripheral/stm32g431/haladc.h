/**
 * @file  haladc.h
 * @brief ADC1/ADC2 — 3-shunt current sensing + voltage/temp monitoring
 */

#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>

#ifndef HALADC_ADC1
#define HALADC_ADC1         0U
#endif
#ifndef HALADC_ADC2
#define HALADC_ADC2         1U
#endif

/* Regular channel indices */
#ifndef HALADC_REG_BUS_VOLTAGE
#define HALADC_REG_BUS_VOLTAGE   0U  /* ADC1_IN1  (PA0) */
#endif
#ifndef HALADC_REG_TEMPERATURE
#define HALADC_REG_TEMPERATURE   1U  /* ADC1_IN5  (PB14) */
#endif
#ifndef HALADC_REG_POTENTIOMETER
#define HALADC_REG_POTENTIOMETER 2U  /* ADC1_IN11 (PB12) */
#endif

/* Injected channel indices */
#define HALADC_INJ_U        0U  /* ADC1_IN3  (PA2, OPAMP1 out) */
#define HALADC_INJ_V        1U  /* ADC2_IN3  (PA6, OPAMP2 out) */
#define HALADC_INJ_W        1U  /* ADC1_IN12 (PB1, OPAMP3 out) */

void haladc_Init(void);
void haladc_StartRegular(void);
uint32_t haladc_GetRegular(uint32_t wChannel);
uint32_t haladc_GetInjected(uint32_t wAdc, uint32_t wRank);

#endif /* __HALADC_H__ */
