/**
 * @file  haltim1.h
 * @brief TIM1 — 3-phase complementary PWM, center-aligned, CH4→ADC trigger
 */

#ifndef __HALTIM1_H__
#define __HALTIM1_H__

#include <stdint.h>
#include <stdbool.h>

void haltim1_Init(void);
void haltim1_SetDuty(float fU, float fV, float fW);
void haltim1_Start(void);
void haltim1_Stop(void);

#endif /* __HALTIM1_H__ */
