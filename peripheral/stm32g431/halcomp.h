/**
 * @file  halcomp.h
 * @brief COMP1/2/4 — overcurrent protection, outputs→TIM1 BKIN
 */

#ifndef __HALCOMP_H__
#define __HALCOMP_H__

#include <stdint.h>

#define HALCOMP_IDX_COMP1   0
#define HALCOMP_IDX_COMP2   1
#define HALCOMP_IDX_COMP4   2

void halcomp_Init(void);
uint32_t halcomp_GetOutput(uint32_t wIdx);

#endif /* __HALCOMP_H__ */
