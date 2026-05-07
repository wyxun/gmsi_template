/*******************************************************************************
 * @file    foc_hal.h
 * @brief   FOC HAL 层统一入口
 ******************************************************************************/

#ifndef __FOC_HAL_H__
#define __FOC_HAL_H__

#include "foc_hal_types.h"
#include "foc_hal_pwm.h"
#include "foc_hal_adc.h"

extern int  foc_hal_Init(void);
extern void foc_hal_mdi_register(void);
void foc_hal_current_reconstruct(phase_current_handle_t *ptHandle);

#endif /* __FOC_HAL_H__ */
