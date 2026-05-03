/*******************************************************************************
 * @file    foc_hal_adc.h
 * @brief   FOC ADC HAL 全局注册接口（平台无关）
 ******************************************************************************/

#ifndef __FOC_HAL_ADC_H__
#define __FOC_HAL_ADC_H__

#include "foc_hal_types.h"

#define HALADC_ADC1     0U
#define HALADC_ADC2     1U

#define HALADC_REG_BUS_VOLTAGE      0U
#define HALADC_REG_TEMPERATURE      1U
#define HALADC_REG_POTENTIOMETER    2U

void foc_hal_adc_register(foc_adc_ops_t *ptOps);
foc_adc_ops_t *foc_hal_adc_get(void);

#endif /* __FOC_HAL_ADC_H__ */
