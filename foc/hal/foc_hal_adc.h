/*******************************************************************************
 * @file    foc_hal_adc.h
 * @brief   FOC ADC HAL 全局注册接口（平台无关）
 ******************************************************************************/

#ifndef __FOC_HAL_ADC_H__
#define __FOC_HAL_ADC_H__

#include "foc_hal_types.h"

#define HALADC_ADC1     0U  /**< ADC1 索引 */
#define HALADC_ADC2     1U  /**< ADC2 索引 */

#define HALADC_REG_BUS_VOLTAGE      0U  /**< 母线电压寄存器索引 */
#define HALADC_REG_TEMPERATURE      1U  /**< 温度寄存器索引 */
#define HALADC_REG_POTENTIOMETER    2U  /**< 电位器寄存器索引 */

#endif /* __FOC_HAL_ADC_H__ */
