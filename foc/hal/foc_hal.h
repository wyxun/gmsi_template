/*******************************************************************************
 * @file    foc_hal.h
 * @brief   FOC HAL 层统一入口
 ******************************************************************************/

#ifndef __FOC_HAL_H__
#define __FOC_HAL_H__

#include "foc_hal_types.h"
#include "foc_hal_pwm.h"
#include "foc_hal_adc.h"

typedef struct {
    foc_pwm_if_t tPwm;  /**< PWM 接口 */
    foc_adc_if_t tAdc;  /**< ADC 接口 */
} foc_hal_t;

/**
 * @brief  验证 HAL 接口完整性
 * @param  ptHal  HAL 指针
 * @return        FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_Validate(const foc_hal_t *ptHal);
/**
 * @brief  设置三相占空比
 * @param  ptPwm   PWM 接口指针
 * @param  qDutyU  U 相占空比
 * @param  qDutyV  V 相占空比
 * @param  qDutyW  W 相占空比
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_SetDuty(const foc_pwm_if_t *ptPwm,
                             q_type qDutyU,
                             q_type qDutyV,
                             q_type qDutyW);
/**
 * @brief  使能或禁用 PWM 输出
 * @param  ptPwm    PWM 接口指针
 * @param  bEnable  true=使能, false=禁用
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_Enable(const foc_pwm_if_t *ptPwm, bool bEnable);
/**
 * @brief  紧急停止 PWM 输出
 * @param  ptPwm  PWM 接口指针
 */
void foc_hal_EmergencyStop(const foc_pwm_if_t *ptPwm);
/**
 * @brief  执行电流采样偏移校准
 * @param  ptAdc    ADC 接口指针
 * @param  ptCalib  输出校准值
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_CurrentCalibrate(const foc_adc_if_t *ptAdc,
                                      foc_adc_calib_t *ptCalib);
/**
 * @brief  读取三相 ADC 原始值
 * @param  ptAdc   ADC 接口指针
 * @param  pwRawU  输出 U 相原始值
 * @param  pwRawV  输出 V 相原始值
 * @param  pwRawW  输出 W 相原始值
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_CurrentGetRaw(const foc_adc_if_t *ptAdc,
                                   uint32_t *pwRawU,
                                   uint32_t *pwRawV,
                                   uint32_t *pwRawW);
/**
 * @brief  从 ADC 原始值重建三相电流
 * @param  ptAdc     ADC 接口指针
 * @param  ptHandle  输出电流句柄
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_CurrentReconstruct(
    const foc_adc_if_t *ptAdc,
    phase_current_handle_t *ptHandle);

#endif /* __FOC_HAL_H__ */
