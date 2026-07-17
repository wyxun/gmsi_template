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
    foc_pwm_if_t tPwm;
    foc_adc_if_t tAdc;
} foc_hal_t;

foc_result_t foc_hal_Validate(const foc_hal_t *ptHal);
foc_result_t foc_hal_SetDuty(const foc_pwm_if_t *ptPwm,
                             q_type qDutyU,
                             q_type qDutyV,
                             q_type qDutyW);
foc_result_t foc_hal_Enable(const foc_pwm_if_t *ptPwm, bool bEnable);
void foc_hal_EmergencyStop(const foc_pwm_if_t *ptPwm);
foc_result_t foc_hal_CurrentCalibrate(const foc_adc_if_t *ptAdc,
                                      foc_adc_calib_t *ptCalib);
foc_result_t foc_hal_CurrentGetRaw(const foc_adc_if_t *ptAdc,
                                   uint32_t *pwRawU,
                                   uint32_t *pwRawV,
                                   uint32_t *pwRawW);
foc_result_t foc_hal_CurrentReconstruct(
    const foc_adc_if_t *ptAdc,
    phase_current_handle_t *ptHandle);

#endif /* __FOC_HAL_H__ */
