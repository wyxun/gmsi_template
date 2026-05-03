/*******************************************************************************
 * @file    foc_hal_pwm.h
 * @brief   FOC PWM 硬件抽象接口（平台无关）
 ******************************************************************************/

#ifndef __FOC_HAL_PWM_H__
#define __FOC_HAL_PWM_H__

#include <stdint.h>
#include <stdbool.h>
#include "foc_math_types.h"

typedef struct {
    void (*fnSetDuty)(q_type qDutyU, q_type qDutyV, q_type qDutyW);
    void (*fnEnable)(bool bEn);
    void (*fnEmergencyStop)(void);
} foc_pwm_ops_t;

void foc_hal_pwm_register(foc_pwm_ops_t *ptOps);
foc_pwm_ops_t *foc_hal_pwm_get(void);

#endif /* __FOC_HAL_PWM_H__ */
