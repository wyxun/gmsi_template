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
    void *pContext;                                 /**< PWM 上下文 */
    foc_result_t (*fnSetDuty)(void *pContext,
                              q_type qDutyU,
                              q_type qDutyV,
                              q_type qDutyW);      /**< 设置占空比 */
    foc_result_t (*fnEnable)(void *pContext, bool bEnable); /**< 使能/禁用 */
    void (*fnEmergencyStop)(void *pContext);        /**< 紧急停止 */
} foc_pwm_if_t;

#endif /* __FOC_HAL_PWM_H__ */
