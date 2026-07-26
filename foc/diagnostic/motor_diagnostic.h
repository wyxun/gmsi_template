/*******************************************************************************
 * @file    motor_diagnostic.h
 * @brief   Hardware bring-up diagnostics (build-gated, non-production)
 *
 * This module is compiled only when FOC_ENABLE_DIAGNOSTIC=1. It drives the
 * power stage exclusively through the narrow gated motor diagnostic API in
 * motor.h plus the public snapshot/raw-current queries; it never accesses
 * motor private members.
 ******************************************************************************/

#ifndef MOTOR_DIAGNOSTIC_H
#define MOTOR_DIAGNOSTIC_H

#include "motor.h"

#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC

/**
 * @brief  固定占空比 PWM/ADC 连线测试
 *         强制要求电机在 IDLE、无故障状态下执行，自带限幅和保护
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_diagnostic_FixedDutyTest(motor_handle_t *ptMotor);

#endif /* FOC_ENABLE_DIAGNOSTIC */

#endif /* MOTOR_DIAGNOSTIC_H */
