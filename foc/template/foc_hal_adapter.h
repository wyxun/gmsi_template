/*******************************************************************************
 * @file    foc_hal_adapter.h
 * @brief   Modus FOC — Platform HAL adapter template
 *
 * NOTE: This is a TEMPLATE for porting Modus FOC to a new MCU platform.
 * Search for "TODO" markers and replace with your platform's implementation.
 *
 * Architecture:
 *   Modus FOC core  ->  foc_hal_t (PWM + ADC interface)  ->  this adapter
 *   This adapter bridges foc_hal_t callbacks to the actual MCU timer and ADC
 *   hardware. Each motor instance gets its own context (pContext).
 *
 * Required reading before porting:
 *   foc/README.md §5 (hardware interface spec)
 ******************************************************************************/

#ifndef FOC_HAL_ADAPTER_H
#define FOC_HAL_ADAPTER_H

#include "foc_hal.h"

/*
 * Per-motor hardware context.
 *
 * Modifies:
 *   - wPwmPeriod: must match timer ARR/period register value
 *
 * A single static default instance is provided below. For multi-motor builds,
 * declare separate contexts per motor.
 */
typedef struct {
    uint32_t wPwmPeriod;        /**< 定时器自动重装载值（ARR/period） */
} foc_motor_context_t;

/**
 * @brief  初始化 HAL 适配器并绑定所有回调
 * @param  ptHal     [out] 待填充的 HAL 接口
 * @param  ptContext [in]  单电机硬件上下文（必须在整个生命周期内有效）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_Bind(foc_hal_t *ptHal, foc_motor_context_t *ptContext);

/**
 * @brief  使用静态默认上下文绑定 HAL 接口
 * @param  ptHal  [out] 待填充的 HAL 接口
 * @return        FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hal_BindDefault(foc_hal_t *ptHal);

#endif /* FOC_HAL_ADAPTER_H */
