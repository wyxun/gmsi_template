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
    /* TODO: add hardware handles here, e.g.:
     *   TIM_TypeDef      *ptTimer;         // PWM timer base
     *   ADC_TypeDef      *ptAdc;           // ADC peripheral
     *   uint32_t          wAdcChannelU;    // ADC channel index for phase U
     *   uint32_t          wAdcChannelV;
     *   uint32_t          wAdcChannelW;    */
    uint32_t wPwmPeriod;                    /* Timer auto-reload / period */
} foc_motor_context_t;

/*
 * One-shot bind: populate all HAL callbacks and validate.
 *
 * @param ptHal      [out] HAL interface to populate
 * @param ptContext  [in]  per-motor context, must live as long as the motor
 * @return FOC_RESULT_OK on success
 */
foc_result_t foc_hal_Bind(foc_hal_t *ptHal, foc_motor_context_t *ptContext);

/*
 * Convenience: bind ptHal to a static default context.
 * Multi-motor boards should call foc_hal_Bind() with per-motor contexts instead.
 */
foc_result_t foc_hal_BindDefault(foc_hal_t *ptHal);

#endif /* FOC_HAL_ADAPTER_H */
