/**
 * @file   grblhal_driver.h
 * @brief  Internal declarations for the grblHAL adapt layer
 */

#ifndef __GRBLHAL_DRIVER_H__
#define __GRBLHAL_DRIVER_H__

/*
 * Avoid name collision: both modus (mbase.h) and grblHAL (messages.h)
 * define message_t. Since modus.h is always included first by the class
 * header, we rename grblHAL's version while including its headers.
 */
#define message_t grblhal_message_t

#include "grbl.h"
#include "hal.h"
#include "core_handlers.h"

#undef message_t

/* --------------------------------------------------------------------------
 *  CNC Shield GPIO Port and Pin Mappings
 * -------------------------------------------------------------------------- */
#define STEPPER_EN_PORT         GPIOA
#define STEPPER_EN_PIN          GPIO_PINS_9

#define X_STEP_PORT             GPIOA
#define X_STEP_PIN              GPIO_PINS_10
#define X_DIR_PORT              GPIOB
#define X_DIR_PIN               GPIO_PINS_4

#define Y_STEP_PORT             GPIOB
#define Y_STEP_PIN              GPIO_PINS_3
#define Y_DIR_PORT              GPIOB
#define Y_DIR_PIN               GPIO_PINS_10

#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              GPIO_PINS_5
#define Z_DIR_PORT              GPIOA
#define Z_DIR_PIN               GPIO_PINS_8

#define X_LIMIT_PORT            GPIOC
#define X_LIMIT_PIN             GPIO_PINS_7

#define Y_LIMIT_PORT            GPIOA
#define Y_LIMIT_PIN             GPIO_PINS_15

#define Z_LIMIT_PORT            GPIOA
#define Z_LIMIT_PIN             GPIO_PINS_7

#ifdef __cplusplus
extern "C" {
#endif

/* Stream I/O init (real — RTT) */
void grblhal_stream_init(void);

/* Driver init (called from grblhal_class_Init) */
bool driver_init(void);

/* get_elapsed_ticks — driven by 1ms SysTick */
uint32_t grblhal_get_ticks(void);
void grblhal_ticks_inc(void);

/* Individual handler groups (implemented in grblhal_stubs.c) */
void  grblhal_stepper_wake_up(void);
void  grblhal_stepper_go_idle(bool clear_signals);
void  grblhal_stepper_enable(axes_signals_t enable, bool hold);
void  grblhal_stepper_cycles_per_tick(uint32_t cycles_per_tick);
void  grblhal_stepper_pulse_start(stepper_t *stepper);
void  grblhal_limits_enable(bool on, axes_signals_t homing_cycle);
limit_signals_t grblhal_limits_get_state(void);
control_signals_t grblhal_control_get_state(void);
void  grblhal_coolant_set_state(coolant_state_t state);
coolant_state_t grblhal_coolant_get_state(void);
void  grblhal_delay_ms(uint32_t ms, delay_callback_ptr callback);
void  grblhal_set_bits_atomic(volatile uint_fast16_t *value, uint_fast16_t bits);
uint_fast16_t grblhal_clear_bits_atomic(volatile uint_fast16_t *value, uint_fast16_t bits);
uint_fast16_t grblhal_set_value_atomic(volatile uint_fast16_t *value, uint_fast16_t bits);
bool  grblhal_driver_setup(settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* __GRBLHAL_DRIVER_H__ */
