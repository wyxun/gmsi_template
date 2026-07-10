/**
 * @file   grblhal_stubs.c
 * @brief  Stub implementations for all grblHAL hal.* handlers
 *
 * Every handler not covered by grblhal_stream.c is stubbed here
 * with minimal "return 0 / true / empty" behavior.
 * Stepper stubs log to RTT so planner output is observable.
 */

#include "grblhal_driver.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include "modbus.h"
#include "canbus.h"
#include "encoders.h"

/* =========================================================================
 *  driver_setup
 * ========================================================================= */
bool grblhal_driver_setup(settings_t *settings)
{
    (void)settings;
    return true;
}

/* =========================================================================
 *  delay_ms — busy-wait via SysTick
 * ========================================================================= */
void grblhal_delay_ms(uint32_t ms, delay_callback_ptr callback)
{
    (void)callback;
    /* Simple volatile loop — sufficient for Phase 1.
     * In production, use a hardware timer with callback. */
    for (volatile uint32_t i = 0; i < ms * 10000; i++) {
        /* spin */
    }
}

/* =========================================================================
 *  Atomic operations — bare C (safe for cooperative Phase 1)
 * ========================================================================= */
void grblhal_set_bits_atomic(volatile uint_fast16_t *value, uint_fast16_t bits)
{
    *value |= bits;
}

uint_fast16_t grblhal_clear_bits_atomic(volatile uint_fast16_t *value, uint_fast16_t v)
{
    uint_fast16_t prev = *value;
    *value &= ~v;
    return prev;
}

uint_fast16_t grblhal_set_value_atomic(volatile uint_fast16_t *value, uint_fast16_t bits)
{
    uint_fast16_t prev = *value;
    *value = bits;
    return prev;
}

/* =========================================================================
 *  Limits
 * ========================================================================= */
void grblhal_limits_enable(bool on, axes_signals_t homing_cycle)
{
    (void)on;
    (void)homing_cycle;
}

limit_signals_t grblhal_limits_get_state(void)
{
    limit_signals_t sig = {0};
    return sig;
}

/* =========================================================================
 *  Control signals
 * ========================================================================= */
control_signals_t grblhal_control_get_state(void)
{
    control_signals_t sig = {0};
    return sig;
}

/* =========================================================================
 *  Coolant
 * ========================================================================= */
void grblhal_coolant_set_state(coolant_state_t state)
{
    (void)state;
}

coolant_state_t grblhal_coolant_get_state(void)
{
    coolant_state_t state = {0};
    return state;
}

/* =========================================================================
 *  Spindle data — stub get/reset
 * ========================================================================= */
static spindle_data_t s_spindle_data;

static spindle_data_t *grblhal_spindle_get_data(spindle_data_request_t request)
{
    (void)request;
    return &s_spindle_data;
}

static void grblhal_spindle_reset_data(void)
{
    memset(&s_spindle_data, 0, sizeof(s_spindle_data));
}

/* =========================================================================
 *  Stepper — stub + RTT log on major state transitions
 * ========================================================================= */
void grblhal_stepper_wake_up(void)
{
    SEGGER_RTT_WriteString(0, "[grblHAL] stepper wake_up\n");
}

void grblhal_stepper_go_idle(bool clear_signals)
{
    (void)clear_signals;
    SEGGER_RTT_WriteString(0, "[grblHAL] stepper go_idle\n");
}

void grblhal_stepper_enable(axes_signals_t enable, bool hold)
{
    (void)enable;
    (void)hold;
}

void grblhal_stepper_cycles_per_tick(uint32_t cycles_per_tick)
{
    (void)cycles_per_tick;
}

void grblhal_stepper_pulse_start(stepper_t *stepper)
{
    (void)stepper;
}

/* =========================================================================
 *  Probe — not connected
 * ========================================================================= */
static probe_state_t grblhal_probe_get_state(void)
{
    probe_state_t s = {0};
    return s;
}

static bool grblhal_probe_is_triggered(probe_id_t probe_id)
{
    (void)probe_id;
    return false;
}

/* =========================================================================
 *  RTC — not available
 * ========================================================================= */
static bool grblhal_rtc_get_datetime(struct tm *datetime)
{
    (void)datetime;
    return false;
}

static bool grblhal_rtc_set_datetime(struct tm *datetime)
{
    (void)datetime;
    return false;
}

/* =========================================================================
 *  Tool — stub select
 * ========================================================================= */
static void grblhal_tool_select(tool_data_t *tool, bool next)
{
    (void)tool;
    (void)next;
}

/* =========================================================================
 *  driver_init — assembles the full hal struct
 * ========================================================================= */
bool driver_init(void)
{
    memset(&hal, 0, sizeof(grbl_hal_t));

    /* Required properties */
    hal.info            = "modus_template stub driver";
    hal.driver_version  = "260710";
    hal.step_us_min     = 2.0f;
    hal.f_step_timer    = 170000000U;
    hal.f_mcu           = 170U;
    hal.rx_buffer_size  = 256U;
    hal.driver_cap.value = 0;
    hal.signals_cap.mask = 0;
    hal.limits_cap.bits  = 0;
    hal.home_cap         = (home_signals_t){0};
    hal.coolant_cap.mask = 0;
    hal.motor_warning_cap = (home_signals_t){0};
    hal.motor_fault_cap   = (home_signals_t){0};
    hal.signals_pullup_disable_cap.mask = 0;

    /* Required function pointers */
    hal.driver_setup      = grblhal_driver_setup;
    hal.delay_ms          = grblhal_delay_ms;
    hal.set_bits_atomic   = grblhal_set_bits_atomic;
    hal.clear_bits_atomic = grblhal_clear_bits_atomic;
    hal.set_value_atomic  = grblhal_set_value_atomic;

    /* Limits */
    hal.limits.enable    = grblhal_limits_enable;
    hal.limits.get_state = grblhal_limits_get_state;

    /* Control */
    hal.control.get_state = grblhal_control_get_state;

    /* Coolant */
    hal.coolant.set_state = grblhal_coolant_set_state;
    hal.coolant.get_state = grblhal_coolant_get_state;

    /* Spindle data */
    hal.spindle_data.get   = grblhal_spindle_get_data;
    hal.spindle_data.reset = grblhal_spindle_reset_data;

    /* Stepper */
    hal.stepper.wake_up         = grblhal_stepper_wake_up;
    hal.stepper.go_idle         = grblhal_stepper_go_idle;
    hal.stepper.enable          = grblhal_stepper_enable;
    hal.stepper.cycles_per_tick = grblhal_stepper_cycles_per_tick;
    hal.stepper.pulse_start     = grblhal_stepper_pulse_start;

    /* Probe */
    hal.probe.get_state    = grblhal_probe_get_state;
    hal.probe.is_triggered = grblhal_probe_is_triggered;

    /* RTC */
    hal.rtc.get_datetime = grblhal_rtc_get_datetime;
    hal.rtc.set_datetime = grblhal_rtc_set_datetime;

    /* Tool */
    hal.tool.select = grblhal_tool_select;

    /* Stream — populated by grblhal_stream_init() */
    grblhal_stream_init();

    /* Optional: get_elapsed_ticks (driven by grblhal_Clock) */
    hal.get_elapsed_ticks = grblhal_get_ticks;

    /* NVS — NULL (use grblHAL compile-time defaults) */
    hal.nvs.memcpy_from_nvs  = NULL;
    hal.nvs.memcpy_to_nvs    = NULL;
    hal.nvs.memcpy_from_flash = NULL;
    hal.nvs.memcpy_to_flash   = NULL;
    hal.nvs.get_byte  = NULL;
    hal.nvs.put_byte  = NULL;

    return true;
}

void _exit(int status)
{
    (void)status;
    while (1) {}
}

void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
    (void)file;
    (void)line;
    (void)func;
    (void)failedexpr;
    while (1) {}
}

/* =========================================================================
 *  Optional module stubs (CAN, Modbus, Encoders)
 * ========================================================================= */
uint8_t encoders_get_count(void)
{
    return 0;
}

bool canbus_enabled(void)
{
    return false;
}

modbus_cap_t modbus_isup(void)
{
    modbus_cap_t cap = {0};
    return cap;
}

const modbus_function_properties_t *modbus_get_function_properties(modbus_function_t function)
{
    (void)function;
    return NULL;
}

status_code_t modbus_message(uint8_t server, modbus_function_t function, uint16_t address, uint16_t *values, uint8_t registers, modbus_callback_ptr callback)
{
    (void)server;
    (void)function;
    (void)address;
    (void)values;
    (void)registers;
    (void)callback;
    return Status_GcodeUnsupportedCommand;
}
