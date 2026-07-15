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
#include "port_nvs.h"
#include "at32f403a_407.h"

static volatile uint32_t s_wGrblhalTicks = 0;

uint32_t grblhal_get_ticks(void)
{
    return s_wGrblhalTicks;
}

void grblhal_ticks_inc(void)
{
    s_wGrblhalTicks++;
}

#if MODUS_ENABLE
#include "modus.h"
static on_execute_realtime_ptr s_fnPrevExecuteRealtime = NULL;

static void debug_modus_realtime_hook(sys_state_t state)
{
    if (s_fnPrevExecuteRealtime) {
        s_fnPrevExecuteRealtime(state);
    }
    modus_Run();
}
#endif

/* =========================================================================
 *  driver_setup
 * ========================================================================= */
bool grblhal_driver_setup(settings_t *settings)
{
    (void)settings;
#if MODUS_ENABLE
    s_fnPrevExecuteRealtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = debug_modus_realtime_hook;
#endif

    /* Enable DWT Cycle Counter for precise microsecond delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *(volatile uint32_t *)0xE0001FB0 = 0xC5ACCE55; /* Unlock DWT write access on Cortex-M4 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Initialize TMR5 for Stepper Pulse Timer */
    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR5);
    
    /* TMR5 basic setup: 32-bit timer, counting up, no prescaler (runs at SystemCoreClock) */
    tmr_base_init(TMR5, 0xFFFFFFFF, 0);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
    
    /* Enable TMR5 Update (Overflow) Interrupt */
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, TRUE);
    
    /* Configure NVIC for TMR5 (Priority 0, highest) */
    nvic_irq_enable(TMR5_GLOBAL_IRQn, 0, 0);

    /* Explicitly call my_plugin_init() here.
     * grbllib.c in this build does NOT include plugins_init.h,
     * so the weak symbol is never referenced and --gc-sections strips
     * the entire app_plugins.o.  Calling it from driver_setup (which IS
     * in the live call-graph) keeps it alive. */
    extern void my_plugin_init(void);
    my_plugin_init();

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
    /* Read limit switches (active-low due to pull-up; RESET/0 = triggered) */
    sig.min.x = (gpio_input_data_bit_read(X_LIMIT_PORT, X_LIMIT_PIN) == RESET);
    sig.min.y = (gpio_input_data_bit_read(Y_LIMIT_PORT, Y_LIMIT_PIN) == RESET);
    sig.min.z = (gpio_input_data_bit_read(Z_LIMIT_PORT, Z_LIMIT_PIN) == RESET);
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
    /* Clear update flag and enable TMR5 counter */
    tmr_flag_clear(TMR5, TMR_OVF_FLAG);
    tmr_counter_enable(TMR5, TRUE);
}

void grblhal_stepper_go_idle(bool clear_signals)
{
    (void)clear_signals;
    /* Disable TMR5 counter */
    tmr_counter_enable(TMR5, FALSE);
}

void grblhal_stepper_enable(axes_signals_t enable, bool hold)
{
    (void)hold;
    /* CNC Shield Stepper Enable is active-low (LOW = Enable, HIGH = Disable) */
    if (enable.mask == 0) {
        gpio_bits_set(STEPPER_EN_PORT, STEPPER_EN_PIN);
    } else {
        gpio_bits_reset(STEPPER_EN_PORT, STEPPER_EN_PIN);
    }
}

void grblhal_stepper_cycles_per_tick(uint32_t cycles_per_tick)
{
    if (cycles_per_tick < 2) {
        cycles_per_tick = 2;
    }
    /* Write new period value to 32-bit TMR5 */
    tmr_period_value_set(TMR5, cycles_per_tick - 1);
}

void grblhal_stepper_pulse_start(stepper_t *stepper)
{
    /* 1. Output Direction Signals if changed */
    if (stepper->dir_changed.value != 0) {
        if (stepper->dir_out.x) {
            gpio_bits_set(X_DIR_PORT, X_DIR_PIN);  /* X-DIR HIGH */
        } else {
            gpio_bits_reset(X_DIR_PORT, X_DIR_PIN); /* X-DIR LOW */
        }
        if (stepper->dir_out.y) {
            gpio_bits_set(Y_DIR_PORT, Y_DIR_PIN);  /* Y-DIR HIGH */
        } else {
            gpio_bits_reset(Y_DIR_PORT, Y_DIR_PIN); /* Y-DIR LOW */
        }
        if (stepper->dir_out.z) {
            gpio_bits_set(Z_DIR_PORT, Z_DIR_PIN);  /* Z-DIR HIGH */
        } else {
            gpio_bits_reset(Z_DIR_PORT, Z_DIR_PIN); /* Z-DIR LOW */
        }
    }

    /* 2. Output Step Pulses HIGH */
    if (stepper->step_out.x) {
        gpio_bits_set(X_STEP_PORT, X_STEP_PIN); /* X-STEP HIGH */
    }
    if (stepper->step_out.y) {
        gpio_bits_set(Y_STEP_PORT, Y_STEP_PIN); /* Y-STEP HIGH */
    }
    if (stepper->step_out.z) {
        gpio_bits_set(Z_STEP_PORT, Z_STEP_PIN); /* Z-STEP HIGH */
    }

    /* 3. Delay for minimum step pulse width (hal.step_us_min microseconds) using DWT */
    uint32_t delay_ticks = (uint32_t)(hal.step_us_min * 240.0f); /* 240 ticks per microsecond at 240MHz */
    uint32_t start_time = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_time) < delay_ticks) {
        /* spin */
    }

    /* 4. Reset Step Pins LOW */
    if (stepper->step_out.x) {
        gpio_bits_reset(X_STEP_PORT, X_STEP_PIN); /* X-STEP LOW */
    }
    if (stepper->step_out.y) {
        gpio_bits_reset(Y_STEP_PORT, Y_STEP_PIN); /* Y-STEP LOW */
    }
    if (stepper->step_out.z) {
        gpio_bits_reset(Z_STEP_PORT, Z_STEP_PIN); /* Z-STEP LOW */
    }
}

void grblhal_stepper_isr(void)
{
    if (hal.stepper.interrupt_callback != NULL) {
        hal.stepper.interrupt_callback();
    }
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
 *  Settings changed — stub settings change handler
 * ========================================================================= */
static void grblhal_settings_changed(settings_t *settings, settings_changed_flags_t changed)
{
    (void)settings;
    (void)changed;
}

static bool grblhal_nvs_read_flash(uint8_t *dest)
{
    return port_nvs_read(dest, hal.nvs.size);
}

static bool grblhal_nvs_write_flash(uint8_t *source)
{
    return port_nvs_write(source, hal.nvs.size);
}

/* =========================================================================
 *  driver_init — assembles the full hal struct
 * ========================================================================= */
bool driver_init(void)
{
    /* Required properties */
    hal.info            = "AT32F407 CNC Controller";
    hal.driver_version  = "260715";
    hal.step_us_min     = 2.0f;
    hal.f_step_timer    = 240000000U;
    hal.f_mcu           = 240U;
    hal.rx_buffer_size  = 256U;
    hal.driver_cap.value = 0;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.step_pulse_delay = 1;
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

    /* NVS — SPIM Flash configuration */
    port_nvs_init();
    hal.nvs.type = NVS_Flash;
    hal.nvs.size = GRBL_NVS_SIZE;
    hal.nvs.size_max = 2048;
    hal.nvs.memcpy_from_flash = grblhal_nvs_read_flash;
    hal.nvs.memcpy_to_flash   = grblhal_nvs_write_flash;
    hal.nvs.memcpy_from_nvs   = NULL;
    hal.nvs.memcpy_to_nvs     = NULL;
    hal.nvs.get_byte          = NULL;
    hal.nvs.put_byte          = NULL;

    hal.settings_changed = grblhal_settings_changed;

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
    return Status_GcodeUnsupportedCommand;
}

#if !defined(MSHELL_ENABLE) || (MSHELL_ENABLE == 0)
// Provide dummy SEGGER RTT functions if RTT is not included in the release build
unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s)
{
    (void)BufferIndex;
    (void)s;
    return 0;
}
unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes)
{
    (void)BufferIndex;
    (void)pBuffer;
    (void)NumBytes;
    return 0;
}
unsigned SEGGER_RTT_PutChar(unsigned BufferIndex, char c)
{
    (void)BufferIndex;
    (void)c;
    return 0;
}
#endif
