/**
 * @file grblhal_motion.c
 * @brief AT32F407 stepper, spindle and motion-output safety adapter.
 */

#include "grblhal_driver.h"
#include "perf_counter.h"
#ifdef UNUSED
#undef UNUSED
#endif
#include "at32f403a_407.h"

static uint32_t s_wStepPulseUs = 2;
static uint32_t s_wDirectionDelayUs = 0;
static axes_signals_t s_tStepInvert = {0};
static axes_signals_t s_tDirInvert = {0};
static axes_signals_t s_tEnableInvert = {0};
static spindle_state_t s_tSpindleState = {0};
static spindle_id_t s_chSpindleId = -1;

static void stepper_write_dir(axes_signals_t signals)
{
    signals.mask ^= s_tDirInvert.mask;
    gpio_bits_write(X_DIR_PORT, X_DIR_PIN, signals.x ? TRUE : FALSE);
    gpio_bits_write(Y_DIR_PORT, Y_DIR_PIN, signals.y ? TRUE : FALSE);
    gpio_bits_write(Z_DIR_PORT, Z_DIR_PIN, signals.z ? TRUE : FALSE);
}

static void stepper_write_step(axes_signals_t signals)
{
    signals.mask ^= s_tStepInvert.mask;
    gpio_bits_write(X_STEP_PORT, X_STEP_PIN, signals.x ? TRUE : FALSE);
    gpio_bits_write(Y_STEP_PORT, Y_STEP_PIN, signals.y ? TRUE : FALSE);
    gpio_bits_write(Z_STEP_PORT, Z_STEP_PIN, signals.z ? TRUE : FALSE);
}

void grblhal_motion_settings_changed(settings_t *settings,
                                     settings_changed_flags_t changed)
{
    (void)changed;
    if (settings == NULL) {
        return;
    }

    s_tStepInvert = settings->steppers.step_invert;
    s_tDirInvert = settings->steppers.dir_invert;
    s_tEnableInvert = settings->steppers.enable_invert;
    s_wStepPulseUs = (uint32_t)settings->steppers.pulse_microseconds;
    if ((float)s_wStepPulseUs < settings->steppers.pulse_microseconds) {
        s_wStepPulseUs++;
    }
    if (s_wStepPulseUs < (uint32_t)hal.step_us_min) {
        s_wStepPulseUs = (uint32_t)hal.step_us_min;
    }
    s_wDirectionDelayUs =
        (uint32_t)settings->steppers.pulse_delay_microseconds;
    if ((float)s_wDirectionDelayUs <
        settings->steppers.pulse_delay_microseconds) {
        s_wDirectionDelayUs++;
    }
}

void grblhal_motion_setup(void)
{
    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR5);
    tmr_base_init(TMR5, 0xFFFFFFFF, 0);
    tmr_32_bit_function_enable(TMR5, TRUE);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR5_GLOBAL_IRQn, 0, 0);
}

void grblhal_stepper_wake_up(void)
{
    tmr_flag_clear(TMR5, TMR_OVF_FLAG);
    tmr_counter_value_set(TMR5, 0);
    tmr_counter_enable(TMR5, TRUE);
}

void grblhal_stepper_go_idle(bool clear_signals)
{
    tmr_counter_enable(TMR5, FALSE);
    if (clear_signals) {
        stepper_write_step((axes_signals_t){0});
        stepper_write_dir((axes_signals_t){0});
    }
}

void grblhal_stepper_enable(axes_signals_t enable, bool hold)
{
    (void)hold;
    bool pin_high = enable.mask == 0;
    if (s_tEnableInvert.mask != 0) {
        pin_high = !pin_high;
    }
    gpio_bits_write(STEPPER_EN_PORT, STEPPER_EN_PIN,
                    pin_high ? TRUE : FALSE);
}

void grblhal_stepper_cycles_per_tick(uint32_t cycles_per_tick)
{
    if (cycles_per_tick < 2u) {
        cycles_per_tick = 2u;
    }
    tmr_period_value_set(TMR5, cycles_per_tick - 1u);
}

void grblhal_stepper_pulse_start(stepper_t *stepper)
{
    if (stepper->dir_changed.value != 0) {
        stepper_write_dir(stepper->dir_out);
        if (stepper->step_out.mask != 0) {
            perfc_delay_us(s_wDirectionDelayUs);
        }
    }
    stepper_write_step(stepper->step_out);
    perfc_delay_us(s_wStepPulseUs);
    stepper_write_step((axes_signals_t){0});
}

void grblhal_stepper_isr(void)
{
    if (hal.stepper.interrupt_callback != NULL) {
        hal.stepper.interrupt_callback();
    }
}

static void dc_spindle_set_state(spindle_ptrs_t *spindle,
                                 spindle_state_t state, float rpm)
{
    (void)spindle;
    (void)rpm;
    s_tSpindleState.value = 0;
    s_tSpindleState.on = state.on;
    gpio_bits_write(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN,
                    state.on ? TRUE : FALSE);
    gpio_bits_reset(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN);
}

static spindle_state_t dc_spindle_get_state(spindle_ptrs_t *spindle)
{
    (void)spindle;
    return s_tSpindleState;
}

bool grblhal_spindle_init(void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
        .cap.enable = On,
        .cap.direction = Off,
        .cap.variable = Off,
        .cap.gpio_controlled = On,
        .set_state = dc_spindle_set_state,
        .get_state = dc_spindle_get_state,
    };

    if (s_chSpindleId < 0) {
        s_chSpindleId = spindle_register(&spindle, "DC spindle");
    }
    return s_chSpindleId >= 0;
}

void grblhal_emergency_stop(void)
{
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, FALSE);
    tmr_counter_enable(TMR5, FALSE);
    gpio_bits_set(STEPPER_EN_PORT, STEPPER_EN_PIN);
    gpio_bits_reset(X_STEP_PORT, X_STEP_PIN);
    gpio_bits_reset(Y_STEP_PORT, Y_STEP_PIN);
    gpio_bits_reset(Z_STEP_PORT, Z_STEP_PIN);
    gpio_bits_reset(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
    gpio_bits_reset(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN);
    s_tSpindleState.value = 0;
}
