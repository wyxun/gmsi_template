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

/* ---- PWM spindle — TMR3 @ 120 MHz ---- */
static spindle_pwm_t s_tSpindlePwm = {0};
static spindle_pwm_settings_t s_tSpindlePwmSettings = {0};

static uint_fast16_t pwm_spindle_get_pwm(spindle_ptrs_t *spindle, float rpm)
{
    return s_tSpindlePwm.compute_value(&s_tSpindlePwm, rpm, false);
}

static void pwm_spindle_update_pwm(spindle_ptrs_t *spindle, uint_fast16_t pwm)
{
    TMR3->c1dt = pwm;
}

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

/* ---- FG pulse tracking & Alarm helpers ---- */
static volatile uint32_t s_wFgPulseCount = 0;
static volatile uint32_t s_wLastFgPulseTicks = 0;

uint32_t grblhal_spindle_get_fg_count(void)
{
    return s_wFgPulseCount;
}

void grblhal_spindle_reset_fg_count(void)
{
    s_wFgPulseCount = 0;
    s_wLastFgPulseTicks = grblhal_get_ticks();
}

uint32_t grblhal_spindle_get_fg_idle_time_ms(void)
{
    return grblhal_get_ticks() - s_wLastFgPulseTicks;
}

bool grblhal_spindle_get_alarm(void)
{
    return gpio_input_data_bit_read(SPINDLE_ALARM_PORT, SPINDLE_ALARM_PIN) == RESET;
}

void grblhal_spindle_fg_isr(void)
{
    if (exint_flag_get(EXINT_LINE_1) != RESET) {
        s_wFgPulseCount++;
        s_wLastFgPulseTicks = grblhal_get_ticks();
        exint_flag_clear(EXINT_LINE_1);
    }
}

static void dc_spindle_set_state(spindle_ptrs_t *spindle,
                                 spindle_state_t state, float rpm)
{
    s_tSpindleState.value = 0;
    s_tSpindleState.on = state.on;
    s_tSpindleState.ccw = state.ccw;

    /* PA5 (DIR): LOW = CW, HIGH = CCW */
    gpio_bits_write(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN,
                    state.on && state.ccw ? TRUE : FALSE);
    /* PB0 (EN): HIGH = Enabled, LOW = Disabled */
    gpio_bits_write(SPINDLE_EN_PORT, SPINDLE_EN_PIN,
                    state.on ? TRUE : FALSE);
    pwm_spindle_update_pwm(spindle, state.on ? spindle->get_pwm(spindle, rpm) : 0);
}

static spindle_state_t dc_spindle_get_state(spindle_ptrs_t *spindle)
{
    (void)spindle;
    return s_tSpindleState;
}

static bool grblhal_spindle_config(spindle_ptrs_t *spindle)
{
    (void)spindle;

    /* Enable GPIO and TMR3 clocks */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

    /* PA5 — Spindle Direction Output */
    gpio_init_struct.gpio_pins = SPINDLE_DIR_PIN;
    gpio_init(SPINDLE_DIR_PORT, &gpio_init_struct);
    gpio_bits_reset(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN);

    /* PB0 — Spindle Enable Output */
    gpio_init_struct.gpio_pins = SPINDLE_EN_PIN;
    gpio_init(SPINDLE_EN_PORT, &gpio_init_struct);
    gpio_bits_reset(SPINDLE_EN_PORT, SPINDLE_EN_PIN);

    /* PC5 — Spindle Alarm Input (Pull-up) */
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_pins = SPINDLE_ALARM_PIN;
    gpio_init(SPINDLE_ALARM_PORT, &gpio_init_struct);

    /* PB1 — Spindle FG Pulse Input (EXTI Line 1) */
    gpio_init_struct.gpio_pins = SPINDLE_FG_PIN;
    gpio_init(SPINDLE_FG_PORT, &gpio_init_struct);

    gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOB, GPIO_PINS_SOURCE1);

    exint_init_type exint_init_struct;
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_select = EXINT_LINE_1;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    exint_init_struct.line_enable = TRUE;
    exint_init(&exint_init_struct);

    nvic_irq_enable(EXINT1_IRQn, 1, 0);

    s_tSpindlePwmSettings = (spindle_pwm_settings_t){
        .rpm_max               = 10000.0f,
        .rpm_min               = 0.0f,
        .pwm_freq              = 5000.0f,
        .pwm_off_value         = 0.0f,
        .pwm_min_value         = 0.0f,
        .pwm_max_value         = 100.0f,
        .flags.pwm_disable     = false,
        .flags.enable_rpm_controlled = false,
        .flags.laser_mode_disable    = true,
        .flags.pwm_ramped            = true,
        .flags.ignore_delays         = false,
    };

    spindle_precompute_pwm_values(spindle, &s_tSpindlePwm, &s_tSpindlePwmSettings, 120000000UL);

    tmr_reset(TMR3);
    tmr_base_init(TMR3, s_tSpindlePwm.period - 1, 0);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);

    static tmr_output_config_type tmr_oc;
    tmr_output_default_para_init(&tmr_oc);
    tmr_oc.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc.oc_idle_state   = FALSE;
    tmr_oc.oc_polarity     = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_oc.oc_output_state = TRUE;
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_1, &tmr_oc);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, 0);
    tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_counter_enable(TMR3, TRUE);

    return true;
}

bool grblhal_spindle_init(void)
{
    static const spindle_ptrs_t spindle = {
        .type       = SpindleType_PWM,
        .cap.enable = On,
        .cap.direction = Off,
        .cap.variable  = On,
        .cap.gpio_controlled = On,
        .config      = grblhal_spindle_config,
        .set_state   = dc_spindle_set_state,
        .get_state   = dc_spindle_get_state,
        .get_pwm     = pwm_spindle_get_pwm,
        .update_pwm  = pwm_spindle_update_pwm,
    };

    if (s_chSpindleId < 0) {
        s_chSpindleId = spindle_register(&spindle, "PWM Spindle");
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
    gpio_bits_reset(SPINDLE_EN_PORT, SPINDLE_EN_PIN);
    gpio_bits_reset(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN);
    TMR3->c1dt = 0; /* Drive PWM duty to 0 (PA6 is TMR3_CH1) */
    s_tSpindleState.value = 0;
}

