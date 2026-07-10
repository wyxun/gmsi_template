/**
 * @file   halpwm.c
 * @brief  AT32F413 TMR1 3-phase complementary PWM + ADC trigger (Motor EVB V1)
 *
 * Pin mapping (AT_MOTOR_EVB_V1):
 *   PA8  — TMR1 CH1  (U high-side)
 *   PA9  — TMR1 CH2  (V high-side)
 *   PA10 — TMR1 CH3  (W high-side)
 *   PB13 — TMR1 CH1N (U low-side)
 *   PB14 — TMR1 CH2N (V low-side)
 *   PB15 — TMR1 CH3N (W low-side)
 *   PB12 — TMR1 BKIN (brake input)
 *   PA11 — TMR1 CH4  (ADC trigger output)
 */

#include "at32f413.h"
#include "halpwm.h"

void halpwm_Init(void)
{
    gpio_init_type gpio_init_struct = {0};
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_config_struct = {0};

    /* Enable clocks */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /* ---- GPIO: High-side PA8/9/10 ---- */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull      = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);

    /* ---- GPIO: Low-side PB13/14/15 ---- */
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_14;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_15;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- GPIO: Brake input PB12 ---- */
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- GPIO: ADC trigger output PA11 ---- */
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init(GPIOA, &gpio_init_struct);

    /* ---- TMR1 base config: center-aligned, 3-shunt ---- */
    tmr_repetition_counter_set(TMR1, 1);  /* ISR on underflow (high-side PWM on) */
    tmr_base_init(TMR1, PWM_PERIOD, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_1);
    tmr_clock_source_div_set(TMR1, DEADTIME_CLK_DIV);

    /* ---- CH1/2/3 initial duty: 50% ---- */
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, HALF_PWM_PERIOD);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, HALF_PWM_PERIOD);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, HALF_PWM_PERIOD);

    /* ---- CH1/2/3 output config: complementary PWM mode A ---- */
    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity     = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state   = FALSE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.occ_polarity    = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_idle_state  = FALSE;

    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);

    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);

    /* ---- CH4: ADC trigger output ---- */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, HALF_PWM_PERIOD / 2U);

    /* ---- Brake / dead-time ---- */
    tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
    tmr_brkdt_config_struct.brk_enable        = TRUE;
    tmr_brkdt_config_struct.auto_output_enable = FALSE;
    tmr_brkdt_config_struct.deadtime          = DEADTIME_VAL;
    tmr_brkdt_config_struct.fcsodis_state     = TRUE;
    tmr_brkdt_config_struct.fcsoen_state      = TRUE;
    tmr_brkdt_config_struct.brk_polarity      = TMR_BRK_INPUT_ACTIVE_LOW;
    tmr_brkdt_config_struct.wp_level          = TMR_WP_OFF;
    tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);

    /* ---- Primary overflow as output ---- */
    tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);

    /* ---- Clear flags, enable interrupts ---- */
    tmr_flag_clear(TMR1, TMR_OVF_FLAG | TMR_BRK_FLAG | TMR_C4_INT);
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
    tmr_interrupt_enable(TMR1, TMR_BRK_INT, TRUE);

    /* ---- NVIC: TMR1 BRK priority 0, OVF priority 2 ---- */
    nvic_irq_enable(TMR1_BRK_TMR9_IRQn, 0, 0);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 2, 0);

    tmr_one_cycle_mode_enable(TMR1, FALSE);
    tmr_output_enable(TMR1, TRUE);
}

void halpwm_Start(void)
{
    tmr_counter_enable(TMR1, TRUE);
}

void halpwm_Stop(void)
{
    tmr_counter_enable(TMR1, FALSE);
}

void halpwm_SetDuty(uint32_t wDutyU, uint32_t wDutyV, uint32_t wDutyW)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, wDutyU);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, wDutyV);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, wDutyW);
}
