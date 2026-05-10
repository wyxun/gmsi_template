/**
 * @file   halled.c
 * @brief  AT32F413 Motor EVB V1 status LED initialization
 */

#include "at32f413.h"

void halled_Init(void)
{
    gpio_init_type gpio_init_struct;

    /* PB9 — Error LED */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    /* PC13/14/15 — Status LEDs */
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;

    /* Error LED — PB9, off (high) */
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init(GPIOB, &gpio_init_struct);
    GPIOB->scr = GPIO_PINS_9;

    /* Status1 — PC13, off (high) */
    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init(GPIOC, &gpio_init_struct);
    GPIOC->scr = GPIO_PINS_13;

    /* Status2 — PC14, off (high) */
    gpio_init_struct.gpio_pins = GPIO_PINS_14;
    gpio_init(GPIOC, &gpio_init_struct);
    GPIOC->scr = GPIO_PINS_14;

    /* Status3 — PC15, off (high) */
    gpio_init_struct.gpio_pins = GPIO_PINS_15;
    gpio_init(GPIOC, &gpio_init_struct);
    GPIOC->scr = GPIO_PINS_15;
}
