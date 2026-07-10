/**
 * @file   halusart.c
 * @brief  AT32F413 USART1 initialization (PB6/7 remap, debug shell)
 */

#include "at32f413.h"
#include "halusart.h"

#define USART1_BAUDRATE  115200

void halusart_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* USART1 TX — PB6 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins      = GPIO_PINS_6;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* USART1 RX — PB7 */
    gpio_init_struct.gpio_pins = GPIO_PINS_7;
    gpio_init(GPIOB, &gpio_init_struct);

    /* Remap USART1 to PB6/7: USART1_GMUX_0001 */
    gpio_pin_remap_config(USART1_GMUX_0001, TRUE);

    nvic_irq_enable(USART1_IRQn, 8, 0);

    usart_init(USART1, USART1_BAUDRATE, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);

    usart_flag_clear(USART1, USART_TDBE_FLAG);
    usart_flag_clear(USART1, USART_TDC_FLAG);
    usart_flag_clear(USART1, USART_RDBF_FLAG);
    usart_interrupt_enable(USART1, USART_TDC_INT, TRUE);

    /* Bind USART1 to MDI stream instance */
    extern void at32_usart1_init(void);
    at32_usart1_init();
}
