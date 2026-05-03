/**
 * @file  halledgpio.c
 * @brief Status LED GPIO driver — PC6, active-low
 */

#include "halledgpio.h"

void halledgpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    LED_GPIO_CLK_EN();

    GPIO_InitStruct.Pin   = LED_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    LED_OFF();
}
