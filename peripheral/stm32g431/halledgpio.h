/**
 * @file  halledgpio.h
 * @brief Status LED GPIO driver — PC6, active-low
 */

#ifndef __HAL_LED_GPIO_H__
#define __HAL_LED_GPIO_H__

#include "stm32g4xx_hal.h"

#define LED_GPIO_PORT       GPIOC
#define LED_GPIO_PIN        GPIO_PIN_6
#define LED_GPIO_CLK_EN()   __HAL_RCC_GPIOC_CLK_ENABLE()

#define LED_ON()            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET)
#define LED_OFF()           HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET)
#define LED_TOGGLE()        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN)

extern void halledgpio_Init(void);

#endif /* __HAL_LED_GPIO_H__ */
