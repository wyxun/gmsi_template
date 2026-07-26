/**
 * @file   port_mdi.h
 * @brief  STM32G431 MDI 端口（声明外设句柄）
 */

#ifndef __PORT_MDI_H__
#define __PORT_MDI_H__

#include "stm32g4xx_hal.h"

/* USART1 句柄（由 port_sys.c 定义，stm32g4xx_it.c 引用） */
extern UART_HandleTypeDef huart1;

/* 三相预装载原子组：一次调用写完 TIM1 三个 CCR */
int32_t port_mdi_MotorPwmSetDuty3(uint32_t wDutyU, uint32_t wDutyV, uint32_t wDutyW);

#endif /* __PORT_MDI_H__ */
