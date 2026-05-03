/**
 * @file   port_gdi.h
 * @brief  STM32G431 GDI 端口（声明外设句柄）
 */

#ifndef __PORT_GDI_H__
#define __PORT_GDI_H__

#include "stm32g4xx_hal.h"

/* USART1 句柄（由 port_sys.c 定义，stm32g4xx_it.c 引用） */
extern UART_HandleTypeDef huart1;

#endif /* __PORT_GDI_H__ */
