/**
 * @file  halusart.h
 * @brief USART interrupt-based driver — queue TX/RX, 3 ports
 */

#ifndef __HAL_USART_H__
#define __HAL_USART_H__

#include "stm32g4xx_hal.h"
#include "util_queue.h"

/* USART1: PB6(TX) / PB7(RX) — AF7 */
#define USART1_TX_PORT          GPIOB
#define USART1_TX_PIN           GPIO_PIN_6
#define USART1_RX_PORT          GPIOB
#define USART1_RX_PIN           GPIO_PIN_7
#define USART1_GPIO_AF          GPIO_AF7_USART1
#define USART1_GPIO_CLK_EN()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART1_BAUDRATE         115200UL

/* USART2: PB3(TX) / PB4(RX) — AF7 */
#define USART2_TX_PORT          GPIOB
#define USART2_TX_PIN           GPIO_PIN_3
#define USART2_RX_PORT          GPIOB
#define USART2_RX_PIN           GPIO_PIN_4
#define USART2_GPIO_AF          GPIO_AF7_USART2
#define USART2_GPIO_CLK_EN()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART2_BAUDRATE         1843200UL

/* USART3: PB10(TX) / PB11(RX) — AF7 */
#define USART3_TX_PORT          GPIOB
#define USART3_TX_PIN           GPIO_PIN_10
#define USART3_RX_PORT          GPIOB
#define USART3_RX_PIN           GPIO_PIN_11
#define USART3_GPIO_AF          GPIO_AF7_USART3
#define USART3_GPIO_CLK_EN()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART3_BAUDRATE         115200UL

#define USART1_TX_BUFFER_MAX    128
#define USART1_RX_BUFFER_MAX    128
#define USART2_TX_BUFFER_MAX    64
#define USART2_RX_BUFFER_MAX    64
#define USART3_TX_BUFFER_MAX    64
#define USART3_RX_BUFFER_MAX    64

typedef struct {
    UART_HandleTypeDef *ptUsart;
    util_queue_t        tRXQueue;
    uint8_t             chRXFinishTime;
    uint8_t             chRXFlag;
    util_queue_t        tTXQueue;
    uint8_t             chTXFinishTime;
    uint8_t             chTXFlag;
} usartbuffer_t;

extern void     halusart_Init(void);
extern uint16_t halusart_SendData(uint8_t chUsartNum, uint8_t *pchSendData, uint16_t hwLength);
extern uint16_t halusart_receiveData(uint8_t chUsartNum, uint8_t *pchReceiveData);
extern void     halusart_Clock(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#endif /* __HAL_USART_H__ */
