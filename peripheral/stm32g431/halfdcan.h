/**
 * @file  halfdcan.h
 * @brief FDCAN1 driver — CAN FD with BRS, interrupt TX/RX
 */

#ifndef __HAL_FDCAN_H__
#define __HAL_FDCAN_H__

#include "stm32g4xx_hal.h"

/* FDCAN1: PA11(RX) / PB9(TX) — AF9 */
#define FDCAN1_RX_PORT          GPIOA
#define FDCAN1_RX_PIN           GPIO_PIN_11
#define FDCAN1_RX_CLK_EN()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define FDCAN1_TX_PORT          GPIOB
#define FDCAN1_TX_PIN           GPIO_PIN_9
#define FDCAN1_TX_CLK_EN()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define FDCAN1_GPIO_AF          GPIO_AF9_FDCAN1

#define CAN_MSG_QUEUE_LENGTH    8

typedef struct {
    uint32_t wId;
    uint8_t  chLength;
    uint8_t  chData[64];
} can_msg_t;

typedef struct {
    can_msg_t tMsg[CAN_MSG_QUEUE_LENGTH];
    uint8_t   chWriteIdx;
    uint8_t   chReadIdx;
} can_msg_queue_t;

extern void     halfdcan_Init(void);
extern void     halfdcan_Send(uint32_t wId, uint8_t *pchData, uint16_t hwSize);
extern void     halfdcan_PushQueue(uint32_t wId, uint8_t *pchData, uint16_t hwSize);
extern void     halfdcan_PullQueue(void);
extern uint32_t halfdcan_Recv(can_msg_t *ptMsg);

extern FDCAN_HandleTypeDef hfdcan1;

#endif /* __HAL_FDCAN_H__ */
