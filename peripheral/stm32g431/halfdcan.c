/**
 * @file  halfdcan.c
 * @brief FDCAN1 driver — CAN FD with BRS, interrupt TX/RX
 */

#include "halfdcan.h"
#include <string.h>

FDCAN_HandleTypeDef hfdcan1;

static FDCAN_TxHeaderTypeDef s_tTxHeader = {
    .TxFrameType        = FDCAN_DATA_FRAME,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch       = FDCAN_BRS_ON,
    .FDFormat            = FDCAN_FD_CAN,
    .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
    .MessageMarker       = 0,
};

static FDCAN_RxHeaderTypeDef s_tRxHeader;
static uint8_t s_chRxData[64];

static can_msg_queue_t s_tTxQueue = { .chWriteIdx = 0, .chReadIdx = 0 };
static uint8_t s_chCanSendBusy = 0;

static void MX_FDCAN1_Init(void);

static uint32_t get_fdcan_dlc(uint16_t hwSize)
{
    if (hwSize <= 8)  return (uint32_t)hwSize << 16;
    if (hwSize <= 12) return FDCAN_DLC_BYTES_12;
    if (hwSize <= 16) return FDCAN_DLC_BYTES_16;
    if (hwSize <= 20) return FDCAN_DLC_BYTES_20;
    if (hwSize <= 24) return FDCAN_DLC_BYTES_24;
    if (hwSize <= 32) return FDCAN_DLC_BYTES_32;
    if (hwSize <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (hfdcan->Instance == FDCAN1) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        __HAL_RCC_FDCAN_CLK_ENABLE();
        FDCAN1_RX_CLK_EN();
        FDCAN1_TX_CLK_EN();

        GPIO_InitStruct.Pin       = FDCAN1_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = FDCAN1_GPIO_AF;
        HAL_GPIO_Init(FDCAN1_RX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = FDCAN1_TX_PIN;
        HAL_GPIO_Init(FDCAN1_TX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }
}

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 5;
    hfdcan1.Init.NominalTimeSeg2      = 2;
    hfdcan1.Init.DataPrescaler        = 4;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 2;
    hfdcan1.Init.DataTimeSeg2         = 1;
    hfdcan1.Init.StdFiltersNbr        = 0;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    HAL_FDCAN_Init(&hfdcan1);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0);

    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
        hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

    HAL_FDCAN_Start(&hfdcan1);
}

void halfdcan_Init(void)
{
    MX_FDCAN1_Init();
}

void halfdcan_Send(uint32_t wId, uint8_t *pchData, uint16_t hwSize)
{
    s_tTxHeader.Identifier = wId;
    s_tTxHeader.IdType     = (wId > 0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    s_tTxHeader.DataLength = get_fdcan_dlc(hwSize);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &s_tTxHeader, pchData);
}

void halfdcan_PushQueue(uint32_t wId, uint8_t *pchData, uint16_t hwSize)
{
    if (s_chCanSendBusy == 0) {
        halfdcan_Send(wId, pchData, hwSize);
        s_chCanSendBusy = 1;
    } else {
        can_msg_t *ptMsg = &s_tTxQueue.tMsg[s_tTxQueue.chWriteIdx];
        ptMsg->wId       = wId;
        ptMsg->chLength  = (uint8_t)hwSize;
        memcpy(ptMsg->chData, pchData, hwSize);
        s_tTxQueue.chWriteIdx = (s_tTxQueue.chWriteIdx + 1) % CAN_MSG_QUEUE_LENGTH;
    }
}

void halfdcan_PullQueue(void)
{
    if (s_tTxQueue.chReadIdx != s_tTxQueue.chWriteIdx) {
        can_msg_t *ptMsg = &s_tTxQueue.tMsg[s_tTxQueue.chReadIdx];
        halfdcan_Send(ptMsg->wId, ptMsg->chData, ptMsg->chLength);
        s_tTxQueue.chReadIdx = (s_tTxQueue.chReadIdx + 1) % CAN_MSG_QUEUE_LENGTH;
    } else {
        s_chCanSendBusy = 0;
    }
}

uint32_t halfdcan_Recv(can_msg_t *ptMsg)
{
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &s_tRxHeader, s_chRxData) != HAL_OK) {
        return 0;
    }
    if (ptMsg != NULL) {
        ptMsg->wId      = s_tRxHeader.Identifier;
        ptMsg->chLength = (uint8_t)(s_tRxHeader.DataLength >> 16);
        memcpy(ptMsg->chData, s_chRxData, ptMsg->chLength);
    }
    return s_tRxHeader.DataLength;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    can_msg_t tMsg;
    if (halfdcan_Recv(&tMsg) != 0) {
        /* TODO: process received CAN FD message */
    }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    halfdcan_PullQueue();
}
