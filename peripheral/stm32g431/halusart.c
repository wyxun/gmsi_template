/**
 * @file  halusart.c
 * @brief USART interrupt-based driver — mringbuf TX/RX, 3 ports
 */

#include "halusart.h"
#include <string.h>

#define USART_RXFLAG_IDLE       0
#define USART_RXFLAG_BUSY       1
#define USART_RXFLAG_FINISH     2
#define USART_TXFLAG_IDLE       0
#define USART_TXFLAG_BUSY       1
#define USART_DELAYTIME         4

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static uint8_t s_chUsart1TxBuf[USART1_TX_BUFFER_MAX];
static uint8_t s_chUsart1RxBuf[USART1_RX_BUFFER_MAX];
static uint8_t s_chUsart2TxBuf[USART2_TX_BUFFER_MAX];
static uint8_t s_chUsart2RxBuf[USART2_RX_BUFFER_MAX];
static uint8_t s_chUsart3TxBuf[USART3_TX_BUFFER_MAX];
static uint8_t s_chUsart3RxBuf[USART3_RX_BUFFER_MAX];

static usartbuffer_t s_tUsart1Buffer;
static usartbuffer_t s_tUsart2Buffer;
static usartbuffer_t s_tUsart3Buffer;

static uint8_t s_chRxByte;

static void MX_USART1_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);

static usartbuffer_t *get_usart_buffer(uint8_t chUsartNum)
{
    switch (chUsartNum) {
        case 0:  return &s_tUsart1Buffer;
        case 1:  return &s_tUsart2Buffer;
        case 2:  return &s_tUsart3Buffer;
        default: return &s_tUsart1Buffer;
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (huart->Instance == USART1) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        __HAL_RCC_USART1_CLK_ENABLE();
        USART1_GPIO_CLK_EN();

        GPIO_InitStruct.Pin       = USART1_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = USART1_GPIO_AF;
        HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = USART1_RX_PIN;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (huart->Instance == USART2) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        __HAL_RCC_USART2_CLK_ENABLE();
        USART2_GPIO_CLK_EN();

        GPIO_InitStruct.Pin       = USART2_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = USART2_GPIO_AF;
        HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = USART2_RX_PIN;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (huart->Instance == USART3) {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        __HAL_RCC_USART3_CLK_ENABLE();
        USART3_GPIO_CLK_EN();

        GPIO_InitStruct.Pin       = USART3_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = USART3_GPIO_AF;
        HAL_GPIO_Init(USART3_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = USART3_RX_PIN;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        HAL_GPIO_Init(USART3_RX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

static void MX_USART1_Init(void)
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = USART1_BAUDRATE;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart1);
    HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart1);
}

static void MX_USART2_Init(void)
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = USART2_BAUDRATE;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart2);
    HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart2);
}

static void MX_USART3_Init(void)
{
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = USART3_BAUDRATE;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart3);
    HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart3);
}

void halusart_Init(void)
{
    MX_USART1_Init();
    MX_USART2_Init();
    MX_USART3_Init();

    mringbuf_Init(&s_tUsart1Buffer.tRXQueue, s_chUsart1RxBuf, USART1_RX_BUFFER_MAX);
    mringbuf_Init(&s_tUsart1Buffer.tTXQueue, s_chUsart1TxBuf, USART1_TX_BUFFER_MAX);
    s_tUsart1Buffer.ptUsart = &huart1;

    mringbuf_Init(&s_tUsart2Buffer.tRXQueue, s_chUsart2RxBuf, USART2_RX_BUFFER_MAX);
    mringbuf_Init(&s_tUsart2Buffer.tTXQueue, s_chUsart2TxBuf, USART2_TX_BUFFER_MAX);
    s_tUsart2Buffer.ptUsart = &huart2;

    mringbuf_Init(&s_tUsart3Buffer.tRXQueue, s_chUsart3RxBuf, USART3_RX_BUFFER_MAX);
    mringbuf_Init(&s_tUsart3Buffer.tTXQueue, s_chUsart3TxBuf, USART3_TX_BUFFER_MAX);
    s_tUsart3Buffer.ptUsart = &huart3;

    HAL_UART_Receive_IT(&huart1, &s_chRxByte, 1);
    HAL_UART_Receive_IT(&huart2, &s_chRxByte, 1);
    HAL_UART_Receive_IT(&huart3, &s_chRxByte, 1);
}

uint16_t halusart_SendData(uint8_t chUsartNum, uint8_t *pchSendData, uint16_t hwLength)
{
    usartbuffer_t *ptBuf = get_usart_buffer(chUsartNum);
    uint16_t hwCounter;

    for (hwCounter = 0; hwCounter < hwLength; hwCounter++) {
        if (mringbuf_Write(&ptBuf->tTXQueue, *(pchSendData + hwCounter)) == 0) {
            return hwCounter;
        }
    }

    if (ptBuf->chTXFlag == USART_TXFLAG_BUSY) {
        return hwCounter;
    }

    if (mringbuf_Read(&ptBuf->tTXQueue, &ptBuf->chTXByte) == 1) {
        HAL_UART_Transmit_IT(ptBuf->ptUsart, &ptBuf->chTXByte, 1);
        ptBuf->chTXFlag = USART_TXFLAG_BUSY;
    }
    return hwCounter;
}

uint16_t halusart_receiveData(uint8_t chUsartNum, uint8_t *pchReceiveData)
{
    usartbuffer_t *ptBuf = get_usart_buffer(chUsartNum);
    uint16_t hwCounter = 0;

    if (ptBuf->chRXFlag != USART_RXFLAG_FINISH) {
        return 0;
    }

    while (mringbuf_Read(&ptBuf->tRXQueue, pchReceiveData) == 1) {
        hwCounter++;
        pchReceiveData++;
    }

    ptBuf->chRXFlag = USART_RXFLAG_IDLE;
    return hwCounter;
}

void halusart_Clock(void)
{
    usartbuffer_t *ptBufList[] = { &s_tUsart1Buffer, &s_tUsart2Buffer, &s_tUsart3Buffer };
    for (int i = 0; i < 3; i++) {
        usartbuffer_t *ptBuf = ptBufList[i];
        if (ptBuf->chRXFlag == USART_RXFLAG_BUSY) {
            if (ptBuf->chRXFinishTime) {
                ptBuf->chRXFinishTime--;
            } else {
                ptBuf->chRXFlag = USART_RXFLAG_FINISH;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    usartbuffer_t *ptBuf = NULL;
    if      (huart == &huart1) ptBuf = &s_tUsart1Buffer;
    else if (huart == &huart2) ptBuf = &s_tUsart2Buffer;
    else if (huart == &huart3) ptBuf = &s_tUsart3Buffer;
    else return;

    mringbuf_Write(&ptBuf->tRXQueue, s_chRxByte);
    ptBuf->chRXFinishTime = USART_DELAYTIME;
    ptBuf->chRXFlag       = USART_RXFLAG_BUSY;

    HAL_UART_Receive_IT(huart, &s_chRxByte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    usartbuffer_t *ptBuf = NULL;
    if      (huart == &huart1) ptBuf = &s_tUsart1Buffer;
    else if (huart == &huart2) ptBuf = &s_tUsart2Buffer;
    else if (huart == &huart3) ptBuf = &s_tUsart3Buffer;
    else return;

    if (mringbuf_Read(&ptBuf->tTXQueue, &ptBuf->chTXByte) == 1) {
        HAL_UART_Transmit_IT(huart, &ptBuf->chTXByte, 1);
    } else {
        ptBuf->chTXFlag = USART_TXFLAG_IDLE;
    }
}
