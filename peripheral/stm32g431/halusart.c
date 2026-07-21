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

static uint8_t s_chUsart1RxByte;
static uint8_t s_chUsart2RxByte;
static uint8_t s_chUsart3RxByte;

static void MX_USART2_Init(void);

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

        HAL_NVIC_SetPriority(USART2_IRQn, 4, 0);
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

void halusart_Init(void)
{
    MX_USART2_Init();

    mringbuf_Init(&s_tUsart2Buffer.tRXQueue, s_chUsart2RxBuf, USART2_RX_BUFFER_MAX);
    mringbuf_Init(&s_tUsart2Buffer.tTXQueue, s_chUsart2TxBuf, USART2_TX_BUFFER_MAX);
    s_tUsart2Buffer.ptUsart = &huart2;
    s_tUsart2Buffer.chTXFlag = USART_TXFLAG_IDLE;
    s_tUsart2Buffer.chRXFlag = USART_RXFLAG_IDLE;
    s_tUsart2Buffer.chRXFinishTime = 0;

    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
}

uint16_t halusart_SendData(uint8_t chUsartNum, uint8_t *pchSendData, uint16_t hwLength)
{
    usartbuffer_t *ptBuf = get_usart_buffer(chUsartNum);
    uint16_t hwCounter;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (hwCounter = 0; hwCounter < hwLength; hwCounter++) {
        if (mringbuf_Write(&ptBuf->tTXQueue, *(pchSendData + hwCounter)) == 0) {
            break;
        }
    }

    if (ptBuf->chTXFlag == USART_TXFLAG_IDLE) {
        uint8_t chData;
        if (mringbuf_Read(&ptBuf->tTXQueue, &chData) == 1) {
            ptBuf->ptUsart->Instance->TDR = chData;
            ptBuf->chTXFlag = USART_TXFLAG_BUSY;
            __HAL_UART_ENABLE_IT(ptBuf->ptUsart, UART_IT_TC);
        }
    }

    __set_PRIMASK(primask);
    return hwCounter;
}

uint16_t halusart_receiveData(uint8_t chUsartNum, uint8_t *pchReceiveData)
{
    usartbuffer_t *ptBuf = get_usart_buffer(chUsartNum);
    uint16_t hwCounter = 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (ptBuf->chRXFlag != USART_RXFLAG_FINISH) {
        __set_PRIMASK(primask);
        return 0;
    }

    while (mringbuf_Read(&ptBuf->tRXQueue, &pchReceiveData[hwCounter]) == 1) {
        hwCounter++;
    }

    if (mringbuf_GetUsed(&ptBuf->tRXQueue) == 0) {
        ptBuf->chRXFlag = USART_RXFLAG_IDLE;
    }

    __set_PRIMASK(primask);
    return hwCounter;
}

void halusart_Clock(void)
{
    usartbuffer_t *ptBufList[] = { &s_tUsart1Buffer, &s_tUsart2Buffer, &s_tUsart3Buffer };
    for (int i = 0; i < 3; i++) {
        usartbuffer_t *ptBuf = ptBufList[i];

        if (ptBuf->chRXFlag == USART_RXFLAG_BUSY) {
            if (ptBuf->chRXFinishTime > 0) {
                ptBuf->chRXFinishTime--;
            } else {
                ptBuf->chRXFlag = USART_RXFLAG_FINISH;
            }
        }
    }
}

void halusart_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        usartbuffer_t *ptPriv = &s_tUsart2Buffer;
        USART_TypeDef *ptHW = huart->Instance;
        uint32_t isr = ptHW->ISR;

        // 硬件中断中：瞬间清空可能出现的错误（ORE / FE / NE / PE）
        if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
            ptHW->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
        }

        // 接收寄存器非空中断
        if ((ptHW->CR1 & USART_CR1_RXNEIE) && (isr & USART_ISR_RXNE)) {
            uint8_t ch = (uint8_t)(ptHW->RDR & 0xFFU);
            mringbuf_Write(&ptPriv->tRXQueue, ch);
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag = USART_RXFLAG_BUSY;
        }

        // 发送完成中断
        if ((ptHW->CR1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
            ptHW->ICR = USART_ICR_TCCF; // 清除 TC 中断标志
            uint8_t chData;
            if (mringbuf_Read(&ptPriv->tTXQueue, &chData) == 1) {
                ptHW->TDR = chData;
                ptPriv->chTXFlag = USART_TXFLAG_BUSY;
            } else {
                ptPriv->chTXFlag = USART_TXFLAG_IDLE;
                __HAL_UART_DISABLE_IT(huart, UART_IT_TC); // 队列空，关闭 TC 中断允许
            }
        }
    }
}
