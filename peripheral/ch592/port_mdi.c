#include "mdi_hw.h"
#include "port_mdi.h"
#include "CH59x_common.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* GPIO LED (PA8) 对象驱动 */
static int32_t ch592_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    if (eLevel == MDI_GPIO_HIGH) {
        GPIOA_SetBits(GPIO_Pin_8);   /* 灭 */
    } else {
        GPIOA_ResetBits(GPIO_Pin_8); /* 亮 */
    }
    return 0;
}

static int32_t ch592_gpio_Get(void *pPriv)
{
    return (R32_PA_PIN & (1 << 8)) ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
}

static int32_t ch592_gpio_Toggle(void *pPriv)
{
    GPIOA_InverseBits(GPIO_Pin_8);
    return 0;
}

static mdi_gpio_t s_tLedGpio = {
    .pPriv    = NULL,
    .fnSet    = ch592_gpio_Set,
    .fnGet    = ch592_gpio_Get,
    .fnToggle = ch592_gpio_Toggle,
};

ch592_usart_priv_t s_tUsart1Priv;
static uint8_t s_chUsart1TxBuf[1024];
static uint8_t s_chUsart1RxBuf[1024];

void ch592_usart_init(uint32_t baudrate)
{
    /* 1. GPIO 引脚配置 (UART1: PA8=RXD, PA9=TXD) */
    GPIOA_SetBits(GPIO_Pin_9);
    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    
    /* 2. 串口基础硬件初始化（基于官方标准外设库） */
    UART1_DefInit();
    UART1_BaudRateCfg(baudrate);

    /* 3. 驱动级 RingBuffer 与状态标记初始化 */
    mringbuf_Init(&s_tUsart1Priv.tTxQueue, s_chUsart1TxBuf, sizeof(s_chUsart1TxBuf));
    mringbuf_Init(&s_tUsart1Priv.tRxQueue, s_chUsart1RxBuf, sizeof(s_chUsart1RxBuf));
    
    s_tUsart1Priv.chTXFlag = USART_TXFLAG_IDLE;
    s_tUsart1Priv.chRXFlag = USART_RXFLAG_IDLE;
    s_tUsart1Priv.chRXFinishTime = 0;

    /* 4. 使能串口的中断触发（接收就绪与发送空闲、线路状态） */
    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_THR_EMPTY | RB_IER_LINE_STAT);

    /* 5. 开启中断控制器对应的 UART1 中断请求 */
    PFIC_EnableIRQ(UART1_IRQn);
}

void ch592_usart_timer_1ms(ch592_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv) return;
    if (ptPriv->chRXFlag == USART_RXFLAG_BUSY) {
        if (ptPriv->chRXFinishTime > 0) {
            ptPriv->chRXFinishTime--;
        } else {
            ptPriv->chRXFlag = USART_RXFLAG_FINISH;
        }
    }
}

/* UART0 流传输对象驱动 — 中断环形缓冲区模式 */
static int32_t ch592_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    ch592_usart_priv_t *ptPriv = (ch592_usart_priv_t *)pPriv;
    uint32_t i;

    if (!pchData || !ptPriv) return 0;

    for (i = 0; i < wLen; i++) {
        if (mringbuf_Write(&ptPriv->tTxQueue, pchData[i]) == 0) {
            break;
        }
    }

    /* 数据全放进队列后，如果当前发送是空闲的，启动第一次发送 */
    if (ptPriv->chTXFlag == USART_TXFLAG_IDLE && mringbuf_GetUsed(&ptPriv->tTxQueue) > 0) {
        uint8_t chData;
        if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
            UART1_SendByte(chData);
            ptPriv->chTXFlag = USART_TXFLAG_BUSY;
        }
    }
    return i;
}

static int32_t ch592_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    ch592_usart_priv_t *ptPriv = (ch592_usart_priv_t *)pPriv;
    uint32_t i = 0;

    if (!pchBuf || !ptPriv) return 0;

    while (i < wLen) {
        if (mringbuf_Read(&ptPriv->tRxQueue, &pchBuf[i]) == 1) {
            i++;
        } else {
            break;
        }
    }

    /* 当队列被读空时，重置接收状态（可选，用于保持状态机干净） */
    if (mringbuf_GetUsed(&ptPriv->tRxQueue) == 0) {
        ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    }

    return i;
}

static int32_t ch592_stream_IsBusy(void *pPriv)
{
    ch592_usart_priv_t *ptPriv = (ch592_usart_priv_t *)pPriv;
    return (ptPriv->chTXFlag == USART_TXFLAG_BUSY) ? 1 : 0;
}

void ch592_usart_irq_handler(ch592_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv) return;

    switch(UART1_GetITFlag())
    {
        case UART_II_LINE_STAT:
        {
            UART1_GetLinSTA();
            break;
        }

        case UART_II_RECV_RDY:
        case UART_II_RECV_TOUT:
        {
            while (R8_UART1_RFC) {
                uint8_t ch = UART1_RecvByte();
                mringbuf_Write(&ptPriv->tRxQueue, ch);
            }
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag       = USART_RXFLAG_BUSY;
            break;
        }

        case UART_II_THR_EMPTY:
        {
            uint8_t chData;
            if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
                UART1_SendByte(chData);
                ptPriv->chTXFlag = USART_TXFLAG_BUSY;
            } else {
                ptPriv->chTXFlag = USART_TXFLAG_IDLE;
            }
            break;
        }

        case UART_II_MODEM_CHG:
        default:
            break;
    }
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart1Priv,
    .fnWrite  = ch592_stream_Write,
    .fnRead   = ch592_stream_Read,
    .fnIsBusy = ch592_stream_IsBusy,
};

/* 静态注册 */
const mdi_hardware_t HW = {
    .ptLedStatus   = &s_tLedGpio,
    .ptSerial      = &s_tStreamSerial,
};
