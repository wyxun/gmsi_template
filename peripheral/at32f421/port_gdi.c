#include "gdi_hw.h"
#include "at32f421.h"

/*============================================================================
 * AT32F421 GPIO 适配 (目前作为示范)
 *===========================================================================*/

static int32_t at32_gpio_Set(void *pPriv, gdi_gpio_level_t eLevel)
{
    /* (void)pPriv;
     * TODO: gpio_bits_write((gpio_type *)pPriv->port, pPriv->pin, (confirm_state)eLevel);
     */
    return 0;
}

static int32_t at32_gpio_Get(void *pPriv)
{
    return GDI_GPIO_LOW;
}

static int32_t at32_gpio_Toggle(void *pPriv)
{
    return 0;
}

/*============================================================================
 * 外设实例（静态分配）
 * 这里将具体的引脚、寄存器通过 pPriv 与操作函数绑定
 *===========================================================================*/

static gdi_gpio_t s_tLedGpio = {
    .pPriv    = NULL, /* 未来可以填入如 GPIOB 及其 Pin 号等私有结构体指针 */
    .fnSet    = at32_gpio_Set,
    .fnGet    = at32_gpio_Get,
    .fnToggle = at32_gpio_Toggle,
};

/*============================================================================
 * AT32F421 USART1 (RS232/Dwin) 适配
 *===========================================================================*/
#define USART1_TX_BUFFER_SIZE  256
#define USART1_RX_BUFFER_SIZE  256

#define USART_RXFLAG_IDLE   0
#define USART_RXFLAG_BUSY   1
#define USART_RXFLAG_FINISH 2

#define USART_TXFLAG_IDLE   0
#define USART_TXFLAG_BUSY   1

#define USART_DELAYTIME     5

static uint8_t s_chUsart1TxBuf[USART1_TX_BUFFER_SIZE];
static uint8_t s_chUsart1RxBuf[USART1_RX_BUFFER_SIZE];

#include "port_gdi.h"

at32_usart_priv_t s_tUsart1Priv = {
    .ptUsart = NULL,
};

void at32_usart1_init(void)
{
    // 如果硬件指针已经绑定了，说明初始化过了，直接返回
    if (s_tUsart1Priv.ptUsart != NULL) { 
        return; 
    }
    at32_usart_init(&s_tUsart1Priv, USART1, 
                    s_chUsart1TxBuf, USART1_TX_BUFFER_SIZE,
                    s_chUsart1RxBuf, USART1_RX_BUFFER_SIZE);
}

/* Generic Initialization */
void at32_usart_init(at32_usart_priv_t *ptPriv, 
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize)
{
    if (NULL == ptPriv || NULL == ptUsart) return;
    
    ptPriv->ptUsart = ptUsart;
    queue_init(&ptPriv->tTxQueue, pchTxBuf, wTxBufSize);
    queue_init(&ptPriv->tRxQueue, pchRxBuf, wRxBufSize);
    ptPriv->chTXFlag = USART_TXFLAG_IDLE;
    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    ptPriv->chRXFinishTime = 0;
}

/* Generic 1ms Timer Logic */
void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv) return;
    if (ptPriv->chRXFlag == USART_RXFLAG_BUSY) {
        if (ptPriv->chRXFinishTime > 0) {
            ptPriv->chRXFinishTime--;
            }
        else {
            ptPriv->chRXFlag = USART_RXFLAG_FINISH;
        }
    }
}

static int32_t at32_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i;

    for (i = 0; i < wLen; i++) {
        if (queue_write(&ptPriv->tTxQueue, pchData[i]) != QUEUE_OK) {
            break;
        }
    }

    if (ptPriv->chTXFlag == USART_TXFLAG_IDLE && !queue_isEmpty(&ptPriv->tTxQueue)) {
        uint8_t chData;
        if (queue_read(&ptPriv->tTxQueue, &chData) == QUEUE_OK) {
            usart_data_transmit(ptPriv->ptUsart, chData);
        ptPriv->chTXFlag = USART_TXFLAG_BUSY;
        }
    }
    return i;
}

static int32_t at32_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i = 0;
    qstatus_t tQueueStatus;

    if (ptPriv->chRXFlag != USART_RXFLAG_FINISH) return 0;

    do {
        tQueueStatus = queue_read(&ptPriv->tRxQueue, &pchBuf[i]);
        if (tQueueStatus == QUEUE_OK) {
            i++;
        }
        if (i >= wLen) {
            break;
        }
    } while (tQueueStatus != QUEUE_EMPTY);

    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    return i;
}

static int32_t at32_stream_IsBusy(void *pPriv)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    return (ptPriv->chTXFlag == USART_TXFLAG_BUSY) ? 1 : 0;
}

static gdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart1Priv,
    .fnWrite  = at32_stream_Write,
    .fnRead   = at32_stream_Read,
    .fnIsBusy = at32_stream_IsBusy,
};

/* Generic IRQ Handler logic */
void at32_usart_irq_handler(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return;
    uint8_t chData;
    usart_type *ptHW = ptPriv->ptUsart;

    if (ptHW->ctrl1_bit.rdbfien != RESET) {
        if (usart_interrupt_flag_get(ptHW, USART_RDBF_FLAG) != RESET) {
            uint8_t ch = usart_data_receive(ptHW);
            queue_write(&ptPriv->tRxQueue, ch);
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag = USART_RXFLAG_BUSY;
        }
    }

    if (ptHW->ctrl1_bit.tdcien != RESET) {
        if (usart_flag_get(ptHW, USART_TDC_FLAG) != RESET) {
            usart_flag_clear(ptHW, USART_TDC_FLAG);
            if (queue_read(&ptPriv->tTxQueue, &chData) == QUEUE_OK) {
                usart_data_transmit(ptHW, chData);
                ptPriv->chTXFlag = USART_TXFLAG_BUSY;
            } else {
                ptPriv->chTXFlag = USART_TXFLAG_IDLE;
            }
        }
    }
}

/*============================================================================
 * 全局硬件资源池实例化
 *===========================================================================*/

const gdi_hardware_t HW = {
    .ptSerial      = &s_tStreamSerial,
};
