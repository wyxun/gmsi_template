/**
 * @file   port_mdi.c
 * @brief  AT32F407 MDI adapter — binds peripheral hardware to MDI objects
 *
 * USART1 interrupt-driven TX/RX ringbuf (same pattern as AT32F413).
 * GPIO wrappers for status LED.
 */

#include "mdi_hw.h"
#include "at32f403a_407.h"
#include "mdi/mdi.h"

#include "port_mdi.h"

/*============================================================================
 * AT32F407 USART1 adapter (same ringbuf pattern as AT32F413)
 *===========================================================================*/
#define USART1_TX_BUFFER_SIZE  2048
#define USART1_RX_BUFFER_SIZE  256

#define USART_RXFLAG_IDLE   0
#define USART_RXFLAG_BUSY   1
#define USART_RXFLAG_FINISH 2

#define USART_TXFLAG_IDLE   0
#define USART_TXFLAG_BUSY   1

#define USART_DELAYTIME     5

static uint8_t s_chUsart1TxBuf[USART1_TX_BUFFER_SIZE];
static uint8_t s_chUsart1RxBuf[USART1_RX_BUFFER_SIZE];

/* ---- GPIO wrappers ---- */

static int32_t at32_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    gpio_bits_write((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1], (confirm_state)eLevel);
    return 0;
}

static int32_t at32_gpio_Get(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
}

static int32_t at32_gpio_Toggle(void *pPriv)
{
    void **ap = (void **)pPriv;
    gpio_bits_toggle((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1]);
    return 0;
}

static int32_t at32_gpio_Get_ActiveLow(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_LOW : MDI_GPIO_HIGH;
}

/* ---- LED GPIO instance (active-low, PD13 = AT-START-F407 LED2) ---- */

static void *s_apvLedStatPriv[] = { GPIOD, (void *)(uintptr_t)GPIO_PINS_13 };
static mdi_gpio_t s_tLedStatus = {
    .pPriv = s_apvLedStatPriv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};

/* ---- USART1 ringbuf logic (reused from AT32F413) ---- */

at32_usart_priv_t s_tUsart1Priv = { .ptUsart = NULL };

void at32_usart1_init(void)
{
    if (s_tUsart1Priv.ptUsart != NULL) return;
    at32_usart_init(&s_tUsart1Priv, USART1,
                    s_chUsart1TxBuf, USART1_TX_BUFFER_SIZE,
                    s_chUsart1RxBuf, USART1_RX_BUFFER_SIZE);
}

void at32_usart_init(at32_usart_priv_t *ptPriv,
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize)
{
    if (NULL == ptPriv || NULL == ptUsart) return;

    ptPriv->ptUsart = ptUsart;
    mringbuf_Init(&ptPriv->tTxQueue, pchTxBuf, (uint16_t)wTxBufSize);
    mringbuf_Init(&ptPriv->tRxQueue, pchRxBuf, (uint16_t)wRxBufSize);
    ptPriv->chTXFlag = USART_TXFLAG_IDLE;
    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    ptPriv->chRXFinishTime = 0;
}

void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv)
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

static int32_t at32_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return 0;

    usart_type *ptHW = ptPriv->ptUsart;
    uint32_t i;

    for (i = 0; i < wLen; i++) {
        /* 直接轮询 TDBE 硬件标志，确保 TDR 为空后再写入 */
        while (usart_flag_get(ptHW, USART_TDBE_FLAG) == RESET) {}
        usart_data_transmit(ptHW, pchData[i]);
    }

    return i;
}

static int32_t at32_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i = 0;
    uint16_t hwRead;

    if (ptPriv->chRXFlag != USART_RXFLAG_FINISH) return 0;

    do {
        hwRead = mringbuf_Read(&ptPriv->tRxQueue, &pchBuf[i]);
        if (hwRead > 0) i++;
        if (i >= wLen) break;
    } while (hwRead > 0);

    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    return i;
}

static int32_t at32_stream_IsBusy(void *pPriv)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    return (ptPriv->chTXFlag == USART_TXFLAG_BUSY) ? 1 : 0;
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart1Priv,
    .fnWrite  = at32_stream_Write,
    .fnRead   = at32_stream_Read,
    .fnIsBusy = at32_stream_IsBusy,
};

void at32_usart_irq_handler(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return;
    uint8_t chData;
    usart_type *ptHW = ptPriv->ptUsart;

    /* 清除可能发生的溢出与帧错误，防止接收锁死 */
    if (usart_flag_get(ptHW, USART_ROERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_FERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_NERR_FLAG) != RESET ||
        usart_flag_get(ptHW, USART_PERR_FLAG) != RESET) {
        usart_flag_clear(ptHW, USART_ROERR_FLAG);
        usart_flag_clear(ptHW, USART_FERR_FLAG);
        usart_flag_clear(ptHW, USART_NERR_FLAG);
        usart_flag_clear(ptHW, USART_PERR_FLAG);
        (void)usart_data_receive(ptHW);
    }

    if (ptHW->ctrl1_bit.rdbfien != RESET) {
        if (usart_flag_get(ptHW, USART_RDBF_FLAG) != RESET) {
            uint8_t ch = usart_data_receive(ptHW);
            extern bool protocol_enqueue_realtime_command(uint8_t c);
            if (!protocol_enqueue_realtime_command(ch)) {
                mringbuf_Write(&ptPriv->tRxQueue, ch);
            }
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag = USART_RXFLAG_BUSY;
        }
    }


    /* 发送方向：直接轮询发送，无需 TDBE 中断，保留此段注释供将来恢复中断驱动 */
}

/*============================================================================
 * Global hardware resource pool
 *===========================================================================*/

const mdi_hardware_t HW = {
    .ptLedStatus = &s_tLedStatus,   /* PD13 — primary heartbeat LED */
    .ptSerial    = &s_tStreamSerial,
};
