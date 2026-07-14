/**
 * @file   grblhal_stream.c
 * @brief  grblHAL stream I/O — RTT (STM32G431) or MDI UART (AT32F407)
 */

#include "grblhal_driver.h"

#ifdef GRBLHAL_STREAM_UART

/* =========================================================================
 *  AT32F407 path: stream I/O via MDI UART (HW.ptSerial = USART1)
 * ========================================================================= */
#include "mdi_hw.h"
#include "mdi/mdi.h"
#include "port_mdi.h"
#include "protocol.h"
#include <string.h>

static void uart_write_n(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0) {
        MDI_Write(HW.ptSerial, data, (uint32_t)length);
    }
}

static bool uart_write_char(const uint8_t c)
{
    return MDI_Write(HW.ptSerial, &c, 1) == 1;
}

static void uart_write_string(const char *text)
{
    uart_write_n((const uint8_t *)text, (uint16_t)strlen(text));
}

static bool uart_is_connected(void)
{
    return true;
}

static int32_t uart_read(void)
{
    uint8_t ch;
    if (HW.ptSerial != NULL && HW.ptSerial->pPriv != NULL) {
        at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)HW.ptSerial->pPriv;
        while (mringbuf_Read(&ptPriv->tRxQueue, &ch) > 0) {
            if (protocol_enqueue_realtime_command(ch)) {
                continue;
            }
            return (int32_t)ch;
        }
    }
    return -1;
}

static void uart_reset_read_buffer(void)
{
    uint8_t ch;
    if (HW.ptSerial != NULL && HW.ptSerial->pPriv != NULL) {
        at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)HW.ptSerial->pPriv;
        while (mringbuf_Read(&ptPriv->tRxQueue, &ch) > 0) {}
    }
}

static bool uart_suspend_read(bool suspend)
{
    (void)suspend;
    return true;
}

static bool uart_enqueue_rt_command(uint8_t c)
{
    (void)c;
    return false;
}

static uint16_t uart_get_rx_buffer_available(void)
{
    return 0;
}

/* grblHAL 必须：无 NULL 检查直接调用 (report.c:1294) */
static uint16_t uart_get_rx_buffer_free(void)
{
    if (HW.ptSerial != NULL && HW.ptSerial->pPriv != NULL) {
        at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)HW.ptSerial->pPriv;
        return (uint16_t)mringbuf_GetFree(&ptPriv->tRxQueue);
    }
    return 256u;
}

/* 清除输入缓冲区并插入 ASCII_CAN (0x18) 字符 */
static void uart_cancel_read_buffer(void)
{
    uart_reset_read_buffer();
    /* 可选：向协议层插入 CAN (软复位信号)
       当前阶段不需要，留作展开点 */
}

/* 设置实时命令字符处理器（核心会就为我们调用） */
static enqueue_realtime_command_ptr uart_set_enqueue_rt_handler(enqueue_realtime_command_ptr handler)
{
    (void)handler;
    return handler;
}

static io_stream_t s_grblhal_uart = {
    .type                  = StreamType_Serial,
    .is_connected          = uart_is_connected,
    .read                  = uart_read,
    .reset_read_buffer     = uart_reset_read_buffer,
    .cancel_read_buffer    = uart_cancel_read_buffer,
    .set_enqueue_rt_handler = uart_set_enqueue_rt_handler,
    .suspend_read          = uart_suspend_read,
    .enqueue_rt_command    = uart_enqueue_rt_command,
    .get_rx_buffer_free    = uart_get_rx_buffer_free,
    .get_rx_buffer_count   = uart_get_rx_buffer_available,
    .write                 = uart_write_string,
    .write_all             = uart_write_string,
    .write_n               = uart_write_n,
    .write_char            = uart_write_char,
};

void grblhal_stream_init(void)
{
    hal.stream = s_grblhal_uart;
}

#else /* !GRBLHAL_STREAM_UART — original STM32G431 RTT path (unchanged) */

/* =========================================================================
 *  STM32G431 path: stream I/O via SEGGER RTT channel 0
 * ========================================================================= */
#include "SEGGER_RTT.h"

static void rtt_write_n(const uint8_t *data, uint16_t length);
static void rtt_write_string(const char *text);
static bool rtt_write_char(const uint8_t c);
static bool rtt_is_connected(void);
static int32_t rtt_read(void);
static void rtt_reset_read_buffer(void);
static bool rtt_suspend_read(bool suspend);
static bool rtt_enqueue_rt_command(uint8_t c);
static uint16_t rtt_get_rx_buffer_available(void);

static io_stream_t s_grblhal_rtt = {
    .type                  = StreamType_Serial,
    .is_connected          = rtt_is_connected,
    .read                  = rtt_read,
    .reset_read_buffer     = rtt_reset_read_buffer,
    .suspend_read          = rtt_suspend_read,
    .enqueue_rt_command    = rtt_enqueue_rt_command,
    .get_rx_buffer_count   = rtt_get_rx_buffer_available,
    .write                 = rtt_write_string,
    .write_all             = rtt_write_string,
    .write_n               = rtt_write_n,
    .write_char            = rtt_write_char,
};

static void rtt_write_n(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0) {
        SEGGER_RTT_Write(0, data, (unsigned)length);
    }
}

static void rtt_write_string(const char *text)
{
    rtt_write_n((const uint8_t *)text, (uint16_t)strlen(text));
}

static bool rtt_write_char(const uint8_t c)
{
    SEGGER_RTT_PutChar(0, (char)c);
    return true;
}

static bool rtt_is_connected(void) { return true; }

static int32_t rtt_read(void)
{
    uint8_t ch;
    if (SEGGER_RTT_Read(0, &ch, 1) == 1) {
        return (int32_t)ch;
    }
    return -1;
}

static void rtt_reset_read_buffer(void)    { /* no-op for RTT */ }
static bool rtt_suspend_read(bool s)       { (void)s; return true; }
static bool rtt_enqueue_rt_command(uint8_t c) { (void)c; return false; }
static uint16_t rtt_get_rx_buffer_available(void)
{
    uint8_t ch;
    return (SEGGER_RTT_Read(0, &ch, 1) == 1) ? 1 : 0;
}

void grblhal_stream_init(void)
{
    hal.stream = s_grblhal_rtt;
}

#endif /* GRBLHAL_STREAM_UART */
