/**
 * @file   grblhal_stream.c
 * @brief  grblHAL stream I/O — RTT (STM32G431) or MDI UART (AT32F407)
 */

#include "grblhal_driver.h"

#if defined(GRBLHAL_STREAM_USB)

/* =========================================================================
 *  AT32F407 path: stream I/O via USB CDC
 * ========================================================================= */
#ifdef UNUSED
#undef UNUSED
#endif
#include "usb_conf.h"
#include "usbd_core.h"
#include "protocol.h"
#include <string.h>

#define USB_RX_BUF_SIZE 256
static uint8_t s_usb_rx_buffer[USB_RX_BUF_SIZE];
static volatile uint16_t s_usb_rx_head = 0;
static volatile uint16_t s_usb_rx_tail = 0;
static volatile uint32_t s_usb_rx_overflow = 0;
static enqueue_realtime_command_ptr s_usb_enqueue_rt =
    protocol_enqueue_realtime_command;
static bool usb_is_connected(void);

static void usb_rx_enqueue(uint8_t c)
{
    uint16_t next = (s_usb_rx_head + 1) % USB_RX_BUF_SIZE;
    if (next != s_usb_rx_tail) {
        s_usb_rx_buffer[s_usb_rx_head] = c;
        s_usb_rx_head = next;
    }
}

static int32_t usb_rx_dequeue(void)
{
    if (s_usb_rx_head == s_usb_rx_tail) {
        return -1;
    }
    uint8_t c = s_usb_rx_buffer[s_usb_rx_tail];
    s_usb_rx_tail = (s_usb_rx_tail + 1) % USB_RX_BUF_SIZE;
    return (int32_t)c;
}

static uint16_t usb_rx_available(void)
{
    if (s_usb_rx_head >= s_usb_rx_tail) {
        return s_usb_rx_head - s_usb_rx_tail;
    }
    return USB_RX_BUF_SIZE - (s_usb_rx_tail - s_usb_rx_head);
}

static uint16_t usb_rx_free_space(void)
{
    return USB_RX_BUF_SIZE - 1 - usb_rx_available();
}

static void usb_rx_poll(void)
{
    extern uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *recv_data);
    extern void *get_usb_core_dev(void);
    
    uint8_t tmp_buf[64];
    uint16_t len;
    
    len = usb_vcp_get_rxdata(get_usb_core_dev(), tmp_buf);
    for (uint16_t i = 0; i < len; i++) {
        if (s_usb_enqueue_rt(tmp_buf[i])) {
            continue;
        }
        uint16_t free_before = usb_rx_free_space();
        usb_rx_enqueue(tmp_buf[i]);
        if (free_before == 0) {
            s_usb_rx_overflow++;
        }
    }
}

static void usb_write_n(const uint8_t *data, uint16_t length)
{
    extern error_status usb_vcp_send_data(void *udev, uint8_t *send_data, uint16_t len);
    extern void *get_usb_core_dev(void);
    
    uint16_t sent = 0;
    if (!usb_is_connected()) {
        return;
    }
    while (sent < length) {
        uint16_t chunk = length - sent;
        if (chunk > 64) {
            chunk = 64;
        }
        while (usb_vcp_send_data(get_usb_core_dev(),
                                 (uint8_t *)&data[sent], chunk) == ERROR) {
            if (!usb_is_connected() ||
                hal.stream_blocking_callback == NULL ||
                !hal.stream_blocking_callback()) {
                return;
            }
        }
        sent += chunk;
    }
}

static bool usb_write_char(const uint8_t c)
{
    usb_write_n(&c, 1);
    return usb_is_connected();
}

static void usb_write_string(const char *text)
{
    usb_write_n((const uint8_t *)text, (uint16_t)strlen(text));
}

static bool usb_is_connected(void)
{
    extern void *get_usb_core_dev(void);
    return usbd_connect_state_get((usbd_core_type *)get_usb_core_dev()) ==
           USB_CONN_STATE_CONFIGURED;
}

static int32_t usb_read(void)
{
    usb_rx_poll();
    int32_t ch;
    while ((ch = usb_rx_dequeue()) >= 0) {
        return ch;
    }
    return -1;
}

static void usb_reset_read_buffer(void)
{
    extern uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *recv_data);
    extern void *get_usb_core_dev(void);
    uint8_t discard[64];
    s_usb_rx_head = 0;
    s_usb_rx_tail = 0;
    while (usb_vcp_get_rxdata(get_usb_core_dev(), discard) != 0) {}
}

static void usb_cancel_read_buffer(void)
{
    usb_reset_read_buffer();
    (void)s_usb_enqueue_rt(ASCII_CAN);
}

static bool usb_enqueue_rt_command(uint8_t c)
{
    return s_usb_enqueue_rt(c);
}

static uint16_t usb_get_rx_buffer_available(void)
{
    usb_rx_poll();
    return usb_rx_available();
}

static uint16_t usb_get_rx_buffer_free(void)
{
    usb_rx_poll();
    return usb_rx_free_space();
}

static enqueue_realtime_command_ptr usb_set_enqueue_rt_handler(enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = s_usb_enqueue_rt;
    s_usb_enqueue_rt = handler ? handler : protocol_enqueue_realtime_command;
    return prev;
}

static io_stream_t s_grblhal_usb = {
    .type                  = StreamType_Serial,
    .is_connected          = usb_is_connected,
    .read                  = usb_read,
    .reset_read_buffer     = usb_reset_read_buffer,
    .cancel_read_buffer    = usb_cancel_read_buffer,
    .set_enqueue_rt_handler = usb_set_enqueue_rt_handler,
    .enqueue_rt_command    = usb_enqueue_rt_command,
    .get_rx_buffer_free    = usb_get_rx_buffer_free,
    .get_rx_buffer_count   = usb_get_rx_buffer_available,
    .write                 = usb_write_string,
    .write_all             = usb_write_string,
    .write_n               = usb_write_n,
    .write_char            = usb_write_char,
};

void grblhal_stream_init(void)
{
    hal.stream = s_grblhal_usb;
}

#elif defined(GRBLHAL_STREAM_UART)

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
    uint16_t written = 0;
    while (data != NULL && written < length) {
        int32_t count = MDI_Write(HW.ptSerial, &data[written],
                                  (uint32_t)(length - written));
        if (count > 0) {
            written += (uint16_t)count;
        } else {
            if (hal.stream_blocking_callback == NULL ||
                !hal.stream_blocking_callback()) {
                break;
            }
        }
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

static enqueue_realtime_command_ptr s_uart_enqueue_rt = protocol_enqueue_realtime_command;

static bool uart_enqueue_rt_wrapper(uint8_t c)
{
    return s_uart_enqueue_rt(c);
}

static int32_t uart_read(void)
{
    uint8_t ch;
    if (HW.ptSerial != NULL && HW.ptSerial->pPriv != NULL) {
        at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)HW.ptSerial->pPriv;
        extern void at32_usart_rx_dma_poll(void);
        at32_usart_rx_dma_poll();
        while (mringbuf_Read(&ptPriv->tRxQueue, &ch) > 0) {
            if (s_uart_enqueue_rt(ch)) {
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
        extern void at32_usart_rx_dma_poll(void);
        at32_usart_rx_dma_poll();
        while (mringbuf_Read(&ptPriv->tRxQueue, &ch) > 0) {}
    }
}

static bool uart_enqueue_rt_command(uint8_t c)
{
    return s_uart_enqueue_rt(c);
}

static uint16_t uart_get_rx_buffer_available(void)
{
    if (HW.ptSerial != NULL && HW.ptSerial->pPriv != NULL) {
        at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)HW.ptSerial->pPriv;
        at32_usart_rx_dma_poll();
        return (uint16_t)mringbuf_GetUsed(&ptPriv->tRxQueue);
    }
    return 0u;
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
    (void)s_uart_enqueue_rt(ASCII_CAN);
}

/* 设置实时命令字符处理器（核心会就为我们调用） */
static enqueue_realtime_command_ptr uart_set_enqueue_rt_handler(enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = s_uart_enqueue_rt;
    s_uart_enqueue_rt = handler ? handler : protocol_enqueue_realtime_command;
    return prev;
}

static io_stream_t s_grblhal_uart = {
    .type                  = StreamType_Serial,
    .is_connected          = uart_is_connected,
    .read                  = uart_read,
    .reset_read_buffer     = uart_reset_read_buffer,
    .cancel_read_buffer    = uart_cancel_read_buffer,
    .set_enqueue_rt_handler = uart_set_enqueue_rt_handler,
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
    port_mdi_set_enqueue_rt_handler(uart_enqueue_rt_wrapper);
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
