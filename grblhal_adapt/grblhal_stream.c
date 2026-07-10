/**
 * @file   grblhal_stream.c
 * @brief  grblHAL stream I/O via SEGGER RTT channel 0
 */

#include "grblhal_driver.h"
#include "SEGGER_RTT.h"

static void rtt_write_n(const uint8_t *data, uint16_t length);
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
    /* optional fields */
    .write_n               = rtt_write_n,
    .write_char            = rtt_write_char,
};

static void rtt_write_n(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0) {
        SEGGER_RTT_Write(0, data, (unsigned)length);
    }
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
