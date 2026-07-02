/*============================ INCLUDES ======================================*/
#include "global_define.h"
#include <stdio.h>
#include "peripheral.h"
#include "mdi_hw.h"
#include "modus.h"
#include "mstorage.h"
#include "debug_transport.h"
#include "perf_counter.h"
#include "util_debug.h"

#if FOC_SUPPORT
#include "foc.h"
#endif

#define DEBUG_MINIMAL   0

#if !DEBUG_MINIMAL
static uint8_t s_chSysDataBuf[128];
static mstorage_data_t s_tSysData = {
    .ptFlash              = NULL,
    .wFlashAddr           = 0,
    .pchStorageStartAddr  = s_chSysDataBuf,
    .hwStorageLength      = sizeof(s_chSysDataBuf),
};
static modus_t s_tModus = { .ptAppFlash = NULL };
#endif

#if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
#include <string.h>
void user_trace_output(const char *str)
{
    debug_transport_write_string(str);

    /* 同时输出到 UART0 */
    if (str && HW.ptSerial) {
        MDI_Write(HW.ptSerial, (const uint8_t *)str, strlen(str));
    }
}
#endif

/*============================ MSHELL 串口后端 ==================================*/
#if defined(USERCONFIG_MSHELL_ON_SERIAL) && USERCONFIG_MSHELL_ON_SERIAL && MSHELL_ENABLE
#include "mshell.h"
static unsigned serial_shell_read(char *pchBuf, unsigned hwSize)
{
    return MDI_Read(HW.ptSerial, (uint8_t *)pchBuf, hwSize);
}

static void serial_shell_write(const char *pchBuf, unsigned hwSize)
{
    MDI_Write(HW.ptSerial, (const uint8_t *)pchBuf, hwSize);
}

static const mshell_io_t s_tSerialShellIO = {
    .pfcnRead  = serial_shell_read,
    .pfcnWrite = serial_shell_write,
};
#endif

int main(void)
{
    peripheral_Init();
    perfc_init(true);

#if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
    debug_transport_init();
#if defined(USERCONFIG_MSHELL_ON_SERIAL) && USERCONFIG_MSHELL_ON_SERIAL
    mshell_SetIO(&s_tSerialShellIO);
#endif
    MLOG(I, "\r\n=== MODUS Template BOOT OK ===\r\n");
#endif

#if !DEBUG_MINIMAL
    modus_Init(&s_tModus);
#endif

    uint32_t wCounter = 0;
    while (1) {
#if !DEBUG_MINIMAL
        modus_Run();
#endif

        if (perfc_is_time_out_ms(1000)) {
            wCounter++;
            MLOG(T, "time out\r\n");
        }
    }
    return 0;
}
