/*============================ INCLUDES ======================================*/
#include "global_define.h"
#include <stdio.h>
#include "peripheral.h"
#include "modus.h"
#include "mstorage.h"
#include "SEGGER_RTT.h"
#include "perf_counter.h"
#include "util_debug.h"

#if FOC_SUPPORT
#include "foc.h"
#endif

#define DEBUG_MINIMAL   0

volatile uint8_t s_bInitDone = 0;

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
    SEGGER_RTT_WriteString(0, str);

    /* 同时输出到 UART0 (PB7 TX, 115200bps) */
    if (str && HW.ptSerial) {
        MDI_Write(HW.ptSerial, (const uint8_t *)str, strlen(str));
    }
}

void mshell_uart_init(void);
#endif

int main(void)
{
    /* 必须在 peripheral_Init (启动 SysTick) 之前调用 */
    perfc_init(true);
    peripheral_Init();

#if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
    SEGGER_RTT_Init();
    mshell_uart_init();
    MLOG(I, "\r\n=== MODUS Template BOOT OK ===\r\n");
#endif

#if !DEBUG_MINIMAL
    modus_Init(&s_tModus);
#endif

    s_bInitDone = 1;

    uint32_t wCounter = 0;
    while (1) {
#if !DEBUG_MINIMAL
        modus_Run();
#endif

        if (perfc_is_time_out_ms(1000)) {
            wCounter++;
        }
    }
    return 0;
}
