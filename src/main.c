/*============================ INCLUDES ======================================*/
#include "global_define.h"
#include <stdio.h>
#include "peripheral.h"
#include "modus.h"
#include "mstorage.h"
#include "SEGGER_RTT.h"
#include "perf_counter.h"
#include "util_debug.h"
//#include "CH592SFR.h"

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
#endif

int main(void)
{
    /* 必须在 peripheral_Init (启动 SysTick) 之前调用 */
    perfc_init(true);
    peripheral_Init();

#if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
    SEGGER_RTT_Init();
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
        uint8_t chBuf[64];
        int32_t nReadBytes = MDI_Read(HW.ptSerial, chBuf, sizeof(chBuf));
        if (nReadBytes > 0) {
            MDI_Write(HW.ptSerial, chBuf, nReadBytes);
        }

        if (perfc_is_time_out_ms(1000)) {
            wCounter++;
            /* 直接写 UART (绕过 MDI/RTT, 纯诊断) */
            // while (!(R8_UART0_LSR & RB_LSR_TX_FIFO_EMP));
            // R8_UART0_THR = '0' + (uint8_t)(wCounter % 10);
            // while (!(R8_UART0_LSR & RB_LSR_TX_FIFO_EMP));
            // R8_UART0_THR = '\r';
            // while (!(R8_UART0_LSR & RB_LSR_TX_FIFO_EMP));
            // R8_UART0_THR = '\n';
        }
    }
    return 0;
}
