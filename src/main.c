/*============================ INCLUDES ======================================*/
#include <stdio.h>
#include "peripheral.h"
#include "gdi_hw.h"
#include "gmsi.h"
#include "gstorage.h"
#include "SEGGER_RTT.h"
#include "perf_counter.h"
#include "util_debug.h"

#if FOC_SUPPORT
#include "foc.h"
#endif

/*============================ MACROS ========================================*/
#define DEBUG_MINIMAL   0

/*============================ PROTOTYPES ====================================*/

/*============================ GLOBAL VARIABLES ==============================*/

volatile uint8_t s_bInitDone = 0;

/*============================ LOCAL VARIABLES ===============================*/

#if !DEBUG_MINIMAL
/* RAM buffer for storage (optional, no flash device for now) */
static uint8_t s_chSysDataBuf[32];
static gstorage_data_t s_tSysData = {
    .ptFlash              = NULL,               /* no flash backend yet */
    .wFlashAddr           = 0,
    .pchStorageStartAddr  = s_chSysDataBuf,
    .hwStorageLength      = sizeof(s_chSysDataBuf),
};
static gmsi_t s_tGmsi = { .ptAppFlash = NULL };

#endif

/*============================ IMPLEMENTATION ================================*/

void user_trace_output(const char *str)
{
    SEGGER_RTT_WriteString(0, str);
}

/*============================ MAIN ==========================================*/

int main(void)
{
    /* 1. Low-level HAL, Clock & all peripherals init */
    peripheral_Init();

    /* 2. perf_counter init — must be after Clock setup */
    perfc_init(true);

    /* 3. RTT init — print BEFORE complex peripheral init */
    SEGGER_RTT_Init();
    GLOG(I, "\r\n=== GMSI Template BOOT OK ===\r\n");


#if !DEBUG_MINIMAL
    /* 4. GMSI framework init (also auto-inits FOC app via linker section) */
    gmsi_Init(&s_tGmsi);
#endif

    /* 5. Allow SysTick_Handler to call gmsi_Clock */
    s_bInitDone = 1;

    /* 6. Main loop */
    uint32_t wCounter = 0;

    while (1) {
#if !DEBUG_MINIMAL
        gmsi_Run();
#endif
        if (perfc_is_time_out_ms(1000)) {
            GDI_Toggle(HW.ptLedStatus);
            wCounter++;
            GLOGF(T, "[TICK] %lu s  SYSCLK=%lu Hz\r\n",
                  (unsigned long)wCounter,
                  (unsigned long)get_system_core_clock_hz());
        }
    }
    return 0;
}
