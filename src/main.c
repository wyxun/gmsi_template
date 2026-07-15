/*============================ INCLUDES ======================================*/
#include "global_define.h"
#include <stdio.h>
#include "peripheral.h"
#include "mdi_hw.h"
#include "perf_counter.h"
#include "util_debug.h"
#include "debug_transport.h"

#if MODUS_ENABLE
#include "modus.h"
static modus_t s_tModus = { .ptAppFlash = NULL };
#endif

#if MODUS_ENABLE && (MSHELL_ENABLE || !defined(__NO_USE_LOG__))
#include <string.h>
void user_trace_output(const char *str)
{
    debug_transport_write_string(str);

    /* Output to Serial too */
    if (str && HW.ptSerial) {
        MDI_Write(HW.ptSerial, (const uint8_t *)str, strlen(str));
    }
}
#endif

int main(void)
{
    peripheral_Init();
    perfc_init(true);

#if MODUS_ENABLE
    #if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
        debug_transport_init();
    #endif
    modus_Init(&s_tModus);
#endif

    /* Direct grblHAL handover (contains its own infinite blocking loop) */
    extern int grbl_enter(void);
    grbl_enter();

    return 0;
}
