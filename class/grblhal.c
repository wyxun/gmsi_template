/**
 * @file   grblhal.c
 * @brief  grblHAL MODUS Object — wraps grblHAL init + main loop
 *
 * Public API (grblhal.h):
 *   grblhal_FeedGcode()    — feed a G-code line from external modules
 *   grblhal_PostRealtime() — inject realtime command (!, ~, ?, Ctrl-X, etc.)
 *
 * Inter-object communication:
 *   Events — mbase_EventPost(GRBLHAL_ID, event) with grblHAL realtime char
 *            as the event word (lower 8 bits).
 *   Ring buffer — mbase_MessagePostToRing(GRBLHAL_ID, data, len) feeds
 *                 G-code text directly.
 */

#include "grblhal.h"

#if GRBLHAL_ENABLE

#include "grblhal_driver.h"
#include "grbllib.h"
#include "protocol.h"
#include "system.h"
#include "SEGGER_RTT.h"
#include <string.h>

#undef  this
#define this (*ptThis)

static int grblhal_Clock(uintptr_t wObjectAddr);
static int grblhal_Run  (uintptr_t wObjectAddr);

static modus_base_t     s_tGrblhalBase;
static modus_base_cfg_t s_tGrblhalBaseCfg = {
    .wId     = GRBLHAL_ID,
    .wParent = 0,
    .FcnInterface = {
        .Clock = grblhal_Clock,
        .Run   = grblhal_Run,
    },
};

/* ---- 1ms tick counter for hal.get_elapsed_ticks ---- */
static volatile uint32_t s_wGrblhalTicks;

/* ---- Startup banner ---- */
static void grblhal_banner(void)
{
    SEGGER_RTT_WriteString(0,
        "\r\n"
        "========================================\r\n"
        " grblHAL on modus_template (Phase 1)\r\n"
        " Type G-code and press Enter to execute.\r\n"
        " Realtime: ! = feed hold, ~ = resume,\r\n"
        "           ? = status, Ctrl-X = reset\r\n"
        "========================================\r\n"
        "\r\n"
    );
}

/* =========================================================================
 *  Init — called automatically by modus_Init() via .init_infos section
 * ========================================================================= */
int grblhal_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    grblhal_t     *ptThis = (grblhal_t *)wObjectAddr;
    grblhal_cfg_t *ptCfg  = (grblhal_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        return MODUS_EFAIL;
    }

    ptThis->ptBase = &s_tGrblhalBase;
    this.chState   = 0;

    s_tGrblhalBaseCfg.pchRingBuffer = ptCfg->pchRingBuffer;
    s_tGrblhalBaseCfg.hwRingSize    = ptCfg->hwRingSize;

    s_wGrblhalTicks = 0;

    /* grblHAL boot sequence:
     *   grbllib.c:grbl_enter() calls driver_init() (the real symbol from
     *   grblhal_stubs.c). It sets hal.* function pointers and returns true.
     *   Then grbl_enter() continues with settings_load + driver_setup + main loop.
     */
    grbl_enter();

    grblhal_banner();

    return mbase_Init(ptThis->ptBase, &s_tGrblhalBaseCfg);
}

/* =========================================================================
 *  Run — called each main-loop iteration by modus_Run()
 * ========================================================================= */
static int grblhal_Run(uintptr_t wObjectAddr)
{
    grblhal_t *ptThis = (grblhal_t *)wObjectAddr;
    (void)ptThis;

    /*
     * 1. Service incoming MODUS ring-buffer: feed any G-code text
     *    posted from other objects (e.g. a CAN/UART relay Object).
     */
    uint8_t chBuf[128];
    int nLen = mbase_MessagePendFromRing(ptThis->ptBase, chBuf, sizeof(chBuf) - 1);
    if (nLen > 0) {
        chBuf[nLen] = '\0';
        grblhal_FeedGcode((const char *)chBuf);
    }

    /*
     * 2. Service incoming MODUS events: extract realtime command char
     *    from the low 8 bits of the event word.
     */
    uint32_t wEvent = mbase_EventPend(ptThis->ptBase);
    if (wEvent != 0) {
        grblhal_PostRealtime((char)(wEvent & 0xFF));
    }

    /*
     * 3. Run grblHAL main loop.
     */
    protocol_execute_realtime();
    protocol_main_loop();

    return MODUS_SUCCESS;
}

/* =========================================================================
 *  Clock — 1ms tick: drive hal.get_elapsed_ticks
 * ========================================================================= */
static int grblhal_Clock(uintptr_t wObjectAddr)
{
    (void)wObjectAddr;
    s_wGrblhalTicks++;
    return MODUS_SUCCESS;
}

/* =========================================================================
 *  Public API
 * ========================================================================= */

bool grblhal_FeedGcode(const char *line)
{
    if (line == NULL || line[0] == '\0') {
        return false;
    }
    return protocol_enqueue_gcode((char *)line);
}

bool grblhal_PostRealtime(char cmd)
{
    return protocol_enqueue_realtime_command((uint8_t)cmd);
}

/* =========================================================================
 *  Helper for hal.get_elapsed_ticks (exposed to stubs via grblhal_driver.h)
 * ========================================================================= */
uint32_t grblhal_get_ticks(void)
{
    return s_wGrblhalTicks;
}

/* =========================================================================
 *  Auto-registration
 * ========================================================================= */
static uint8_t s_chGrblhalRxBuffer[256];

MODUS_DECLARE_OBJECT(grblhal, GrblHAL,
    .pchRingBuffer = s_chGrblhalRxBuffer,
    .hwRingSize    = sizeof(s_chGrblhalRxBuffer)
)

#else  /* !GRBLHAL_ENABLE — stub registration so linker doesn't complain */

MODUS_DECLARE_OBJECT(grblhal, GrblHAL,
    .pchRingBuffer = NULL,
    .hwRingSize    = 0
)

#endif /* GRBLHAL_ENABLE */
