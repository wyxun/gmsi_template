/**
 * @file   grblhal.h
 * @brief  grblHAL CNC controller MODUS Object — public API
 */

#ifndef __GRBLHAL_H__
#define __GRBLHAL_H__

#include "modus.h"

/* Object ID */
#define GRBLHAL_ID  0x10

/* Realtime command IDs (subset of grblHAL's CMD_* defines) */
enum {
    GRBLHAL_RT_RESET     = 0x18,  /* Ctrl-X — soft reset */
    GRBLHAL_RT_FEED_HOLD = '!',   /* pause motion */
    GRBLHAL_RT_CYCLE_START = '~', /* resume */
    GRBLHAL_RT_STATUS    = '?',   /* immediate status report */
    GRBLHAL_RT_ESTOP     = 0x1A,  /* Ctrl-Z — emergency stop (safety door) */
};

typedef struct {
    uint8_t  *pchRingBuffer;
    uint16_t  hwRingSize;
} grblhal_cfg_t;

typedef struct {
    modus_base_t *ptBase;
    uint8_t       chState;
} grblhal_t;

/* MODUS auto-registration entry */
int grblhal_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/* ---- Public API ----
 *
 * External modules (mShell commands, other MODUS Objects, etc.) call these
 * to feed G-code or inject realtime commands into the running grblHAL core.
 */

/**
 * @brief Feed a null-terminated G-code line into grblHAL.
 * @return true if queued, false if buffer full or parser busy.
 */
bool grblhal_FeedGcode(const char *line);

/**
 * @brief Inject a realtime command character (feed hold, reset, estop, etc.).
 *
 * Command characters defined by grblHAL protocol:
 *   '!' — feed hold   '~' — cycle start   '?' — status report
 *   0x18 (Ctrl-X) — soft reset   0x1A (Ctrl-Z) — safety door / estop
 *
 * @param cmd  single ASCII character.
 * @return true if enqueued, false if the character was not recognized.
 */
bool grblhal_PostRealtime(char cmd);

#endif /* __GRBLHAL_H__ */
