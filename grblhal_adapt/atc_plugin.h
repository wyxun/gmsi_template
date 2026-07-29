/**
 * @file   atc_plugin.h
 * @brief  ATC (Automatic Tool Changer) plugin for rack-style tool changer
 *
 * Hardware: single-row linear rack, 5-8 pockets, pneumatic drawbar
 * I/O:      PB0 = drawbar solenoid (HIGH = release)
 *           PB1 = air blast (HIGH = blast)
 */
#ifndef __ATC_PLUGIN_H__
#define __ATC_PLUGIN_H__

/*
 * Avoid name collision: both modus (mbase.h) and grblHAL (messages.h)
 * define message_t. grblhal_driver.h handles this rename for us.
 */
#define message_t grblhal_message_t

#include "grbl.h"
#include "hal.h"
#include "core_handlers.h"

#undef message_t

/* ---- ATC settings struct ---- */
typedef union {
    uint8_t value;
    struct {
        uint8_t use_fg       :1,  /* bit0: 1=FG closed loop, 0=Timeout delay mode */
                restore_xy   :1,  /* bit1: restore XY position after M6 */
                unassigned   :6;
    };
} atc_flags_t;

typedef struct {
    uint8_t  n_pockets;       /* $900 - number of rack pockets (1-16)        */
    float    pocket_pitch;    /* $901 - X-axis spacing between pockets (mm)  */
    float    rack_origin_x;   /* $902 - X coordinate of pocket #1 (machine)  */
    float    rack_origin_y;   /* $903 - Y coordinate of rack (machine)       */
    float    z_clear_height;  /* $904 - Z safe height above rack (machine)   */
    float    z_pickup_depth;  /* $905 - Z depth for pickup/drop (machine)    */
    float    spindle_rpm;     /* $906 - Spindle RPM during unclamp/clamp     */
    uint16_t timeout_ms;      /* $907 - Tool change timeout / delay (ms)     */
    uint16_t fg_target;       /* $908 - Target FG pulses for unclamp         */
    atc_flags_t flags;        /* $909 - option bits                          */
} atc_settings_t;

/* ---- Default values ---- */
#define ATC_DEFAULT_POCKETS        8
#define ATC_DEFAULT_POCKET_PITCH   50.0f
#define ATC_DEFAULT_RACK_ORIGIN_X  0.0f
#define ATC_DEFAULT_RACK_ORIGIN_Y  0.0f
#define ATC_DEFAULT_Z_CLEAR        0.0f
#define ATC_DEFAULT_Z_PICKUP       0.0f
#define ATC_DEFAULT_SPINDLE_RPM    500.0f
#define ATC_DEFAULT_TIMEOUT_MS     2000
#define ATC_DEFAULT_FG_TARGET      30

/* ---- Plugin entry point ---- */
void atc_init(void);

#endif /* __ATC_PLUGIN_H__ */
