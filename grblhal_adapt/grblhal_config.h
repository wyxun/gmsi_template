/**
 * @file   grblhal_config.h
 * @brief  grblHAL compile-time configuration overrides for modus_template
 *
 * Two paths:
 *   GRBLHAL_FULL_FEATURES (AT32F407) — full feature set, no trimming
 *   default (STM32G431) — trimmed for 128KB flash
 */

#ifndef __GRBLHAL_CONFIG_H__
#define __GRBLHAL_CONFIG_H__

#ifdef GRBLHAL_FULL_FEATURES

/* =========================================================================
 *  AT32F407: full features — 1024KB flash, no trimming needed
 * ========================================================================= */
#define COMPATIBILITY_LEVEL             10

/* ---- Machine geometry ---- */
#define N_AXIS          3       /* X, Y, Z — 3-axis CNC */
#define HOMING_CYCLE_0  (1 << 0)
#define HOMING_CYCLE_1  (1 << 1)
#define HOMING_CYCLE_2  (1 << 2)
/* ---- Axis travel limits ---- */
#define DEFAULT_X_MAX_TRAVEL    450.0f  /* $130 */
#define DEFAULT_Y_MAX_TRAVEL    400.0f  /* $131 */
#define DEFAULT_Z_MAX_TRAVEL    100.0f  /* $132 */

/* ---- Steps per mm ---- */
#define DEFAULT_X_STEPS_PER_MM  80.0f   /* $100 */
#define DEFAULT_Y_STEPS_PER_MM  800.0f  /* $101 */
#define DEFAULT_Z_STEPS_PER_MM  800.0f  /* $102 */

/* ---- Axis speeds ---- */
#define DEFAULT_X_MAX_RATE      500.0f
#define DEFAULT_Y_MAX_RATE      500.0f
#define DEFAULT_Z_MAX_RATE      500.0f

/* ---- Axis acceleration ---- */
#define DEFAULT_X_ACCELERATION  30.0f   /* $120 */
#define DEFAULT_Y_ACCELERATION  30.0f   /* $121 */
#define DEFAULT_Z_ACCELERATION  30.0f   /* $122 */

/* ---- Stepper signals ---- */
#define DEFAULT_STEP_PULSE_MICROSECONDS 10.0f  /* $0   — min pulse width (must also set hal.step_us_min in grblhal_stubs.c) */
#define DEFAULT_STEPPER_IDLE_LOCK_TIME  25      /* $1   — idle lock time (ms) */
#define DEFAULT_STEP_SIGNALS_INVERT_MASK  0     /* $2   — step polarity, 0 = positive pulse */
#define DEFAULT_DIR_SIGNALS_INVERT_MASK   1     /* $3   — dir invert, 1 = invert X-axis only */
#define DEFAULT_ENABLE_SIGNALS_INVERT_MASK 0    /* $4   — enable invert, 0 = no invert (active low) */
#define DEFAULT_STEP_PULSE_DELAY          0.0f  /* $29  — DIR setup time (µs) */
#define DEFAULT_STEPPER_DEENERGIZE_MASK   0     /* $37  — axes to keep energized */
#define DEFAULT_STEPPER_ENABLE_DELAY      0     /* $680 — enable delay (ms) */

/* ---- Buffer sizes ---- */
#define RX_BUFFER_SIZE      256
#define TX_BUFFER_SIZE      256
#define BLOCK_BUFFER_SIZE   36
#define NVS_BUFFER_SIZE     512

/* ---- Feature toggles (all enabled — stubs for now) ---- */
#define SPINDLE_ENABLE      1
#define COOLANT_ENABLE      1
#define PROBE_ENABLE        1
#define LIMITS_ENABLE       1
#define CONTROL_ENABLE      1
#define ENABLE_SAFETY_DOOR_INPUT_PIN  1
#define SDCARD_ENABLE       0
#define EEPROM_ENABLE       0
#define BLUETOOTH_ENABLE    0
#define ETHERNET_ENABLE     0
#define WIFI_ENABLE         0
#define ENABLE_SOFTWARE_DEBOUNCE  0

/* ---- Spindle defaults ---- */
#define DEFAULT_SPINDLE_ON_DELAY     8000    /* $394 — ms, slow accel for DC motor */
#define DEFAULT_SPINDLE_OFF_DELAY    5000    /* $539 — ms, coast-down time */

/* ---- Full feature set (no trimming) ---- */
#define NGC_PARAMETERS_ENABLE           1
#define NVSDATA_BUFFER_ENABLE           1
#define ENABLE_RESTORE_NVS_WIPE_ALL     1
#define ENABLE_RESTORE_NVS_DEFAULT_SETTINGS 1
#define ENABLE_RESTORE_NVS_CLEAR_PARAMETERS 1
#define ENABLE_RESTORE_NVS_DRIVER_PARAMETERS 1
#define SETTINGS_RESTORE_DEFAULTS       1
#define SETTINGS_RESTORE_PARAMETERS     1
#define SETTINGS_RESTORE_STARTUP_LINES  1
#define SETTINGS_RESTORE_BUILD_INFO     1
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 1

#ifdef __SETTINGS_C__
#include <stdint.h>
#include "errors.h"
#include "settings.h"
#endif

#else /* !GRBLHAL_FULL_FEATURES — original STM32G431 trimmed config (unchanged) */

/* =========================================================================
 *  STM32G431: trimmed for 128KB flash
 * ========================================================================= */
#define COMPATIBILITY_LEVEL             10

/* ---- Machine geometry ---- */
#define N_AXIS          3       /* X, Y, Z — 3-axis CNC */
#define HOMING_CYCLE_0  (1 << 0)
#define HOMING_CYCLE_1  (1 << 1)
#define HOMING_CYCLE_2  (1 << 2)

/* ---- Buffer sizes ---- */
#define RX_BUFFER_SIZE      256
#define TX_BUFFER_SIZE      256
#define BLOCK_BUFFER_SIZE   36
#define NVS_BUFFER_SIZE     512

/* ---- Feature toggles (stub driver — minimal capabilities) ---- */
#define SPINDLE_ENABLE      0
#define COOLANT_ENABLE      0
#define PROBE_ENABLE        0
#define LIMITS_ENABLE       0
#define CONTROL_ENABLE      0
#define ENABLE_SAFETY_DOOR_INPUT_PIN  0
#define SDCARD_ENABLE       0
#define EEPROM_ENABLE       0
#define BLUETOOTH_ENABLE    0
#define ETHERNET_ENABLE     0
#define WIFI_ENABLE         0
#define ENABLE_SOFTWARE_DEBOUNCE  0

/* ---- Footprint optimization (disable unused heavy features) ---- */
#define NGC_PARAMETERS_ENABLE           0
#define NVSDATA_BUFFER_ENABLE           0
#define ENABLE_RESTORE_NVS_WIPE_ALL     0
#define ENABLE_RESTORE_NVS_DEFAULT_SETTINGS 0
#define ENABLE_RESTORE_NVS_CLEAR_PARAMETERS 0
#define ENABLE_RESTORE_NVS_DRIVER_PARAMETERS 0
#define SETTINGS_RESTORE_DEFAULTS       0
#define SETTINGS_RESTORE_PARAMETERS     0
#define SETTINGS_RESTORE_STARTUP_LINES  0
#define SETTINGS_RESTORE_BUILD_INFO     0
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 0

#define NO_SAFETY_DOOR_SUPPORT          1
#define NO_SETTINGS_DESCRIPTIONS        1
#define NO_ERROR_DESCRIPTIONS           1
#define NO_TOOL_CHANGE_SUPPORT          1

#ifdef __SETTINGS_C__
#ifdef NO_SAFETY_DOOR_SUPPORT
#include <stdint.h>
#include "errors.h"
#include "settings.h"
static inline status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value) { return Status_OK; }
static inline status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value) { return Status_OK; }
#endif
#endif

#endif /* GRBLHAL_FULL_FEATURES */

#endif /* __GRBLHAL_CONFIG_H__ */
