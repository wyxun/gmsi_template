/**
 * @file   grblhal_config.h
 * @brief  grblHAL compile-time configuration overrides for modus_template
 */

#ifndef __GRBLHAL_CONFIG_H__
#define __GRBLHAL_CONFIG_H__

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

#endif /* __GRBLHAL_CONFIG_H__ */
