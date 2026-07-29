/**
 * @file   atc_plugin.c
 * @brief  ATC plugin — GPIO, settings, tool change state machine
 */

#include "atc_plugin.h"
#include "grblhal_driver.h"
#include "system.h"
#include "motion_control.h"
#include "planner.h"
#include "gcode.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "protocol.h"
#include "report.h"
#include "settings.h"
#include "mlog.h"
#include "mshell.h"

#ifdef UNUSED
#undef UNUSED
#endif
#include "at32f403a_407.h"

#include <string.h>

/* ---- Module state ---- */
static atc_settings_t s_atc = {
    .n_pockets      = ATC_DEFAULT_POCKETS,
    .pocket_pitch   = ATC_DEFAULT_POCKET_PITCH,
    .rack_origin_x  = ATC_DEFAULT_RACK_ORIGIN_X,
    .rack_origin_y  = ATC_DEFAULT_RACK_ORIGIN_Y,
    .z_clear_height = ATC_DEFAULT_Z_CLEAR,
    .z_pickup_depth = ATC_DEFAULT_Z_PICKUP,
    .spindle_rpm    = ATC_DEFAULT_SPINDLE_RPM,
    .timeout_ms     = ATC_DEFAULT_TIMEOUT_MS,
    .fg_target      = ATC_DEFAULT_FG_TARGET,
    .flags.value    = 0x01, /* Default bit0 = 1 (use_fg enabled) */
};

static tool_data_t *s_pending_tool = NULL;

/* =========================================================================
 *  BLDC Spindle Unclamp / Clamp Control Functions
 * ========================================================================= */

static bool atc_unclamp_spindle(void)
{
    spindle_state_t state = { .on = 1, .ccw = 1 };
    uint32_t start_ticks = grblhal_get_ticks();
    spindle_ptrs_t *spindle = spindle_get(0);

    if (grblhal_spindle_get_alarm()) {
        MLOGF(E, "ATC Error: Spindle in alarm state before unclamp!\r\n");
        return false;
    }

    MLOGF(I, "ATC: Unclamping spindle (CCW @ %.0f RPM)...\r\n", s_atc.spindle_rpm);
    spindle_set_state(spindle, state, s_atc.spindle_rpm);

    if (s_atc.flags.use_fg) {
        grblhal_spindle_reset_fg_count();
        while (1) {
            grblhal_delay_ms(10, NULL);

            if (grblhal_spindle_get_alarm()) {
                MLOGF(E, "ATC Error: Spindle alarm during unclamp!\r\n");
                spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
                return false;
            }

            if (grblhal_spindle_get_fg_count() >= s_atc.fg_target) {
                MLOGF(I, "ATC Success: Unclamp FG pulse target reached (%u pulses).\r\n",
                      (unsigned)grblhal_spindle_get_fg_count());
                break;
            }

            if ((grblhal_get_ticks() - start_ticks) > s_atc.timeout_ms) {
                MLOGF(E, "ATC Error: Unclamp timed out (%u ms, pulses=%u)!\r\n",
                      (unsigned)s_atc.timeout_ms, (unsigned)grblhal_spindle_get_fg_count());
                spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
                return false;
            }
        }
    } else {
        /* Timeout delay fallback mode */
        grblhal_delay_ms(s_atc.timeout_ms, NULL);
        if (grblhal_spindle_get_alarm()) {
            MLOGF(E, "ATC Error: Spindle alarm during unclamp delay!\r\n");
            spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
            return false;
        }
    }

    spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
    return true;
}

static bool atc_clamp_spindle(void)
{
    spindle_state_t state = { .on = 1, .ccw = 0 };
    uint32_t start_ticks = grblhal_get_ticks();
    spindle_ptrs_t *spindle = spindle_get(0);

    if (grblhal_spindle_get_alarm()) {
        MLOGF(E, "ATC Error: Spindle in alarm state before clamp!\r\n");
        return false;
    }

    MLOGF(I, "ATC: Clamping spindle (CW @ %.0f RPM)...\r\n", s_atc.spindle_rpm);
    spindle_set_state(spindle, state, s_atc.spindle_rpm);

    if (s_atc.flags.use_fg) {
        grblhal_spindle_reset_fg_count();
        /* Allow motor to ramp up before checking stall */
        grblhal_delay_ms(200, NULL);

        while (1) {
            grblhal_delay_ms(10, NULL);

            if (grblhal_spindle_get_alarm()) {
                MLOGF(E, "ATC Error: Spindle alarm during clamp!\r\n");
                spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
                return false;
            }

            /* Detect stall/tightening: motor stopped generating pulses for >300ms */
            if (grblhal_spindle_get_fg_idle_time_ms() > 300 && grblhal_spindle_get_fg_count() > 5) {
                MLOGF(I, "ATC Success: Clamp stall tightening detected (%u pulses).\r\n",
                      (unsigned)grblhal_spindle_get_fg_count());
                break;
            }

            if ((grblhal_get_ticks() - start_ticks) > s_atc.timeout_ms) {
                MLOGF(E, "ATC Error: Clamp timed out (%u ms, pulses=%u)!\r\n",
                      (unsigned)s_atc.timeout_ms, (unsigned)grblhal_spindle_get_fg_count());
                spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
                return false;
            }
        }
    } else {
        /* Timeout delay fallback mode */
        grblhal_delay_ms(s_atc.timeout_ms, NULL);
        if (grblhal_spindle_get_alarm()) {
            MLOGF(E, "ATC Error: Spindle alarm during clamp delay!\r\n");
            spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
            return false;
        }
    }

    spindle_set_state(spindle, (spindle_state_t){0}, 0.0f);
    return true;
}

/* =========================================================================
 *  Setting getters / setters
 * ========================================================================= */

static uint32_t atc_get_int(setting_id_t id)
{
    switch (id) {
        case 900: return s_atc.n_pockets;
        case 907: return s_atc.timeout_ms;
        case 908: return s_atc.fg_target;
        case 909: return s_atc.flags.value;
        default:  return 0;
    }
}

static float atc_get_float(setting_id_t id)
{
    switch (id) {
        case 901: return s_atc.pocket_pitch;
        case 902: return s_atc.rack_origin_x;
        case 903: return s_atc.rack_origin_y;
        case 904: return s_atc.z_clear_height;
        case 905: return s_atc.z_pickup_depth;
        case 906: return s_atc.spindle_rpm;
        default:  return 0.0f;
    }
}

static status_code_t atc_set_pockets(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value < 1 || int_value > 16)
        return Status_BadNumberFormat;
    s_atc.n_pockets = (uint8_t)int_value;
    return Status_OK;
}

static status_code_t atc_set_timeout(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value < 100 || int_value > 30000)
        return Status_BadNumberFormat;
    s_atc.timeout_ms = (uint16_t)int_value;
    return Status_OK;
}

static status_code_t atc_set_fg_target(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value < 1 || int_value > 1000)
        return Status_BadNumberFormat;
    s_atc.fg_target = (uint16_t)int_value;
    return Status_OK;
}

static status_code_t atc_set_flags(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value > 0x03)
        return Status_BadNumberFormat;
    s_atc.flags.value = (uint8_t)int_value;
    return Status_OK;
}

static status_code_t atc_set_pocket_pitch(setting_id_t id, float value)
{
    (void)id;
    if (value < 1.0f || value > 500.0f)
        return Status_BadNumberFormat;
    s_atc.pocket_pitch = value;
    return Status_OK;
}

static status_code_t atc_set_rack_origin_x(setting_id_t id, float value)
{
    (void)id;
    s_atc.rack_origin_x = value;
    return Status_OK;
}

static status_code_t atc_set_rack_origin_y(setting_id_t id, float value)
{
    (void)id;
    s_atc.rack_origin_y = value;
    return Status_OK;
}

static status_code_t atc_set_z_clear(setting_id_t id, float value)
{
    (void)id;
    s_atc.z_clear_height = value;
    return Status_OK;
}

static status_code_t atc_set_z_pickup(setting_id_t id, float value)
{
    (void)id;
    s_atc.z_pickup_depth = value;
    return Status_OK;
}

static status_code_t atc_set_spindle_rpm(setting_id_t id, float value)
{
    (void)id;
    if (value < 100.0f || value > 10000.0f)
        return Status_BadNumberFormat;
    s_atc.spindle_rpm = value;
    return Status_OK;
}

/* =========================================================================
 *  Pocket position & motion helpers
 * ========================================================================= */

static void atc_get_pocket_pos(uint8_t pocket, float *x, float *y)
{
    *x = s_atc.rack_origin_x + (float)(pocket - 1) * s_atc.pocket_pitch;
    *y = s_atc.rack_origin_y;
}

static bool atc_move_z(float z_target, bool rapid)
{
    plan_line_data_t plan_data;
    float target[N_AXIS];

    system_convert_array_steps_to_mpos(target, sys.position);
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = rapid ? On : Off;

    target[Z_AXIS] = z_target;

    return mc_line(target, &plan_data) && protocol_buffer_synchronize();
}

static bool atc_move_xy(float x, float y, float z_keep)
{
    plan_line_data_t plan_data;
    float target[N_AXIS];

    system_convert_array_steps_to_mpos(target, sys.position);
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    target[X_AXIS] = x;
    target[Y_AXIS] = y;
    target[Z_AXIS] = z_keep;

    return mc_line(target, &plan_data) && protocol_buffer_synchronize();
}

/* =========================================================================
 *  Tool change sequence (called as hal.tool.change on M6)
 * ========================================================================= */

static status_code_t atc_tool_change(parser_state_t *parser_state)
{
    if (s_pending_tool == NULL)
        return Status_GCodeToolError;

    tool_id_t old_id = parser_state->tool->tool_id;
    tool_id_t new_id = s_pending_tool->tool_id;

    if (old_id == new_id)
        return Status_OK;

    /* Validate pocket range: skip check for T0 (no tool) */
    if ((old_id > 0 && old_id > (tool_id_t)s_atc.n_pockets) ||
        (new_id > 0 && new_id > (tool_id_t)s_atc.n_pockets))
        return Status_GCodeToolError;

    plan_line_data_t plan_data;
    float target[N_AXIS];
    float saved_xy[2];
    bool ok;

    plan_data_init(&plan_data);

    /* 1. Stop spindle and coolant */
    spindle_all_off(false);
    hal.coolant.set_state((coolant_state_t){0});

    /* 2. Save current XY position (in machine coords, strip tool offset) */
    system_convert_array_steps_to_mpos(target, sys.position);
    target[Z_AXIS] -= gc_get_offset(Z_AXIS, false);
    saved_xy[0] = target[X_AXIS];
    saved_xy[1] = target[Y_AXIS];

    /* 3. Return old tool (if spindle has one) */
    if (old_id > 0) {
        float px, py;
        atc_get_pocket_pos((uint8_t)old_id, &px, &py);

        ok = atc_move_z(s_atc.z_clear_height, true);
        if (!ok) goto fail;

        ok = atc_move_xy(px, py, s_atc.z_clear_height);
        if (!ok) goto fail;

        ok = atc_move_z(s_atc.z_pickup_depth, true);
        if (!ok) goto fail;

        /* Socket mechanical unclamp via reverse spindle rotation */
        if (!atc_unclamp_spindle()) goto fail;

        ok = atc_move_z(s_atc.z_clear_height, true);
        if (!ok) goto fail;
    }

    /* 4. Pick up new tool (if not T0) */
    if (new_id > 0) {
        float px, py;
        atc_get_pocket_pos((uint8_t)new_id, &px, &py);

        ok = atc_move_xy(px, py, s_atc.z_clear_height);
        if (!ok) goto fail;

        ok = atc_move_z(s_atc.z_pickup_depth, true);
        if (!ok) goto fail;

        /* Socket mechanical clamp via forward spindle rotation */
        if (!atc_clamp_spindle()) goto fail;

        ok = atc_move_z(s_atc.z_clear_height, true);
        if (!ok) goto fail;
    }

    /* 5. Optional: restore XY */
    if (s_atc.flags.restore_xy) {
        plan_data_init(&plan_data);
        plan_data.condition.rapid_motion = On;

        system_convert_array_steps_to_mpos(target, sys.position);
        target[X_AXIS] = saved_xy[0];
        target[Y_AXIS] = saved_xy[1];

        ok = mc_line(target, &plan_data) && protocol_buffer_synchronize();
        if (!ok) goto fail;
    }

    sync_position();

    s_pending_tool = NULL;
    return Status_OK;

fail:
    s_pending_tool = NULL;
    return Status_Reset;
}

/* =========================================================================
 *  HAL callbacks
 * ========================================================================= */

static atc_status_t atc_get_state_cb(void)
{
    return ATC_Online;
}

static void atc_tool_select_cb(tool_data_t *tool, bool next)
{
    if (next) {
        s_pending_tool = tool;
    }
}

/* =========================================================================
 *  Setting registration table
 * ========================================================================= */

static const setting_detail_t atc_settings[] = {
    { 900, Group_Toolchange, "ATC pockets",       NULL, Format_Int16,    "###0", "1",    "16",    Setting_NonCoreFn,
      (void *)atc_set_pockets,      (void *)atc_get_int,      NULL, {0} },
    { 901, Group_Toolchange, "ATC pocket pitch",   "mm", Format_Decimal,  "###0.0", "1",  "500",   Setting_NonCoreFn,
      (void *)atc_set_pocket_pitch, (void *)atc_get_float,    NULL, {0} },
    { 902, Group_Toolchange, "ATC rack origin X", "mm", Format_Decimal,  "###0.0", "-9999", "9999", Setting_NonCoreFn,
      (void *)atc_set_rack_origin_x,(void *)atc_get_float,    NULL, {0} },
    { 903, Group_Toolchange, "ATC rack origin Y", "mm", Format_Decimal,  "###0.0", "-9999", "9999", Setting_NonCoreFn,
      (void *)atc_set_rack_origin_y,(void *)atc_get_float,    NULL, {0} },
    { 904, Group_Toolchange, "ATC Z clear",        "mm", Format_Decimal,  "###0.0", "-9999", "9999", Setting_NonCoreFn,
      (void *)atc_set_z_clear,      (void *)atc_get_float,    NULL, {0} },
    { 905, Group_Toolchange, "ATC Z pickup",       "mm", Format_Decimal,  "###0.0", "-9999", "9999", Setting_NonCoreFn,
      (void *)atc_set_z_pickup,     (void *)atc_get_float,    NULL, {0} },
    { 906, Group_Toolchange, "ATC spindle RPM",   "RPM", Format_Decimal,  "###0.0", "100", "10000", Setting_NonCoreFn,
      (void *)atc_set_spindle_rpm,  (void *)atc_get_float,    NULL, {0} },
    { 907, Group_Toolchange, "ATC timeout",        "ms", Format_Int16,    "###0", "100", "30000", Setting_NonCoreFn,
      (void *)atc_set_timeout,      (void *)atc_get_int,      NULL, {0} },
    { 908, Group_Toolchange, "ATC FG target",  "pulses", Format_Int16,    "###0", "1",    "1000",  Setting_NonCoreFn,
      (void *)atc_set_fg_target,    (void *)atc_get_int,      NULL, {0} },
    { 909, Group_Toolchange, "ATC options",        NULL, Format_Bitfield, "Use FG closed loop,Restore XY","", "", Setting_NonCoreFn,
      (void *)atc_set_flags,        (void *)atc_get_int,      NULL, {0} },
};

static setting_details_t atc_setting_details = {
    .is_core       = false,
    .n_groups      = 0,
    .groups        = NULL,
    .n_settings    = sizeof(atc_settings) / sizeof(setting_detail_t),
    .settings      = atc_settings,
    .n_descriptions = 0,
    .descriptions   = NULL,
    .next          = NULL,
};

/* =========================================================================
 *  atc_init — plugin entry, called from my_plugin_init()
 * ========================================================================= */

void atc_init(void)
{
    settings_register(&atc_setting_details);

    hal.tool.select        = atc_tool_select_cb;
    hal.tool.change        = atc_tool_change;
    hal.tool.atc_get_state = atc_get_state_cb;

    hal.driver_cap.atc = 1;

    MLOG_PRINTF("ATC: BLDC Spindle Mechanical Socket ATC plugin initialized");
    (void)s_atc.n_pockets;
}

/* =========================================================================
 *  Shell commands
 * ========================================================================= */

static void atc_shell_unclamp(const char *args)
{
    (void)args;
    MLOG_PRINTF("Testing ATC Unclamp...\r\n");
    if (atc_unclamp_spindle()) {
        MLOG_PRINTF("ATC Unclamp: Success\r\n");
    } else {
        MLOG_PRINTF("ATC Unclamp: Failed\r\n");
    }
}
MODUS_SHELL_CMD(atc_unclamp, atc_shell_unclamp, "Test BLDC spindle unclamp (CCW)");

static void atc_shell_clamp(const char *args)
{
    (void)args;
    MLOG_PRINTF("Testing ATC Clamp...\r\n");
    if (atc_clamp_spindle()) {
        MLOG_PRINTF("ATC Clamp: Success\r\n");
    } else {
        MLOG_PRINTF("ATC Clamp: Failed\r\n");
    }
}
MODUS_SHELL_CMD(atc_clamp, atc_shell_clamp, "Test BLDC spindle clamp (CW)");


