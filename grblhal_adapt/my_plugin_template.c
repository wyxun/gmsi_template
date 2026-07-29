/**
 * @file   my_plugin_template.c
 * @brief  grblHAL + MODUS 自定义插件标准开发模板 (C Source)
 * 
 * 本模板完整示范了：
 * 1. 结构体存储与默认参数定义
 * 2. grblHAL 系统参数 ($950-$952) 读写与回调挂载
 * 3. 链式 Hook 拦截（实时轮询循环、M 代码扩展、$I 插件宣告）
 * 4. MODUS 终端 Shell 调试命令注册 (MODUS_SHELL_CMD)
 * 5. 模块初始化入口实现
 */

#include "my_plugin_template.h"
#include "grblhal_driver.h"
#include "report.h"
#include "mshell.h"
#include "mlog.h"
#include <string.h>

/* =========================================================================
 * 1. 模块私有变量 (Module State)
 * ========================================================================= */

/* 本插件的实际配置数据实例 */
static my_plugin_settings_t s_cfg = {
    .param_delay_ms  = MY_PLUGIN_DEFAULT_DELAY_MS,
    .param_threshold = MY_PLUGIN_DEFAULT_THRESHOLD,
    .flags.value     = MY_PLUGIN_DEFAULT_FLAGS,
};

/* 用于链式 Hook 调用的前级回调指针保存变量 */
static on_execute_realtime_ptr s_fnPrevExecuteRealtime = NULL;
static on_report_options_ptr   s_fnPrevReportOptions   = NULL;
static user_mcode_ptrs_t       s_prevMCode;

/* =========================================================================
 * 2. grblHAL 系统参数 ($950 ~ $952) 读写回调函数
 * ========================================================================= */

/* 整数型参数读取回调 */
static uint32_t my_plugin_get_int(setting_id_t id)
{
    switch ((uint32_t)id) {
        case 950: return s_cfg.param_delay_ms;
        case 952: return s_cfg.flags.value;
        default:  return 0;
    }
}

/* 浮点型参数读取回调 */
static float my_plugin_get_float(setting_id_t id)
{
    switch ((uint32_t)id) {
        case 951: return s_cfg.param_threshold;
        default:  return 0.0f;
    }
}

/* $950 参数设置回调 */
static status_code_t my_plugin_set_delay(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value < 10 || int_value > 10000) {
        return Status_BadNumberFormat; /* 超出数值范围 */
    }
    s_cfg.param_delay_ms = (uint16_t)int_value;
    return Status_OK;
}

/* $951 参数设置回调 */
static status_code_t my_plugin_set_threshold(setting_id_t id, float value)
{
    (void)id;
    if (value < 0.0f || value > 100.0f) {
        return Status_BadNumberFormat;
    }
    s_cfg.param_threshold = value;
    return Status_OK;
}

/* $952 参数设置回调 */
static status_code_t my_plugin_set_flags(setting_id_t id, uint_fast16_t int_value)
{
    (void)id;
    if (int_value > 0x03) {
        return Status_BadNumberFormat;
    }
    s_cfg.flags.value = (uint8_t)int_value;
    return Status_OK;
}

/* 参数注册列表表单 (ID, 分组, 描述, 单位, 格式, 格式掩码, 最小值, 最大值, 属性, Setter, Getter, ...) */
static const setting_detail_t my_plugin_settings[] = {
    { 950, Group_Root, "MyPlugin delay",     "ms", Format_Int16,    "###0",  "10", "10000", Setting_NonCoreFn,
      (void *)my_plugin_set_delay,     (void *)my_plugin_get_int,   NULL, {0} },
    { 951, Group_Root, "MyPlugin threshold",  "%", Format_Decimal,  "###0.0","0",  "100",   Setting_NonCoreFn,
      (void *)my_plugin_set_threshold, (void *)my_plugin_get_float, NULL, {0} },
    { 952, Group_Root, "MyPlugin options",   NULL, Format_Bitfield, "Enable,AutoMode", "", "", Setting_NonCoreFn,
      (void *)my_plugin_set_flags,     (void *)my_plugin_get_int,   NULL, {0} },
};

static setting_details_t my_plugin_setting_details = {
    .is_core        = false,
    .n_groups       = 0,
    .groups         = NULL,
    .n_settings     = sizeof(my_plugin_settings) / sizeof(setting_detail_t),
    .settings       = my_plugin_settings,
    .n_descriptions = 0,
    .descriptions   = NULL,
    .next           = NULL,
};

/* =========================================================================
 * 3. 链式 Hook 拦截实现 (Hook Chaining)
 * ========================================================================= */

/**
 * Hook 1: 实时循环回调 (由主循环在 kHz 频率调用)
 * 用途：实现轮询逻辑、按键检测、软计时器、简易状态机驱动等
 */
static void my_plugin_OnExecuteRealtime(sys_state_t state)
{
    /* 1. 先调用上一级保存的 Hook (保证链式传递不被断掉) */
    if (s_fnPrevExecuteRealtime) {
        s_fnPrevExecuteRealtime(state);
    }

    /* 2. 执行本插件的非阻塞轮询逻辑 */
    if (s_cfg.flags.enable) {
        /* 例如：轮询判断某个 GPIO 或执行状态 */
    }
}

/**
 * Hook 2: 上报插件信息 ($I 命令响应)
 */
static void my_plugin_OnReportOptions(bool bNewopt)
{
    if (s_fnPrevReportOptions) {
        s_fnPrevReportOptions(bNewopt);
    }
    if (!bNewopt) {
        /* 在 $I 响应列表中增加本插件名称与版本号 */
        report_plugin("My Custom Plugin", "1.00");
    }
}

/**
 * Hook 3: 自定义 M 代码 (例如 M100) 检查、验证与执行
 */
static user_mcode_type_t my_plugin_MCodeCheck(user_mcode_t mcode)
{
    if (mcode == (user_mcode_t)100 || mcode == (user_mcode_t)101) {
        return UserMCode_Normal; /* 返回支持的自定义 M 代码类型 */
    }
    return s_prevMCode.check ? s_prevMCode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t my_plugin_MCodeValidate(parser_block_t *gc_block)
{
    if (gc_block->user_mcode == (user_mcode_t)100 || gc_block->user_mcode == (user_mcode_t)101) {
        return Status_OK;
    }
    return s_prevMCode.validate ? s_prevMCode.validate(gc_block) : Status_GcodeUnsupportedCommand;
}

static void my_plugin_MCodeExecute(sys_state_t state, parser_block_t *gc_block)
{
    if (gc_block->user_mcode == (user_mcode_t)100) {
        MLOGF(I, "MyPlugin: Executed M100 command!\r\n");
        return;
    } else if (gc_block->user_mcode == (user_mcode_t)101) {
        MLOGF(I, "MyPlugin: Executed M101 command!\r\n");
        return;
    }

    if (s_prevMCode.execute) {
        s_prevMCode.execute(state, gc_block);
    }
}

/* =========================================================================
 * 4. MODUS Shell 调试命令注册 (可在 RTT/串口 终端直接敲入测试)
 * ========================================================================= */

static void cmd_my_plugin_status(const char *args)
{
    (void)args;
    MLOGF(I, "=== MyPlugin Status ===\r\n");
    MLOGF(I, "  Enable:    %u\r\n", s_cfg.flags.enable);
    MLOGF(I, "  AutoMode:  %u\r\n", s_cfg.flags.auto_mode);
    MLOGF(I, "  Delay:     %u ms\r\n", (unsigned)s_cfg.param_delay_ms);
    MLOGF(I, "  Threshold: %.2f%%\r\n", (double)s_cfg.param_threshold);
}

/* 使用 MODUS_SHELL_CMD 宏，零侵入注册命令 "my_status" */
MODUS_SHELL_CMD(my_status, cmd_my_plugin_status, "Show custom plugin status");

/* =========================================================================
 * 5. 插件入口函数实现
 * ========================================================================= */

void my_plugin_template_init(void)
{
    /* 1. 注册系统设置参数 $950 - $952 */
    settings_register(&my_plugin_setting_details);

    /* 2. 链式挂载 实时循环 Hook */
    s_fnPrevExecuteRealtime  = grbl.on_execute_realtime;
    grbl.on_execute_realtime = my_plugin_OnExecuteRealtime;

    /* 3. 链式挂载 $I 响应 Hook */
    s_fnPrevReportOptions  = grbl.on_report_options;
    grbl.on_report_options = my_plugin_OnReportOptions;

    /* 4. 链式挂载 自定义 M 代码 Hook (M100/M101) */
    s_prevMCode         = grbl.user_mcode;
    grbl.user_mcode.check    = my_plugin_MCodeCheck;
    grbl.user_mcode.validate = my_plugin_MCodeValidate;
    grbl.user_mcode.execute  = my_plugin_MCodeExecute;

    /* 5. 打印初始化完成日志 */
    MLOGF(I, "MyPlugin: Custom template plugin initialized!\r\n");

    /* 未用上的系统与状态监听 Hook 示例 */ 
    // grbl.on_state_change：系统状态改变
    // grbl.on_reset：系统复位
    // grbl.on_motion_completed：运动完成

    /* 未用上的指令与代码扩展 Hook */
    // grbl.on_unknown_sys_command：自定义系统 $ 指令
    // grbl.on_gcode_message：G 代码注释消息
    // grbl.on_gcode_comment 代码普通注释
}
