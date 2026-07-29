/**
 * @file   my_plugin_template.h
 * @brief  grblHAL + MODUS 自定义插件标准开发模板 (Header)
 * 
 * 本文件展示了如何编写兼容 grblHAL 框架的扩展插件头文件。
 */

#ifndef __MY_PLUGIN_TEMPLATE_H__
#define __MY_PLUGIN_TEMPLATE_H__

/* 
 * 关键技巧：规避 MODUS (mbase.h) 与 grblHAL (messages.h) 重复定义 message_t 的冲突。
 * 在包含 grblHAL 核心头文件之前重命名，包含完后解除重命名。
 */
#define message_t grblhal_message_t

#include "grbl.h"
#include "hal.h"
#include "core_handlers.h"
#include "settings.h"

#undef message_t

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. 自定义插件配置结构体 (设置项 $950 - $959)
 * ========================================================================= */

typedef union {
    uint8_t value;
    struct {
        uint8_t enable      : 1,  /* bit0: 插件功能总开关 (1=开启, 0=关闭) */
                auto_mode   : 1,  /* bit1: 自动模式 (1=自动, 0=手动) */
                reserved    : 6;
    };
} my_plugin_flags_t;

typedef struct {
    uint16_t          param_delay_ms; /* $950: 延时时间 (ms) */
    float             param_threshold;/* $951: 阈值参数 */
    my_plugin_flags_t flags;          /* $952: 标志位组合 */
} my_plugin_settings_t;

/* =========================================================================
 * 2. 预设默认初值 (可通过烧录前修改)
 * ========================================================================= */

#define MY_PLUGIN_DEFAULT_DELAY_MS   1000
#define MY_PLUGIN_DEFAULT_THRESHOLD  25.5f
#define MY_PLUGIN_DEFAULT_FLAGS      0x01  /* 默认 bit0=1 (使能) */

/* =========================================================================
 * 3. 插件对外初始化入口函数
 * ========================================================================= */

/**
 * @brief 初始化插件（在 app_plugins.c 的 my_plugin_init 中调用）
 */
void my_plugin_template_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MY_PLUGIN_TEMPLATE_H__ */
