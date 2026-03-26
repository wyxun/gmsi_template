/**
 * @file   perfc_port_user.c
 * @brief  perf_counter 移植层实现 — AT32F421F8P7 / Cortex-M4
 *
 *  使用 SysTick 作为系统计时器：
 *    - 时钟频率：system_core_clock（HICK 48 MHz）
 *    - 计数方式：倒计数（SysTick 是递减计数器）
 *    - Reload 值：system_core_clock / 1000 - 1  （1ms 节拍）
 *
 *  perf_counter 通过以下接口与硬件定时器对接：
 *    perfc_port_get_system_timer_freq()   → 返回计时器频率
 *    perfc_port_get_system_timer_top()    → 返回最大计数值（reload+1）
 *    perfc_port_get_system_timer_elapsed()→ 返回当前周期内已计数的 tick 数
 *    perfc_port_is_system_timer_ovf_pending() → 是否已触发溢出（即中断挂起）
 *    perfc_port_clear_system_timer_ovf_pending() → 清除溢出挂起位
 *    perfc_port_stop_system_timer_counting()  → 停止计时器
 *    perfc_port_clear_system_timer_counter()  → 清零计时器
 *    perfc_port_init_system_timer()           → 初始化/接管计时器
 */

/*============================ INCLUDES ======================================*/
#undef __PERF_COUNT_PLATFORM_SPECIFIC_HEADER__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define __IMPLEMENT_PERF_COUNTER
#include "perf_counter.h"

#include "at32f421.h"   /* SysTick registers via CMSIS core_cm4.h */

/* system_core_clock 由 system_at32f421.c 提供 */
extern unsigned int system_core_clock;

#if defined(__clang__)
#   pragma clang diagnostic ignored "-Wunknown-warning-option"
#   pragma clang diagnostic ignored "-Wreserved-identifier"
#   pragma clang diagnostic ignored "-Wmissing-prototypes"
#   pragma clang diagnostic ignored "-Wimplicit-function-declaration"
#   pragma clang diagnostic ignored "-Wcast-align"
#endif

/*============================ IMPLEMENTATION ================================*/

#if __PERFC_USE_USER_CUSTOM_PORTING__

/**
 * @brief  初始化 SysTick，配置为 1ms 节拍
 * @param  bIsTimeOccupied  true = 调用者已配置好 SysTick，本函数只接管计数读取；
 *                          false = 本函数自行配置 SysTick
 */
bool perfc_port_init_system_timer(bool bIsTimeOccupied)
{
    if (!bIsTimeOccupied) {
        /* 配置 SysTick：48MHz / 1000 = 48000 tick per ms，reload = 47999 */
        uint32_t wReload = system_core_clock / 1000U - 1U;
        SysTick->LOAD = wReload;
        SysTick->VAL  = 0UL;
        /* 时钟源：处理器时钟；使能中断；使能计数器 */
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                      | SysTick_CTRL_TICKINT_Msk
                      | SysTick_CTRL_ENABLE_Msk;
    }
    return true;
}

/** @brief 返回 SysTick 时钟频率（= 系统时钟频率） */
uint32_t perfc_port_get_system_timer_freq(void)
{
    return system_core_clock;
}

/** @brief SysTick 是否已触发溢出（COUNTFLAG 位） */
bool perfc_port_is_system_timer_ovf_pending(void)
{
    return (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U;
}

/** @brief SysTick 一个周期内的总 tick 数（reload + 1） */
int64_t perfc_port_get_system_timer_top(void)
{
    return (int64_t)(SysTick->LOAD + 1U);
}

/**
 * @brief 当前周期内已计数的 tick 数（从上次溢出到现在）
 *        SysTick 是递减计数器，elapsed = LOAD - VAL
 */
int64_t perfc_port_get_system_timer_elapsed(void)
{
    return (int64_t)(SysTick->LOAD - SysTick->VAL);
}

/** @brief 清除 SysTick 溢出中断挂起位 */
void perfc_port_clear_system_timer_ovf_pending(void)
{
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

/** @brief 停止 SysTick 计数 */
void perfc_port_stop_system_timer_counting(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/** @brief 清零 SysTick 当前计数值 */
void perfc_port_clear_system_timer_counter(void)
{
    SysTick->VAL = 0UL;
}

/* SP 读写辅助函数（供 perf_counter 内部使用） */
__attribute__((noinline))
uintptr_t __perfc_port_get_sp(void)
{
    uintptr_t result;
    __ASM volatile ("mov %0, sp" : "=r" (result));
    return result;
}

__attribute__((noinline))
void __perfc_port_set_sp(uintptr_t nSP)
{
    uint32_t nAlign8Padding = nSP;
    __ASM volatile ("mov sp, %0" : "=r" (nAlign8Padding));
}

#endif /* __PERFC_USE_USER_CUSTOM_PORTING__ */
