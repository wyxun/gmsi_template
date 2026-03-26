/**
 * @file   perfc_port_user.h
 * @brief  perf_counter 移植层头文件 — AT32F421F8P7 / Cortex-M4
 *         使用 SysTick 作为 perf_counter 计时基准。
 */

#if __PERFC_USE_USER_CUSTOM_PORTING__

#include "cmsis_compiler.h"

/*============================ TYPES =========================================*/
typedef uint32_t perfc_global_interrupt_status_t;

/*============================ MACROS ========================================*/
#ifndef __perfc_sync_barrier__
#   define __perfc_sync_barrier__(...) do { __DSB(); __ISB(); } while(0)
#endif

/*============================ IMPLEMENTATION ================================*/

static inline
perfc_global_interrupt_status_t perfc_port_disable_global_interrupt(void)
{
    perfc_global_interrupt_status_t tStatus = __get_PRIMASK();
    __disable_irq();
    return tStatus;
}

static inline
void perfc_port_resume_global_interrupt(perfc_global_interrupt_status_t tStatus)
{
    __set_PRIMASK(tStatus);
}

#endif /* __PERFC_USE_USER_CUSTOM_PORTING__ */
