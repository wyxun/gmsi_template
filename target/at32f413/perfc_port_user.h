/**
 * @file   perfc_port_user.h
 * @brief  perf_counter porting header — AT32F413RCT7 / Cortex-M4F
 *         Uses SysTick as the perf_counter time base.
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
