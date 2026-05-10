/**
 * @file   perfc_port_user.c
 * @brief  perf_counter porting layer — AT32F413RCT7 / Cortex-M4F
 *
 *  Uses SysTick as the system timer.
 *  SystemCoreClock is provided by AT32 CMSIS (system_at32f413.c).
 */

/*============================ INCLUDES ======================================*/
#undef __PERF_COUNT_PLATFORM_SPECIFIC_HEADER__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define __IMPLEMENT_PERF_COUNTER
#include "perf_counter.h"

#include "at32f413.h"   /* SysTick/SCB registers + SystemCoreClock */

#if defined(__clang__)
#   pragma clang diagnostic ignored "-Wunknown-warning-option"
#   pragma clang diagnostic ignored "-Wreserved-identifier"
#   pragma clang diagnostic ignored "-Wmissing-prototypes"
#   pragma clang diagnostic ignored "-Wimplicit-function-declaration"
#   pragma clang diagnostic ignored "-Wcast-align"
#endif

/*============================ IMPLEMENTATION ================================*/

#if __PERFC_USE_USER_CUSTOM_PORTING__

bool perfc_port_init_system_timer(bool bIsTimeOccupied)
{
    if (!bIsTimeOccupied) {
        uint32_t wReload = SystemCoreClock / 1000U - 1U;
        SysTick->LOAD = wReload;
        SysTick->VAL  = 0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                      | SysTick_CTRL_TICKINT_Msk
                      | SysTick_CTRL_ENABLE_Msk;
    }
    return true;
}

uint32_t perfc_port_get_system_timer_freq(void)
{
    return SystemCoreClock;
}

bool perfc_port_is_system_timer_ovf_pending(void)
{
    return (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U;
}

int64_t perfc_port_get_system_timer_top(void)
{
    return (int64_t)(SysTick->LOAD + 1U);
}

int64_t perfc_port_get_system_timer_elapsed(void)
{
    return (int64_t)(SysTick->LOAD - SysTick->VAL);
}

void perfc_port_clear_system_timer_ovf_pending(void)
{
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

void perfc_port_stop_system_timer_counting(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void perfc_port_clear_system_timer_counter(void)
{
    SysTick->VAL = 0UL;
}

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
