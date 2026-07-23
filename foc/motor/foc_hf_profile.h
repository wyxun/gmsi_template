/*******************************************************************************
 * @file    foc_hf_profile.h
 * @brief   Compile-time profiling macro boundary and zero-cost abstraction
 ******************************************************************************/

#ifndef FOC_HF_PROFILE_H
#define FOC_HF_PROFILE_H

#include "foc_config.h"
#include "perf_counter.h"
#include <stdint.h>

/* DWT->CYCCNT is a CoreSight register at a fixed address on every
 * Cortex-M3/M4/M7 and ARMv8-M Mainline core. Use the architectural
 * addresses directly: business code must not include vendor/CMSIS device
 * headers, so relying on the CMSIS `DWT` symbol being visible would
 * silently fall back to get_system_ticks() — a noinline perf_counter call
 * costing hundreds of cycles. At FOC_HF_PROFILE_LEVEL 2 that fallback runs
 * 14 times per control step and can push the 20 kHz ISR past the PWM
 * period, starving SysTick and the main loop. A single LDR keeps the
 * measurement overhead negligible. */
#if defined(DWT) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
#define FOC_HF_PROFILE_HAS_DWT    1
#else
#define FOC_HF_PROFILE_HAS_DWT    0
#endif

#if FOC_HF_PROFILE_HAS_DWT
#define FOC_HF_DWT_CTRL         (*(volatile uint32_t *)0xE0001000U)
#define FOC_HF_DWT_CYCCNT       (*(volatile uint32_t *)0xE0001004U)
#define FOC_HF_COREDEBUG_DEMCR  (*(volatile uint32_t *)0xE000EDFCU)
#define FOC_HF_DWT_CTRL_CYCCNTENA   (1UL << 0)
#define FOC_HF_DEMCR_TRCENA         (1UL << 24)
#endif

#if (FOC_HF_PROFILE_LEVEL >= 1)
/* Enable the DWT cycle counter once before any profiling read. Call from
 * motor_Init(); idempotent and a no-op on cores without DWT. */
static inline void foc_hf_profile_InitCycles(void)
{
#if FOC_HF_PROFILE_HAS_DWT
    FOC_HF_COREDEBUG_DEMCR |= FOC_HF_DEMCR_TRCENA;
    FOC_HF_DWT_CYCCNT = 0U;
    FOC_HF_DWT_CTRL |= FOC_HF_DWT_CTRL_CYCCNTENA;
#endif
}

static inline uint32_t foc_hf_profile_ReadCycles(void)
{
#if FOC_HF_PROFILE_HAS_DWT
    return FOC_HF_DWT_CYCCNT;
#elif defined(__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__)
    return 0U;
#else
    return (uint32_t)get_system_ticks();
#endif
}

#define FOC_HF_PROFILE_TOTAL_BEGIN(name) \
    uint32_t name = foc_hf_profile_ReadCycles()
#define FOC_HF_PROFILE_TOTAL_END(name, destination) \
    do { (destination) = foc_hf_profile_ReadCycles() - (name); } while (0)
#else
#define foc_hf_profile_InitCycles()               do { } while (0)
#define FOC_HF_PROFILE_TOTAL_BEGIN(name)            do { } while (0)
#define FOC_HF_PROFILE_TOTAL_END(name, destination) do { } while (0)
#endif

#if (FOC_HF_PROFILE_LEVEL >= 2)
#define FOC_HF_PROFILE_STAGE_BEGIN(name) \
    uint32_t name = foc_hf_profile_ReadCycles()
#define FOC_HF_PROFILE_STAGE_END(name, destination) \
    do { (destination) = foc_hf_profile_ReadCycles() - (name); } while (0)
#else
#define FOC_HF_PROFILE_STAGE_BEGIN(name)            do { } while (0)
#define FOC_HF_PROFILE_STAGE_END(name, destination) do { } while (0)
#endif

#endif /* FOC_HF_PROFILE_H */
