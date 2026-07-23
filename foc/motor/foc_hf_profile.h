/*******************************************************************************
 * @file    foc_hf_profile.h
 * @brief   Compile-time profiling macro boundary and zero-cost abstraction
 ******************************************************************************/

#ifndef FOC_HF_PROFILE_H
#define FOC_HF_PROFILE_H

#include "foc_config.h"
#include "perf_counter.h"
#include <stdint.h>

#if (FOC_HF_PROFILE_LEVEL >= 1)
static inline uint32_t foc_hf_profile_ReadCycles(void)
{
#if defined(__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__)
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
