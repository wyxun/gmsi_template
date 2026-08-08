/*******************************************************************************
 * @file    foc_hf_profile.h
 * @brief   高频控制性能分析宏：提供编译期可裁剪的周期计数边界
 *
 * 这个文件专门给 20 kHz 电机控制步 motor_HighFrequencyStep() 做性能测量。
 * 它不提供业务 API，只提供一组宏和两个内联函数，用来记录整步或分阶段
 * 的 CPU cycle 消耗。
 *
 * 三档配置：
 *   - FOC_HF_PROFILE = 0
 *       生产构建使用。所有宏展开为空，不产生计时变量、不读取 DWT、
 *       不写回结果，运行开销为零。
 *   - FOC_HF_PROFILE_LEVEL = 1
 *       只测量整次高频步的总 cycle 数，用于快速判断链路整体耗时。
 *   - FOC_HF_PROFILE_LEVEL = 2
 *       进一步细分采样、位置源、算法、提交等阶段，用于定位热点。
 *
 * 为什么直接读取固定 DWT 地址：
 *   FOC 业务代码禁止包含 vendor/CMSIS 设备头，因此不能依赖 CMSIS 的
 *   DWT 符号。如果回退到 get_system_ticks()，每次调用会引入额外开销；
 *   在 LEVEL=2 下每个高频步最多会调用约 14 次，足以把 20 kHz ISR 撑爆。
 *   这里按 Cortex-M 架构固定地址直接读取 DWT->CYCCNT，单次只是一条 LDR。
 *
 * 典型用法：
 *   #include "foc_hf_profile.h"
 *
 *   FOC_HF_PROFILE_TOTAL_BEGIN(tTotal);
 *   ... 需要测量的代码 ...
 *   FOC_HF_PROFILE_TOTAL_END(tTotal, wTotalCycles);
 *
 *   LEVEL=2 时可用 FOC_HF_PROFILE_STAGE_BEGIN/END 包住单个阶段。
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
/* 初始化 DWT cycle counter。只需在 motor_Init() 中调用一次；
 * 重复调用是安全的，没有 DWT 的核会编译为空。 */
static inline void foc_hf_profile_InitCycles(void)
{
#if FOC_HF_PROFILE_HAS_DWT
    FOC_HF_COREDEBUG_DEMCR |= FOC_HF_DEMCR_TRCENA;
    FOC_HF_DWT_CYCCNT = 0U;
    FOC_HF_DWT_CTRL |= FOC_HF_DWT_CTRL_CYCCNTENA;
#endif
}

/* 读取当前 cycle 计数。profile 关闭时不会调用到这里。 */
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

/* 整步计时开始/结束。begin 定义一个局部 uint32_t，end 把差值写回目标。 */
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
/* 阶段计时开始/结束，仅 LEVEL=2 时参与编译，用于拆分单步热点。 */
#define FOC_HF_PROFILE_STAGE_BEGIN(name) \
    uint32_t name = foc_hf_profile_ReadCycles()
#define FOC_HF_PROFILE_STAGE_END(name, destination) \
    do { (destination) = foc_hf_profile_ReadCycles() - (name); } while (0)
#else
#define FOC_HF_PROFILE_STAGE_BEGIN(name)            do { } while (0)
#define FOC_HF_PROFILE_STAGE_END(name, destination) do { } while (0)
#endif

#endif /* FOC_HF_PROFILE_H */
