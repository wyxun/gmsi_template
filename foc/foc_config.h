/*******************************************************************************
 * @file    foc_config.h
 * @brief   Universal-FOC 框架统一配置入口
 *          所有编译期可调参数集中于此，其他模块通过 #include "foc_config.h" 读取
 ******************************************************************************/

#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#if defined(FOC_NUMERIC_FLOAT) && defined(FOC_NUMERIC_FIXED)
#error "Select only one FOC numeric backend"
#elif !defined(FOC_NUMERIC_FLOAT) && !defined(FOC_NUMERIC_FIXED)
#error "Select FOC_NUMERIC_FLOAT or FOC_NUMERIC_FIXED"
#endif

/* Compatibility only. New code selects behavior through foc_numeric.h. */
#if defined(FOC_NUMERIC_FLOAT)
#define FOC_USE_FPU_HARDWARE        1
#else
#define FOC_USE_FPU_HARDWARE        0
#endif

#define FOC_OFFSET_CALIB_TIMES      200U

#define FOC_DEFAULT_SENSING_TOPOLOGY    SENSING_TOPOLOGY_3P

#define FOC_HW_OCP_SUPPORT          1

#define FOC_OCP_THRESHOLD_A         10

#define FOC_DEFAULT_POLE_PAIRS      4

#define FOC_DEBUG_ENABLE            1

/* Experimental routines can energize a stopped motor.  Products must opt in
 * explicitly after providing current, speed, bus-voltage, and timeout limits. */
#ifndef FOC_ENABLE_EXPERIMENTAL_NSD
#define FOC_ENABLE_EXPERIMENTAL_NSD       0
#endif

#ifndef FOC_ENABLE_EXPERIMENTAL_IDENTIFY
#define FOC_ENABLE_EXPERIMENTAL_IDENTIFY  0
#endif

#ifndef FOC_ENABLE_MOTOR_VERIFY
#define FOC_ENABLE_MOTOR_VERIFY 1
#endif

#ifndef FOC_HF_PROFILE
#define FOC_HF_PROFILE 1
#endif

#ifndef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 2
#endif

#if !FOC_HF_PROFILE
#undef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 0
#endif

#if (FOC_HF_PROFILE_LEVEL < 0) || (FOC_HF_PROFILE_LEVEL > 2)
#error "FOC_HF_PROFILE_LEVEL must be 0, 1, or 2"
#endif

#include "foc_trig.h"

#endif /* __FOC_CONFIG_H__ */
