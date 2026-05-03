/*******************************************************************************
 * @file    foc_config.h
 * @brief   Universal-FOC 框架统一配置入口
 *          所有编译期可调参数集中于此，其他模块通过 #include "foc_config.h" 读取
 ******************************************************************************/

#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#define FOC_USE_FPU_HARDWARE        1

#define FOC_OFFSET_CALIB_TIMES      200U

#define FOC_DEFAULT_SENSING_TOPOLOGY    SENSING_TOPOLOGY_3P

#define FOC_HW_OCP_SUPPORT          1

#define FOC_OCP_THRESHOLD_A         10

#define FOC_DEFAULT_POLE_PAIRS      4

#define FOC_DEBUG_ENABLE            1

#endif /* __FOC_CONFIG_H__ */
