/*******************************************************************************
 * @file    motor_position.h
 * @brief   Common motor position-source contract and stateless helpers
 *
 * ===== 位置源架构 =====
 * FOC 的位置信息通过统一的位置源接口（foc_position_source_if_t）获取。
 * 这种设计使得不同的角度/速度估计算法可以互换，而控制环代码无需修改。
 *
 * 位置源示例：
 *   - SMO （滑模观测器）：基于反电势，适合中高速
 *   - NLFO（非线性磁链观测器）：基于磁链，适合宽速度范围
 *   - HFI（高频注入）：零低速时注入高频信号提取转子凸极位置
 *   - Hall（霍尔传感器）：简单的 6 步角度估算
 *   - OpenLoop（开环发生器）：启动阶段使用的频率发生器
 *
 * ===== 源切换 =====
 * 电机启动时使用开环源（tDefaultOpenLoopSource），运行后通过
 * foc_observer_selector_t 平滑过渡到目标位置源（如 SMO 或 NLFO）。
 * 切换过程经历：资格判定 → 混合过渡 → 完成。切换期间两个源
 * 同时运行，角度通过线性插值从初始源过渡到目标源。
 *
 * ===== 有效性标志 =====
 * 每个源输出带有效性标志（eValidFlags），如电角度有效、电速度有效等。
 * 只有对应标志位置位时，下游才能使用该数据。这使单个源可以选择性地
 * 提供部分数据（如霍尔传感器只有角度无速度）。
 ******************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_core.h"

typedef enum {
    FOC_POSITION_VALID_NONE = 0U,                       /**< 无效 */
    FOC_POSITION_VALID_ELECTRICAL_ANGLE = 1U << 0,     /**< 电角度有效 */
    FOC_POSITION_VALID_ELECTRICAL_SPEED = 1U << 1,     /**< 电速度有效 */
    FOC_POSITION_VALID_MECHANICAL_ANGLE = 1U << 2,     /**< 机械角度有效 */
    FOC_POSITION_VALID_MECHANICAL_SPEED = 1U << 3,     /**< 机械速度有效 */
    FOC_POSITION_VALID_MULTI_TURN = 1U << 4,           /**< 多圈位置有效 */
} foc_position_valid_flag_e;

typedef enum {
    FOC_POSITION_FAULT_NONE = 0U,                       /**< 无故障 */
    FOC_POSITION_FAULT_INVALID_DATA = 1U << 0,         /**< 无效数据 */
    FOC_POSITION_FAULT_ILLEGAL_TRANSITION = 1U << 1,   /**< 非法跳变 */
} foc_position_fault_e;

typedef struct {
    foc_ab_t tCurrent;              /**< αβ 电流 */
    foc_ab_t tVoltage;              /**< αβ 电压 */
    foc_scalar_t qSamplePeriod;     /**< 采样周期，pu */
    uint32_t wTimestamp;            /**< 采样时间戳 */
} foc_position_input_t;

typedef struct {
    foc_angle_t tElectricalAngle;                 /**< 电角度 */
    foc_angle_t tMechanicalAngle;                 /**< 机械角度 */
    foc_scalar_t qElectricalSpeed;                /**< 电速度 */
    foc_scalar_t qMechanicalSpeed;                /**< 机械速度 */
    int32_t nMultiTurn;                           /**< 多圈位置计数 */
    foc_scalar_t qConfidence;                     /**< 置信度 [0, 1] */
    foc_position_valid_flag_e eValidFlags;        /**< 有效性标志位 */
    uint32_t wFaults;                             /**< 故障标志位 */
    uint32_t wTimestamp;                          /**< 输出时间戳 */
} foc_position_output_t;

typedef struct {
    void *pSourceContext;               /**< 源上下文 */
    void (*fnReset)(void *pSourceContext);    /**< 复位函数 */
    foc_result_t (*fnStep)(void *pSourceContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput); /**< 步进函数 */
} foc_position_source_if_t;

typedef struct {
    foc_angle_t tMechanicalZero;    /**< 机械零位偏移 */
    foc_angle_t tElectricalOffset;  /**< 电角度补偿偏移 */
    uint8_t chPolePairs;            /**< 极对数 */
    int8_t chDirection;             /**< 旋转方向（+1 或 -1） */
} foc_position_config_t;

typedef struct {
    foc_position_valid_flag_e eRequiredValid;   /**< 要求的有效性标志 */
    foc_angle_t tReferenceAngle;                /**< 参考角度（用于误差判定） */
    foc_scalar_t qReferenceSpeed;               /**< 参考速度 */
    foc_scalar_t qMinimumConfidence;            /**< 最低置信度 */
    foc_scalar_t qMinimumSpeed;                 /**< 最低速度要求 */
    foc_scalar_t qMaximumAngleError;            /**< 最大允许角度误差 */
    uint32_t wNow;                              /**< 当前时间戳 */
    uint32_t wMaximumAge;                       /**< 最大数据时效 */
} foc_position_qualification_t;

/**
 * @brief  检查位置源接口是否有效
 * @param  ptSource  位置源接口指针
 * @return           true=有效, false=无效
 */
bool foc_position_source_IsValid(const foc_position_source_if_t *ptSource);
/**
 * @brief  复位位置源
 * @param  ptSource  位置源接口指针
 */
void foc_position_source_Reset(const foc_position_source_if_t *ptSource);
/**
 * @brief  执行一步位置源运算
 * @param  ptSource  位置源接口指针
 * @param  ptInput   输入（αβ 电流、电压、采样周期）
 * @param  ptOutput  输出（角度、速度、置信度等）
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_position_source_Step(
    const foc_position_source_if_t *ptSource,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput);
/**
 * @brief  应用机械配置（极对数、方向、零位偏移）到位置输出
 * @param  ptConfig  机械配置
 * @param  ptOutput  输入/输出位置
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_position_ApplyMechanicalConfig(
    const foc_position_config_t *ptConfig,
    foc_position_output_t *ptOutput);
/**
 * @brief  检查位置输出是否在时效期内
 * @param  ptOutput        位置输出
 * @param  eRequiredValid  所需的有效标志
 * @param  wNow            当前时间戳
 * @param  wMaximumAge     最大允许时效，单位 tick
 * @return                 true=新鲜有效, false=过期
 */
bool foc_position_IsFresh(const foc_position_output_t *ptOutput,
                          foc_position_valid_flag_e eRequiredValid,
                          uint32_t wNow,
                          uint32_t wMaximumAge);
/**
 * @brief  检查位置输出是否满足切换资格条件
 * @param  ptOutput          位置输出
 * @param  ptQualification   资格判定条件
 * @return                   true=合格, false=不合格
 */
bool foc_position_IsQualified(
    const foc_position_output_t *ptOutput,
    const foc_position_qualification_t *ptQualification);
/**
 * @brief  计算目标角度到实际角度的最短有向误差
 * @param  tTarget  目标角度
 * @param  tActual  实际角度
 * @return          有向误差，范围 (-0.5, 0.5] 圈
 */
foc_scalar_t foc_position_ShortestError(foc_angle_t tTarget,
                                        foc_angle_t tActual);
/**
 * @brief  在两个位置输出之间按进度因子混合
 * @param  ptFrom     起始位置
 * @param  ptTo       目标位置
 * @param  qProgress  混合进度 [0, 1]
 * @param  ptOutput   输出混合结果
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_position_Blend(const foc_position_output_t *ptFrom,
                                const foc_position_output_t *ptTo,
                                foc_scalar_t qProgress,
                                foc_position_output_t *ptOutput);

#endif /* MOTOR_POSITION_H */
