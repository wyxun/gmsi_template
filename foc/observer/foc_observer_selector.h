/*******************************************************************************
 * @file    foc_observer_selector.h
 * @brief   Qualified and blended observer source switching
 ******************************************************************************/

#ifndef FOC_OBSERVER_SELECTOR_H
#define FOC_OBSERVER_SELECTOR_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qMinimumConfidence;    /**< 切换最低置信度 */
    foc_scalar_t qMinimumSpeed;         /**< 切换最低速度 */
    foc_scalar_t qMaximumAngleError;    /**< 最大允许角度误差 */
    uint16_t hwStableSamples;           /**< 资格稳定样本数 */
    uint16_t hwBlendSamples;            /**< 混合过渡样本数 */
} foc_observer_selector_params_t;

typedef struct {
    foc_observer_selector_params_t tParams;     /**< 选择器参数 */
    const foc_position_source_if_t *ptActive;   /**< 当前激活的位置源 */
    const foc_position_source_if_t *ptTarget;   /**< 目标切换的位置源 */
    foc_position_output_t tActiveOutput;        /**< 当前源的最新输出 */
    foc_position_output_t tTargetOutput;        /**< 目标源的最新输出 */
    foc_scalar_t qBlendProgress;                /**< 混合进度 [0, 1] */
    foc_scalar_t qBlendIncrement;               /**< 每次混合增量 */
    uint16_t hwStableCount;                     /**< 资格稳定计数器 */
    uint16_t hwBlendCount;                      /**< 混合步数计数器 */
    bool bBlending;                             /**< 是否正在混合过渡 */
} foc_observer_selector_t;

/**
 * @brief  初始化观测器选择器实例
 * @param  ptSelector 选择器实例指针
 * @param  ptParams   参数（最小置信度、最小速度、最大角度误差等）
 * @param  ptInitial  初始位置源
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_observer_selector_Init(
    foc_observer_selector_t *ptSelector,
    const foc_observer_selector_params_t *ptParams,
    const foc_position_source_if_t *ptInitial);
/**
 * @brief  请求切换到目标位置源
 * @param  ptSelector  选择器实例指针
 * @param  ptTarget    目标位置源
 * @return             FOC_RESULT_OK 或错误码
 */
foc_result_t foc_observer_selector_Request(
    foc_observer_selector_t *ptSelector,
    const foc_position_source_if_t *ptTarget);
/**
 * @brief  取消当前切换请求
 * @param  ptSelector  选择器实例指针
 */
void foc_observer_selector_Cancel(foc_observer_selector_t *ptSelector);
/**
 * @brief  执行一步源切换逻辑（资格判定、混合过渡）
 * @param  ptSelector  选择器实例指针
 * @param  ptInput     输入（电流、电压、采样周期）
 * @param  ptOutput    输出（选中的角度、速度、置信度等）
 * @return             FOC_RESULT_OK 或错误码
 */
foc_result_t foc_observer_selector_Step(
    foc_observer_selector_t *ptSelector,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput);

#endif /* FOC_OBSERVER_SELECTOR_H */
