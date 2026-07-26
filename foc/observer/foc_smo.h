/*******************************************************************************
 * @file    foc_smo.h
 * @brief   Normalized stationary-frame sliding-mode observer
 *
 * ===== SMO 原理概述 =====
 * 滑模观测器基于 PMSM 在 αβ 坐标系下的电流状态方程，通过滑模控制
 * 迫使估计电流跟踪实测电流，两者的误差包含反电势信息。
 *
 *   估计算法本质：
 *     d(îαβ)/dt = -R/L * îαβ + 1/L * vαβ - K/L * sign(îαβ - iαβ)
 *
 *   当滑模面 s = î - i = 0 达到时，等效控制量 ≈ 反电势 eαβ
 *   然后通过 PLL 或 atan2 从 eαβ 提取转子角度 θ
 *
 * ===== 适用场景 =====
 *   - 中高速段（通常 > 5% 额定速度）
 *   - 对凸极率不敏感（也可用于隐极电机）
 *   - 低速时反电势信噪比低，需要注入 HFI 配合
 *
 * ===== 配置要点 =====
 *   - qSlidingGain：必须大于|反电势|以维持滑模运动，过大会导致抖振
 *   - qBoundaryInverse：边界层厚度（饱和函数替代 sign 函数，减小抖振）
 *   - qEmfFilterAlpha：反电势滤波，越小越平滑但引入相位延迟
 *   - qMinimumBemf：低于此幅值认为反电势无效（对应速度过低）
 ******************************************************************************/

#ifndef FOC_SMO_H
#define FOC_SMO_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qModelGain;        /**< 模型增益 */
    foc_scalar_t qResistance;       /**< 定子电阻 */
    foc_scalar_t qSlidingGain;      /**< 滑模增益 */
    foc_scalar_t qBoundaryInverse;  /**< 边界层倒数 */
    foc_scalar_t qEmfFilterAlpha;   /**< 反电势低通滤波系数 */
    foc_scalar_t qMinimumBemf;      /**< 最小反电势幅值 */
} foc_smo_params_t;

typedef struct {
    foc_smo_params_t tParams;       /**< SMO 参数 */
    foc_gain_t tBoundaryInverse;    /**< 边界层倒数（预计算） */
    foc_ab_t tEstimatedCurrent;     /**< 状态：估计电流 αβ */
    foc_ab_t tBemf;                 /**< 状态：反电势 αβ */
    foc_angle_t tAngle;             /**< 状态：估计电角度 */
    foc_scalar_t qSpeed;            /**< 状态：估计电速度 */
    bool bHasAngle;                 /**< 状态：角度是否有效 */
} foc_smo_t;

/**
 * @brief  初始化滑模观测器实例
 * @param  ptSmo    SMO 实例指针
 * @param  ptParams 参数（模型增益、电阻、滑模增益、边界倒数、EMF 滤波系数等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_smo_Init(foc_smo_t *ptSmo,
                          const foc_smo_params_t *ptParams);
/**
 * @brief  复位滑模观测器状态
 * @param  ptSmo  SMO 实例指针
 */
void foc_smo_Reset(foc_smo_t *ptSmo);
/**
 * @brief  执行一步滑模观测器运算
 * @param  ptSmo    SMO 实例指针
 * @param  ptInput  输入（αβ 电流、电压、采样周期）
 * @param  ptOutput 输出（电角度、电速度、置信度等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_smo_Step(foc_smo_t *ptSmo,
                          const foc_position_input_t *ptInput,
                          foc_position_output_t *ptOutput);
/**
 * @brief  获取滑模观测器的位置源接口
 * @param  ptSmo  SMO 实例指针
 * @return        位置源接口
 */
foc_position_source_if_t foc_smo_PositionSourceInterface(foc_smo_t *ptSmo);

#endif /* FOC_SMO_H */
