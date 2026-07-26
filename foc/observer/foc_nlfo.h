/*******************************************************************************
 * @file    foc_nlfo.h
 * @brief   Normalized nonlinear flux observer
 *
 * ===== NLFO 原理概述 =====
 * 非线性磁链观测器基于定子磁链的电压模型和电流模型混合：
 *
 *   磁链电压模型（高速主导）：Ψαβ = ∫(vαβ - R*iαβ)dt
 *   磁链电流模型（低速修正）：Ψαβ = L*iαβ + Ψm * [cosθ, sinθ]ᵀ
 *
 *   角度提取：θ = atan2(Ψβ, Ψα) — 磁链矢量的方向即转子角度
 *
 * 相比 SMO，NLFO 在低速到中速都有较好的表现，因为它不依赖
 * 滑模抖振来提取反电势，而是在磁链层面做融合。
 *
 * ===== 关键参数 =====
 *   qIntegratorGain —— 纯积分器的替代（带通），避免饱和和漂移
 *   qCorrectionGain —— 电流模型校正强度，越高越偏向电流模型
 *   qMinimumFluxRatio —— 低于此磁链比认为估计无效，用于速度判据
 ******************************************************************************/

#ifndef FOC_NLFO_H
#define FOC_NLFO_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qIntegratorGain;   /**< 积分器增益 */
    foc_scalar_t qResistance;       /**< 定子电阻 */
    foc_scalar_t qAverageInductance; /**< 平均电感 */
    foc_scalar_t qFlux;             /**< 永磁磁链 */
    foc_scalar_t qCorrectionGain;   /**< 校正增益 */
    foc_scalar_t qMinimumFluxRatio; /**< 最小磁链比例 */
} foc_nlfo_params_t;

typedef struct {
    foc_nlfo_params_t tParams;      /**< NLFO 参数 */
    foc_gain_t tFluxInverse;        /**< 磁链倒数（预计算） */
    foc_scalar_t qFluxSquared;      /**< 磁链平方值 */
    foc_ab_t tStatorFlux;           /**< 状态：定子磁链 αβ */
    foc_angle_t tAngle;             /**< 状态：估计电角度 */
    foc_scalar_t qSpeed;            /**< 状态：估计电速度 */
    bool bHasAngle;                 /**< 状态：角度是否有效 */
} foc_nlfo_t;

/**
 * @brief  初始化非线性磁链观测器实例
 * @param  ptNlfo   观测器实例指针
 * @param  ptParams 参数（积分增益、电阻、平均电感、磁链、校正增益等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_nlfo_Init(foc_nlfo_t *ptNlfo,
                           const foc_nlfo_params_t *ptParams);
/**
 * @brief  复位非线性磁链观测器状态
 * @param  ptNlfo  观测器实例指针
 */
void foc_nlfo_Reset(foc_nlfo_t *ptNlfo);
/**
 * @brief  执行一步磁链观测器运算
 * @param  ptNlfo   观测器实例指针
 * @param  ptInput  输入（αβ 电流、电压、采样周期）
 * @param  ptOutput 输出（电角度、电速度、置信度等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_nlfo_Step(foc_nlfo_t *ptNlfo,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
/**
 * @brief  获取磁链观测器的位置源接口
 * @param  ptNlfo  观测器实例指针
 * @return         位置源接口
 */
foc_position_source_if_t foc_nlfo_PositionSourceInterface(foc_nlfo_t *ptNlfo);

#endif /* FOC_NLFO_H */
