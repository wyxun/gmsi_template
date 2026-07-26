/*******************************************************************************
 * @file    foc_hfi.h
 * @brief   High-frequency injection and synchronous response demodulation
 *
 * ===== HFI 原理概述 =====
 * 高频注入法通过向 D 轴（或固定轴）注入高频正弦/方波电压信号，
 * 利用 PMSM 的凸极效应（Ld ≠ Lq）在电流响应中产生包含位置
 * 信息的调制信号，通过解调提取转子位置误差。
 *
 *   注入：在估算 D 轴上叠加高频电压 V_inj * sin(ωh*t)
 *   响应：Q 轴电流中会出现与位置误差成正比的信号
 *   解调：同步解调 → LF → 位置误差信号 → PLL → 转子角度
 *
 *   适用场景：
 *     - 零速和极低速（< 1% 额定速度）
 *     - SMO/NLFO 因反电势太小失效时的替代方案
 *     - 启动阶段 + 低速运行，然后过渡到 SMO
 *
 * ===== 工作流程 =====
 * 1. 在 D 轴注入高频载波（由 qPhaseStep 控制频率和相位）
 * 2. 解调 Q 轴电流中的响应分量
 * 3. 输出位置误差（ptOutput->qPositionError），由 PLL 跟踪
 * 4. bValid 标志位指示信噪比是否足够
 *
 * HFI 输出的是位置误差而非绝对角度，因此需要配合 PLL（foc_pll_t）
 * 使用，由 PLL 根据误差信号锁定角度。
 ******************************************************************************/

#ifndef FOC_HFI_H
#define FOC_HFI_H

#include <stdbool.h>

#include "foc_math.h"

typedef struct {
    foc_scalar_t qPhaseStep;            /**< 注入相位步进 */
    foc_scalar_t qInjectionAmplitude;   /**< 注入幅值 */
    foc_scalar_t qHighPassAlpha;        /**< 高通滤波系数 */
    foc_scalar_t qDemodAlpha;           /**< 解调低通滤波系数 */
    foc_gain_t tDemodGain;              /**< 解调增益 */
    foc_scalar_t qMinimumResponse;      /**< 最小响应幅值 */
} foc_hfi_params_t;

typedef struct {
    foc_scalar_t qInjectionD;   /**< 注入 D 轴分量 */
    foc_scalar_t qPositionError; /**< 估计的位置误差 */
    foc_scalar_t qResponse;     /**< 解调响应幅值 */
    bool bValid;                /**< 响应是否有效 */
} foc_hfi_output_t;

typedef struct {
    foc_hfi_params_t tParams;           /**< HFI 参数 */
    foc_angle_t tPhase;                 /**< 状态：注入相位 */
    foc_scalar_t qPreviousCurrentD;     /**< 状态：上次 D 轴电流 */
    foc_scalar_t qPreviousCarrier;      /**< 状态：上次载波信号 */
    foc_scalar_t qHighPass;             /**< 状态：高通滤波输出 */
    foc_scalar_t qDemodulated;          /**< 状态：解调输出 */
    bool bHasPreviousCarrier;           /**< 状态：是否有上次载波 */
} foc_hfi_t;

/**
 * @brief  初始化高频注入实例
 * @param  ptHfi    HFI 实例指针
 * @param  ptParams 参数（相位步进、注入幅值、高通/解调系数等）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hfi_Init(foc_hfi_t *ptHfi,
                          const foc_hfi_params_t *ptParams);
/**
 * @brief  复位高频注入状态
 * @param  ptHfi  HFI 实例指针
 */
void foc_hfi_Reset(foc_hfi_t *ptHfi);
/**
 * @brief  执行一步高频注入解调，输出位置误差估计
 * @param  ptHfi      HFI 实例指针
 * @param  qCurrentD  D 轴电流
 * @param  ptOutput   输出（注入 D 轴分量、位置误差、响应幅值、有效性标志）
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hfi_Step(foc_hfi_t *ptHfi,
                          foc_scalar_t qCurrentD,
                          foc_hfi_output_t *ptOutput);

#endif /* FOC_HFI_H */
