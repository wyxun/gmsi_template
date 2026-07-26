/*******************************************************************************
 * @file    foc_nsd.h
 * @brief   Experimental non-blocking north/south polarity detection
 ******************************************************************************/

#ifndef FOC_NSD_H
#define FOC_NSD_H

#include "foc_config.h"
#include "foc_experiment.h"

typedef enum {
    FOC_NSD_IDLE = 0,       /**< 空闲 */
    FOC_NSD_SETTLE,         /**< 稳定等待 */
    FOC_NSD_POSITIVE,       /**< 正极性测试 */
    FOC_NSD_ZERO,           /**< 零电压等待 */
    FOC_NSD_NEGATIVE,       /**< 负极性测试 */
    FOC_NSD_COMPLETE,       /**< 检测完成 */
    FOC_NSD_ABORTED,        /**< 检测中止 */
} foc_nsd_state_t;

typedef struct {
    foc_experiment_safety_t tSafety;    /**< 实验安全参数 */
    foc_scalar_t qTestVoltage;          /**< 测试电压 */
    uint32_t wSettleSamples;            /**< 稳定等待样本数 */
    uint32_t wPositiveSamples;          /**< 正极性阶段样本数 */
    uint32_t wZeroSamples;              /**< 零电压阶段样本数 */
    uint32_t wNegativeSamples;          /**< 负极性阶段样本数 */
} foc_nsd_params_t;

typedef struct {
    foc_scalar_t qVoltageD;     /**< D 轴电压 */
    bool bReversePolarity;      /**< 是否极性反转 */
    bool bComplete;             /**< 检测是否完成 */
    foc_nsd_state_t eState;     /**< 当前状态 */
} foc_nsd_output_t;

typedef struct {
    foc_nsd_params_t tParams;           /**< NSD 参数 */
    foc_nsd_state_t eState;             /**< 当前状态 */
    uint32_t wStateSamples;             /**< 当前阶段已过样本数 */
    uint32_t wTotalSamples;             /**< 总样本数计数 */
    foc_scalar_t qPositiveResponse;     /**< 正极性响应 */
    foc_scalar_t qNegativeResponse;     /**< 负极性响应 */
    bool bReversePolarity;              /**< 是否极性反转 */
    bool bComplete;                     /**< 是否完成 */
} foc_nsd_t;

/**
 * @brief  初始化 NSD（N/S 极性检测）实例
 * @param  ptNsd    NSD 实例指针
 * @param  ptParams 参数（测试电压、各阶段样本数）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_nsd_Init(foc_nsd_t *ptNsd,
                          const foc_nsd_params_t *ptParams);
/**
 * @brief  启动 NSD 检测流程
 * @param  ptNsd    NSD 实例指针
 * @param  ptGuard  当前电机状态（确认安全）
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t foc_nsd_Start(foc_nsd_t *ptNsd,
                           const foc_experiment_guard_t *ptGuard);
/**
 * @brief  执行一步 NSD 极性检测
 * @param  ptNsd               NSD 实例指针
 * @param  ptGuard             当前电机状态（安全监护）
 * @param  qDemodulatedResponse 解调响应值
 * @param  ptOutput             输出（电压 D 轴分量、极性反转标志、完成标志）
 * @return                      FOC_RESULT_OK 或错误码
 */
foc_result_t foc_nsd_Step(foc_nsd_t *ptNsd,
                          const foc_experiment_guard_t *ptGuard,
                          foc_scalar_t qDemodulatedResponse,
                          foc_nsd_output_t *ptOutput);

#endif /* FOC_NSD_H */
