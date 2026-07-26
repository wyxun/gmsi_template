/*******************************************************************************
 * @file    foc_experiment.h
 * @brief   Shared safety contract for motor-energizing experiments
 ******************************************************************************/

#ifndef FOC_EXPERIMENT_H
#define FOC_EXPERIMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_numeric.h"

typedef struct {
    bool bMotorStopped;         /**< 电机已停止 */
    bool bFault;                /**< 有故障 */
    foc_scalar_t qCurrentMagnitude; /**< 当前电流幅值 */
    foc_scalar_t qElectricalSpeed;  /**< 电速度 */
    foc_scalar_t qBusVoltage;       /**< 母线电压 */
} foc_experiment_guard_t;

typedef struct {
    foc_scalar_t qMaximumCurrent;       /**< 最大允许电流 */
    foc_scalar_t qMaximumSpeed;         /**< 最大允许速度 */
    foc_scalar_t qMinimumBusVoltage;    /**< 最低母线电压 */
    foc_scalar_t qMaximumBusVoltage;    /**< 最高母线电压 */
    uint32_t wTimeoutSamples;           /**< 超时样本数 */
    void *pContext;                     /**< 紧急停止函数上下文 */
    void (*fnEmergencyStop)(void *pContext); /**< 紧急停止回调 */
} foc_experiment_safety_t;

/**
 * @brief  验证实验安全参数是否完整有效
 * @param  ptSafety  安全参数指针
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_experiment_ValidateSafety(
    const foc_experiment_safety_t *ptSafety);
/**
 * @brief  检查当前实验条件是否安全
 * @param  ptSafety  安全参数
 * @param  ptGuard   当前状态监护值
 * @return           true=安全, false=不安全
 */
bool foc_experiment_IsSafe(const foc_experiment_safety_t *ptSafety,
                           const foc_experiment_guard_t *ptGuard);
/**
 * @brief  紧急停止实验
 * @param  ptSafety  安全参数（含紧急停止函数）
 */
void foc_experiment_EmergencyStop(
    const foc_experiment_safety_t *ptSafety);

#endif /* FOC_EXPERIMENT_H */
