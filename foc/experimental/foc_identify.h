/*******************************************************************************
 * @file    foc_identify.h
 * @brief   Experimental non-blocking PMSM parameter identification
 ******************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include "foc_config.h"
#include "foc_experiment.h"
#include "foc_core.h"

typedef enum {
    FOC_IDENTIFY_RS_LD_LQ = 0,  /**< 辨识电阻和电感 */
    FOC_IDENTIFY_FLUX,          /**< 辨识磁链 */
} foc_identify_mode_t;

typedef enum {
    FOC_IDENTIFY_IDLE = 0,      /**< 空闲 */
    FOC_IDENTIFY_RS_HALF,       /**< 半电压测电阻 */
    FOC_IDENTIFY_RS_FULL,       /**< 全电压测电阻 */
    FOC_IDENTIFY_LD_ZERO,       /**< 零电流测 Ld */
    FOC_IDENTIFY_LD_RISE,       /**< 上升沿测 Ld */
    FOC_IDENTIFY_LQ_ZERO,       /**< 零电流测 Lq */
    FOC_IDENTIFY_LQ_RISE,       /**< 上升沿测 Lq */
    FOC_IDENTIFY_FLUX_SETTLE,   /**< 磁链测量等待 */
    FOC_IDENTIFY_COMPLETE,      /**< 辨识完成 */
    FOC_IDENTIFY_ABORTED,       /**< 辨识中止 */
} foc_identify_state_t;

typedef struct {
    foc_experiment_safety_t tSafety;            /**< 实验安全参数 */
    foc_identify_mode_t eMode;                  /**< 辨识模式 */
    foc_scalar_t qHalfVoltage;                  /**< 半电压值 */
    foc_scalar_t qFullVoltage;                  /**< 全电压值 */
    foc_scalar_t qFluxVoltage;                  /**< 磁链测量电压 */
    foc_scalar_t qCurrentRiseRatio;             /**< 电流上升比例 */
    foc_scalar_t qResetCurrentThreshold;        /**< 复位电流阈值 */
    foc_scalar_t qInductanceTimeStep;           /**< 电感测量时间步长 */
    foc_scalar_t qMinimumFluxSpeed;             /**< 磁链测量最低速度 */
    foc_scalar_t qKnownResistance;              /**< 已知电阻值（可选） */
    uint32_t wSettleSamples;                    /**< 稳定等待样本数 */
    uint32_t wResetSamples;                     /**< 复位阶段样本数 */
    uint32_t wMaximumRiseSamples;               /**< 上升阶段最大样本数 */
} foc_identify_params_t;

typedef struct {
    foc_scalar_t qCurrentD;     /**< D 轴电流 */
    foc_scalar_t qCurrentQ;     /**< Q 轴电流 */
    foc_scalar_t qElectricalSpeed; /**< 电速度 */
} foc_identify_input_t;

typedef struct {
    foc_dq_t tVoltage;          /**< 当前 DQ 电压 */
    foc_scalar_t qResistance;   /**< 辨识的电阻值 */
    foc_scalar_t qInductanceD;  /**< 辨识的 D 轴电感 */
    foc_scalar_t qInductanceQ;  /**< 辨识的 Q 轴电感 */
    foc_scalar_t qFlux;         /**< 辨识的磁链 */
    bool bComplete;             /**< 是否完成 */
    foc_identify_state_t eState; /**< 当前状态 */
} foc_identify_output_t;

typedef struct {
    foc_identify_params_t tParams;  /**< 辨识参数 */
    foc_identify_state_t eState;    /**< 当前状态 */
    uint32_t wStateSamples;         /**< 当前阶段已过样本数 */
    uint32_t wTotalSamples;         /**< 总样本数计数 */
    foc_scalar_t qHalfCurrent;      /**< 半电压电流 */
    foc_scalar_t qFullCurrent;      /**< 全电压电流 */
    foc_scalar_t qRiseTime;         /**< 电流上升时间 */
    foc_scalar_t qResistance;       /**< 电阻中间结果 */
    foc_scalar_t qInductanceD;      /**< D 轴电感中间结果 */
    foc_scalar_t qInductanceQ;      /**< Q 轴电感中间结果 */
    foc_scalar_t qFlux;             /**< 磁链中间结果 */
    bool bComplete;                 /**< 是否完成 */
} foc_identify_t;

/**
 * @brief  初始化参数辨识实例
 * @param  ptIdentify  辨识实例指针
 * @param  ptParams    参数（电压等级、电流比例、时序等）
 * @return             FOC_RESULT_OK 或错误码
 */
foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_params_t *ptParams);
/**
 * @brief  启动参数辨识流程
 * @param  ptIdentify  辨识实例指针
 * @param  ptGuard     当前电机状态（确认安全）
 * @return             FOC_RESULT_OK 或错误码
 */
foc_result_t foc_identify_Start(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard);
/**
 * @brief  执行一步参数辨识
 * @param  ptIdentify  辨识实例指针
 * @param  ptGuard     当前电机状态（安全监护）
 * @param  ptInput     输入（dq 电流、电速度）
 * @param  ptOutput    输出（辨识结果：电阻、电感、磁链）
 * @return             FOC_RESULT_OK 或错误码
 */
foc_result_t foc_identify_Step(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard,
    const foc_identify_input_t *ptInput,
    foc_identify_output_t *ptOutput);

#endif /* FOC_IDENTIFY_H */
