/*******************************************************************************
 * @file    foc_optimization.h
 * @brief   Per-motor current, voltage, and inverter compensation algorithms
 ******************************************************************************/

#ifndef FOC_OPTIMIZATION_H
#define FOC_OPTIMIZATION_H

#include "foc_modulation.h"
#include "foc_pid.h"

typedef struct {
    foc_scalar_t qA;    /**< A 相值 */
    foc_scalar_t qB;    /**< B 相值 */
    foc_scalar_t qC;    /**< C 相值 */
} foc_abc_t;

typedef struct {
    foc_pid_params_t tVoltagePid;   /**< 电压环 PID 参数 */
    foc_scalar_t qBaseSpeed;        /**< 基速，pu */
    foc_scalar_t qVoltageLimit;     /**< 电压限幅，pu */
    foc_scalar_t qMinimumId;        /**< 最小 Id 限幅（负值） */
} foc_field_weakening_params_t;

typedef struct {
    foc_field_weakening_params_t tParams;   /**< 弱磁参数 */
    foc_pid_t tVoltagePid;                  /**< 电压环 PID 实例 */
} foc_field_weakening_t;

typedef struct {
    foc_scalar_t qCompensation;     /**< 死区补偿量，pu */
    foc_scalar_t qCurrentThreshold; /**< 电流过零阈值 */
} foc_deadtime_params_t;

/**
 * @brief  MTPA 计算：给定 Iq 和电机参数，计算最优 Id
 * @param  qFlux   永磁磁链
 * @param  qLd     D 轴电感
 * @param  qLq     Q 轴电感
 * @param  qIq     Q 轴电流
 * @param  pqId    输出最优 Id
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t foc_mtpa_Calculate(foc_scalar_t qFlux,
                                foc_scalar_t qLd,
                                foc_scalar_t qLq,
                                foc_scalar_t qIq,
                                foc_scalar_t *pqId);
/**
 * @brief  初始化弱磁控制实例
 * @param  ptWeakening  弱磁实例指针
 * @param  ptParams     参数（电压环 PID、基速、电压限幅等）
 * @return              FOC_RESULT_OK 或错误码
 */
foc_result_t foc_field_weakening_Init(
    foc_field_weakening_t *ptWeakening,
    const foc_field_weakening_params_t *ptParams);
/**
 * @brief  复位弱磁控制器状态
 * @param  ptWeakening  弱磁实例指针
 */
void foc_field_weakening_Reset(foc_field_weakening_t *ptWeakening);
/**
 * @brief  执行一步弱磁控制，输出 Id 补偿值
 * @param  ptWeakening      弱磁实例指针
 * @param  qElectricalSpeed 电速度
 * @param  ptVoltage        dq 电压参考值
 * @return                  Id 补偿量（负值）
 */
foc_scalar_t foc_field_weakening_Step(
    foc_field_weakening_t *ptWeakening,
    foc_scalar_t qElectricalSpeed,
    const foc_dq_t *ptVoltage);
/**
 * @brief  死区补偿，根据电流极性修正占空比
 * @param  ptParams  死区参数（补偿量、电流阈值）
 * @param  ptCurrent 三相电流
 * @param  ptDuty    输入/输出占空比（经补偿修正）
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_deadtime_Compensate(
    const foc_deadtime_params_t *ptParams,
    const foc_abc_t *ptCurrent,
    foc_duty_abc_t *ptDuty);
/**
 * @brief  相位延迟补偿，根据速度修正电角度
 * @param  tAngle               原始电角度
 * @param  qElectricalSpeed     电速度
 * @param  ptDelayTurnsPerSpeed 延迟系数（圈数/速度）
 * @param  qDirectionOffset     方向偏移
 * @param  ptCompensated        输出补偿后的角度
 * @return                      FOC_RESULT_OK 或错误码
 */
foc_result_t foc_phase_delay_Compensate(
    foc_angle_t tAngle,
    foc_scalar_t qElectricalSpeed,
    const foc_gain_t *ptDelayTurnsPerSpeed,
    foc_scalar_t qDirectionOffset,
    foc_angle_t *ptCompensated);

#endif /* FOC_OPTIMIZATION_H */
