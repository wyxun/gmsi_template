/*******************************************************************************
 * @file    foc_pid.h
 * @brief   Multi-instance PID controller with conditional anti-windup
 *
 * ===== PID 算法 =====
 * 增量式 PID（位置式输出），离散时间域公式：
 *
 *   u[k] = Kp * e[k]  +  Ki*Ts * Σe[i]  +  Kd/Ts * (e[k] - e[k-1])
 *
 *   其中 e[k] = Reference - Feedback
 *
 * ===== 参数说明 =====
 *   tKp        —— 比例增益，决定响应速度
 *   tKiTs      —— 积分增益 × 采样周期（已预乘 Ts，与 Ts 无关）
 *   tKdOverTs  —— 微分增益 / 采样周期（已预除 Ts，与 Ts 无关）
 *
 * 之所以预乘/预除 Ts，是因为 FOC 的采样周期在运行时已知且固定，
 * 这样做可以在 Step() 中省去每次的乘法/除法，适合高频 ISR 调用。
 *
 * ===== 条件积分抗饱和（Conditional Anti-Windup） =====
 * 当积分器累计导致输出超出限幅 [qOutputMin, qOutputMax] 时，
 * 积分项停止累加。这防止了输出饱和后积分器继续增长导致的
 * 大滞后过冲（windup 现象），在电流环尤其重要——电流环 PI
 * 的输出直接限定了电压占空比，一旦饱和必须快速退出。
 *
 * ===== Track 机制 =====
 * 用于无扰切换（Bumpless Transfer）。当从一个控制器切换到此
 * PID 时，Track() 根据已知的输出、参考、反馈反算出积分器状态，
 * 使 PID 从当前工作点开始输出，而不是从零积分器起步。
 ******************************************************************************/

#ifndef FOC_PID_H
#define FOC_PID_H

#include "foc_numeric.h"

typedef struct {
    foc_gain_t tKp;                 /**< 比例增益 */
    foc_gain_t tKiTs;               /**< 积分增益 * Ts */
    foc_gain_t tKdOverTs;           /**< 微分增益 / Ts */
    foc_scalar_t qOutputMinimum;    /**< 输出下限 */
    foc_scalar_t qOutputMaximum;    /**< 输出上限 */
    foc_scalar_t qIntegratorMinimum; /**< 积分器下限 */
    foc_scalar_t qIntegratorMaximum; /**< 积分器上限 */
} foc_pid_params_t;

typedef struct {
    foc_pid_params_t tParams;       /**< PID 参数 */
    foc_scalar_t qIntegrator;       /**< 状态：积分器 */
    foc_scalar_t qPreviousError;    /**< 状态：上次误差 */
} foc_pid_t;

/**
 * @brief  初始化 PID 控制器实例
 * @param  ptPid     PID 实例指针
 * @param  ptParams  参数（Kp, KiTs, KdOverTs, 限幅等）
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_pid_Init(foc_pid_t *ptPid,
                          const foc_pid_params_t *ptParams);
/**
 * @brief  复位 PID 积分器和历史误差
 * @param  ptPid  PID 实例指针
 */
void foc_pid_Reset(foc_pid_t *ptPid);
/**
 * @brief  无条件跟踪：用给定输出、参考和反馈更新积分器状态
 *         用于无扰切换场景
 * @param  ptPid       PID 实例指针
 * @param  qOutput     当前输出
 * @param  qReference  参考值
 * @param  qFeedback   反馈值
 */
void foc_pid_Track(foc_pid_t *ptPid,
                   foc_scalar_t qOutput,
                   foc_scalar_t qReference,
                   foc_scalar_t qFeedback);
/**
 * @brief  执行一步 PID 运算并输出控制量
 * @param  ptPid       PID 实例指针
 * @param  qReference  参考值
 * @param  qFeedback   反馈值
 * @return             控制输出
 */
foc_scalar_t foc_pid_Step(foc_pid_t *ptPid,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);

#endif /* FOC_PID_H */
