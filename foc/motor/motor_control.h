/*******************************************************************************
 * @file    motor_control.h
 * @brief   High-frequency and cascaded low-frequency motor control loops
 *
 * ===== 双层控制架构 =====
 * FOC 控制分为两个不同频率的执行层级，通过 motor_handle_t 共享状态：
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │ 低频层 (LF) ~1kHz      主循环 / 低频调度                │
 * │                                                         │
 * │  motor_LowFrequencyStep()                                │
 * │    ├─ 速度 PID     ← qSpeedReference                    │
 * │    ├─ 位置 PID     ← qPositionReference                  │
 * │    ├─ 弱磁控制                                          │
 * │    ├─ MTPA 计算                                         │
 * │    └─ 更新电流环参考值 (tCurrentReference)                │
 * └──────────────────────┬──────────────────────────────────┘
 *                        │ 写入共享状态
 *                        ▼
 * ┌─────────────────────────────────────────────────────────┐
 * │ 高频层 (HF) ~20kHz    ADC 转换完成中断                    │
 * │                                                         │
 * │  motor_HighFrequencyStep()                               │
 * │    ├─ 电流采样 + 重建                                    │
 * │    ├─ Clarke 变换   (三相 → αβ)                          │
 * │    ├─ Park 变换     (αβ → dq)                            │
 * │    ├─ D 轴 PI 环                                        │
 * │    ├─ Q 轴 PI 环                                        │
 * │    ├─ 逆 Park 变换 (dq → αβ)                             │
 * │    └─ SVPWM        (αβ → 占空比输出)                     │
 * └─────────────────────────────────────────────────────────┘
 *
 * HF 和 LF 可并发运行。每个电机步骤不可重入：
 *   同一电机再次进入同一步骤返回 FOC_RESULT_BUSY。
 *   数据竞争通过 motor_sync_if_t enter/exit 保护。
 *
 * ===== 参考值设置 =====
 * motor_Set*Reference() 函数在任意时刻（包括运行时）设置控制环目标值。
 * 这些写入受同步保护，对高频步可见（重新进入高频步时生效）。
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"

/**
 * @brief  设置 DQ 轴电压参考值（开环模式使用）
 * @param  ptMotor  电机句柄
 * @param  qD       D 轴电压
 * @param  qQ       Q 轴电压
 */
void motor_SetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
/**
 * @brief  设置 DQ 轴电流参考值（电流闭环模式使用）
 * @param  ptMotor  电机句柄
 * @param  qD       D 轴电流
 * @param  qQ       Q 轴电流
 */
void motor_SetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
/**
 * @brief  设置速度参考值
 * @param  ptMotor  电机句柄
 * @param  qSpeed   速度值，单位：机械圈数/s
 */
void motor_SetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed);
/**
 * @brief  设置位置参考值
 * @param  ptMotor    电机句柄
 * @param  qPosition  位置值，单位：机械圈数
 */
void motor_SetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition);
/**
 * @brief  执行低频控制步（速度/位置外环，建议在主循环中周期性调用）
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor);
/**
 * @brief  执行并行观测源步进（应在非实时上下文调用，如主循环）
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 * @note    观测器计算（atan2 等）不得放入 20 kHz ISR 或 1 kHz Clock，
 *          会扰动控制/采样时序导致电机震荡。
 */
foc_result_t motor_ObservationStep(motor_handle_t *ptMotor);
/**
 * @brief  执行高频控制步（电流内环，通常在 ADC 中断中调用）
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor);

#endif /* MOTOR_CONTROL_H */
