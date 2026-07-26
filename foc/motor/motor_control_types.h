/*******************************************************************************
 * @file    motor_control_types.h
 * @brief   Per-motor control bindings, references, and runtime values
 ******************************************************************************/

#ifndef MOTOR_CONTROL_TYPES_H
#define MOTOR_CONTROL_TYPES_H

#include "foc_controller.h"
#include "foc_modulation.h"

typedef enum {
    MOTOR_CONTROL_VOLTAGE_OPEN_LOOP = 0,    /**< 电压开环模式 */
    MOTOR_CONTROL_CURRENT,                   /**< 电流闭环模式 */
    MOTOR_CONTROL_SPEED,                     /**< 速度闭环模式 */
    MOTOR_CONTROL_POSITION,                  /**< 位置闭环模式 */
} motor_control_mode_e;

typedef enum {
    MOTOR_MODULATION_SVPWM = 0,              /**< SVPWM 调制 */
    MOTOR_MODULATION_SPWM,                   /**< SPWM 调制 */
    MOTOR_MODULATION_THIRD_HARMONIC,         /**< 三次谐波注入 SPWM */
} motor_modulation_e;

typedef struct {
    foc_pid_params_t    tIdParams;          /**< D 轴 PID 参数 */
    foc_pid_params_t    tIqParams;          /**< Q 轴 PID 参数 */
    foc_pid_params_t    tSpeedParams;       /**< 速度环 PID 参数 */
    foc_pid_params_t    tPositionParams;    /**< 位置环 PID 参数 */
    foc_controller_if_t tIdController;      /**< D 轴控制器接口 */
    foc_controller_if_t tIqController;      /**< Q 轴控制器接口 */
    foc_controller_if_t tSpeedController;   /**< 速度环控制器接口 */
    foc_controller_if_t tPositionController; /**< 位置环控制器接口 */
    motor_modulation_e eModulation;         /**< 调制方式 */
} motor_control_config_t;

typedef struct {
    void *pContext;                         /**< 控制器上下文 */
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);  /**< 单步控制函数 */
} motor_step_controller_if_t;

typedef struct {
    void *pContext;                         /**< 控制器上下文 */
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);  /**< 单步控制函数 */
    void (*fnTrack)(void *pContext,
                    foc_scalar_t qOutput,
                    foc_scalar_t qReference,
                    foc_scalar_t qFeedback);         /**< 跟踪函数 */
} motor_tracking_controller_if_t;

typedef struct {
    motor_step_controller_if_t tIdController;      /**< D 轴控制器 */
    motor_step_controller_if_t tIqController;      /**< Q 轴控制器 */
    motor_tracking_controller_if_t tSpeedController;   /**< 速度环控制器 */
    motor_tracking_controller_if_t tPositionController; /**< 位置环控制器 */
    motor_modulation_e eModulation;                     /**< 调制方式 */
} motor_control_runtime_config_t;

typedef struct {
    motor_control_runtime_config_t tConfig;  /**< 运行时控制配置 */
    motor_control_mode_e eMode;              /**< 当前控制模式 */
    foc_dq_t tCurrentReference;              /**< 电流参考值 */
    foc_dq_t tCurrent;                       /**< 实际电流反馈 */
    foc_dq_t tVoltageReference;              /**< 电压参考值 */
    foc_dq_t tVoltage;                       /**< 实际电压输出 */
    foc_ab_t tVoltageAlphaBeta;              /**< αβ 电压 */
    foc_duty_abc_t tDuty;                    /**< 三相占空比 */
    foc_scalar_t qSpeedReference;            /**< 速度参考值 */
    foc_scalar_t qPositionReference;         /**< 位置参考值 */
} motor_control_t;

#endif /* MOTOR_CONTROL_TYPES_H */
