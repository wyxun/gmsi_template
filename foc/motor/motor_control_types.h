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

#endif /* MOTOR_CONTROL_TYPES_H */
