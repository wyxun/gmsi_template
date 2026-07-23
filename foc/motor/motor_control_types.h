/*******************************************************************************
 * @file    motor_control_types.h
 * @brief   Per-motor control bindings, references, and runtime values
 ******************************************************************************/

#ifndef MOTOR_CONTROL_TYPES_H
#define MOTOR_CONTROL_TYPES_H

#include "foc_controller.h"
#include "foc_modulation.h"

typedef enum {
    MOTOR_CONTROL_VOLTAGE_OPEN_LOOP = 0,
    MOTOR_CONTROL_CURRENT,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
} motor_control_mode_e;

typedef enum {
    MOTOR_MODULATION_SVPWM = 0,
    MOTOR_MODULATION_SPWM,
    MOTOR_MODULATION_THIRD_HARMONIC,
} motor_modulation_e;

typedef struct {
    foc_pid_params_t    tIdParams;
    foc_pid_params_t    tIqParams;
    foc_pid_params_t    tSpeedParams;
    foc_pid_params_t    tPositionParams;
    foc_controller_if_t tIdController;
    foc_controller_if_t tIqController;
    foc_controller_if_t tSpeedController;
    foc_controller_if_t tPositionController;
    motor_modulation_e eModulation;
} motor_control_config_t;

typedef struct {
    void *pContext;
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);
} motor_step_controller_if_t;

typedef struct {
    void *pContext;
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);
    void (*fnTrack)(void *pContext,
                    foc_scalar_t qOutput,
                    foc_scalar_t qReference,
                    foc_scalar_t qFeedback);
} motor_tracking_controller_if_t;

typedef struct {
    motor_step_controller_if_t tIdController;
    motor_step_controller_if_t tIqController;
    motor_tracking_controller_if_t tSpeedController;
    motor_tracking_controller_if_t tPositionController;
    motor_modulation_e eModulation;
} motor_control_runtime_config_t;

typedef struct {
    motor_control_runtime_config_t tConfig;
    motor_control_mode_e eMode;
    foc_dq_t tCurrentReference;
    foc_dq_t tCurrent;
    foc_dq_t tVoltageReference;
    foc_dq_t tVoltage;
    foc_ab_t tVoltageAlphaBeta;
    foc_duty_abc_t tDuty;
    foc_scalar_t qSpeedReference;
    foc_scalar_t qPositionReference;
} motor_control_t;

#endif /* MOTOR_CONTROL_TYPES_H */
