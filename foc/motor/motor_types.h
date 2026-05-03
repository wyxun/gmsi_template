/*******************************************************************************
 * @file    motor_types.h
 * @brief   电机对象核心类型定义
 ******************************************************************************/

#ifndef __MOTOR_TYPES_H__
#define __MOTOR_TYPES_H__

#include "foc_math_types.h"
#include "foc_hal_types.h"
#include "foc_hal_pwm.h"
#include "observer_lib.h"

typedef enum {
    MOTOR_STATE_IDLE        = 0,
    MOTOR_STATE_START,
    MOTOR_STATE_OPEN_LOOP,
    MOTOR_STATE_CLOSE_LOOP,
    MOTOR_STATE_FAULT,
} motor_state_enum_t;

typedef struct {
    q_type  qRs;
    q_type  qLd;
    q_type  qLq;
    q_type  qKe;
    q_type  qJ;
    q_type  qRatedVoltage;
    q_type  qRatedCurrent;
    uint8_t chPolePairs;
} motor_params_t;

typedef struct {
    q_type              qThetaE;
    q_type              qOmegaE;
    q_type              qId;
    q_type              qIq;
    q_type              qVbus;
    motor_state_enum_t  eRunState;
} motor_state_t;

typedef struct {
    motor_params_t          tParams;
    foc_pwm_ops_t           tPwm;
    foc_adc_ops_t           tAdc;
    current_sensing_type_t  eTopology;
} motor_config_t;

typedef struct motor_handle_s {
    motor_params_t          tParams;
    motor_state_t           tRt;
    foc_pwm_ops_t           tPwm;
    phase_current_handle_t  tCurrent;
    sensor_interface_t     *ptSensor;
    observer_interface_t   *ptObserver;
} motor_handle_t;

#endif /* __MOTOR_TYPES_H__ */
