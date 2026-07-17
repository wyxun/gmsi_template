/*******************************************************************************
 * @file    motor_types.h
 * @brief   电机对象核心类型定义
 ******************************************************************************/

#ifndef __MOTOR_TYPES_H__
#define __MOTOR_TYPES_H__

#include "foc_math_types.h"
#include "foc_angle.h"
#include "foc_hal.h"
#include "motor_control_types.h"
#include "observer_lib.h"

typedef enum {
    MOTOR_STATE_IDLE        = 0,
    MOTOR_STATE_START,
    MOTOR_STATE_OPEN_LOOP,
    MOTOR_STATE_CLOSE_LOOP,
    MOTOR_STATE_FAULT,
} motor_state_enum_t;

typedef enum {
    MOTOR_FAULT_NONE            = 0U,
    MOTOR_FAULT_HARDWARE        = 1U << 0,
    MOTOR_FAULT_CURRENT_SAMPLE  = 1U << 1,
    MOTOR_FAULT_INVALID_COMMAND = 1U << 2,
} motor_fault_t;

typedef struct {
    uint32_t wResistanceMilliOhm;
    uint32_t wLdMicroHenry;
    uint32_t wLqMicroHenry;
    uint32_t wBackEmfMicroVoltPerRadSec;
    uint32_t wInertiaNanoKgM2;
    uint32_t wRatedVoltageMilliVolt;
    uint32_t wRatedCurrentMilliAmp;
    uint8_t chPolePairs;
} motor_params_t;

typedef struct {
    foc_angle_t         tThetaE;
    q_type              qOmegaE;
    q_type              qId;
    q_type              qIq;
    q_type              qVbus;
    uint32_t            wFaults;
    motor_state_enum_t  eRunState;
} motor_state_t;

typedef struct {
    motor_params_t          tParams;
    foc_hal_t               tHal;
    motor_control_config_t  tControl;
    current_sensing_type_t  eTopology;
} motor_config_t;

typedef struct motor_handle_s {
    motor_params_t          tParams;
    motor_state_t           tRt;
    foc_hal_t               tHal;
    motor_control_t         tControl;
    phase_current_handle_t  tCurrent;
    sensor_interface_t     *ptSensor;
    observer_interface_t   *ptObserver;
} motor_handle_t;

#endif /* __MOTOR_TYPES_H__ */
