/*******************************************************************************
 * @file    motor.h
 * @brief   电机对象操作接口
 ******************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_types.h"

int motor_Init(motor_handle_t *ptMotor, motor_config_t *ptConfig);
void motor_Reset(motor_handle_t *ptMotor);
void motor_AttachSensor(motor_handle_t *ptMotor, sensor_interface_t *ptSensor);
void motor_AttachObserver(motor_handle_t *ptMotor, observer_interface_t *ptObserver);

#endif /* __MOTOR_H__ */
