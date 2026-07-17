/*******************************************************************************
 * @file    motor.h
 * @brief   电机对象操作接口
 ******************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_types.h"

foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig);
void motor_Reset(motor_handle_t *ptMotor);
foc_result_t motor_SetDuty(motor_handle_t *ptMotor,
                           q_type qDutyU,
                           q_type qDutyV,
                           q_type qDutyW);
foc_result_t motor_Enable(motor_handle_t *ptMotor, bool bEnable);
foc_result_t motor_CalibrateCurrent(motor_handle_t *ptMotor);
foc_result_t motor_GetRawCurrent(motor_handle_t *ptMotor,
                                 uint32_t *pwRawU,
                                 uint32_t *pwRawV,
                                 uint32_t *pwRawW);
foc_result_t motor_SampleCurrent(motor_handle_t *ptMotor);
void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_t eFault);
void motor_AttachSensor(motor_handle_t *ptMotor, sensor_interface_t *ptSensor);
void motor_AttachObserver(motor_handle_t *ptMotor, observer_interface_t *ptObserver);

#endif /* __MOTOR_H__ */
