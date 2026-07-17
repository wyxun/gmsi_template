/*******************************************************************************
 * @file    motor_control.h
 * @brief   High-frequency and cascaded low-frequency motor control loops
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"

foc_result_t motor_ControlStart(motor_handle_t *ptMotor,
                                motor_control_mode_t eMode);
void motor_ControlStop(motor_handle_t *ptMotor);
void motor_ControlSetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
void motor_ControlSetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
void motor_ControlSetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed);
void motor_ControlSetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition);
foc_result_t motor_ControlLowFrequencyStep(motor_handle_t *ptMotor);
foc_result_t motor_ControlHighFrequencyStep(motor_handle_t *ptMotor);

#endif /* MOTOR_CONTROL_H */
