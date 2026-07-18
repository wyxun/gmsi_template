/*******************************************************************************
 * @file    motor_control.h
 * @brief   High-frequency and cascaded low-frequency motor control loops
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"

/* HF and LF may run concurrently. Each step is non-reentrant per motor;
 * attempting to enter the same step again returns FOC_RESULT_BUSY. */

void motor_SetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
void motor_SetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ);
void motor_SetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed);
void motor_SetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition);
foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor);
foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor);

#endif /* MOTOR_CONTROL_H */
