/*******************************************************************************
 * @file    motor_diagnostic.h
 * @brief   Hardware bring-up diagnostics (build-gated, non-production)
 *
 * This module is compiled only when FOC_ENABLE_DIAGNOSTIC=1. It drives the
 * power stage exclusively through the narrow gated motor diagnostic API in
 * motor.h plus the public snapshot/raw-current queries; it never accesses
 * motor private members.
 ******************************************************************************/

#ifndef MOTOR_DIAGNOSTIC_H
#define MOTOR_DIAGNOSTIC_H

#include "motor.h"

#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC

/* Fixed-duty PWM/ADC wiring test. Enforces IDLE, no-fault, duty, current
 * and duration limits; returns FOC_RESULT_OK on success. */
foc_result_t motor_diagnostic_FixedDutyTest(motor_handle_t *ptMotor);

#endif /* FOC_ENABLE_DIAGNOSTIC */

#endif /* MOTOR_DIAGNOSTIC_H */
