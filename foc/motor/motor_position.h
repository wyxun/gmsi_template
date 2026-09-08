/****************************************************************************
 * @file    motor_position.h
 * @brief   Minimal single-active-source position provider interface.
 * @author  Codex
 * @date    2026-09-08
 ****************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "motor_params.h"

typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool bValid;
} motor_position_feedback_t;

typedef struct {
    foc_ab_t tCurrent;
    foc_ab_t tAppliedVoltage;
    foc_scalar_t qBusVoltage;
    foc_scalar_t qSamplePeriod;
} foc_observer_input_t;

typedef struct {
    foc_result_t (*fnInit)(void *pContext,
                           const motor_params_t *ptMotor,
                           foc_scalar_t qHighFrequencyPeriod);
    void (*fnReset)(void *pContext);
    int32_t (*fnSlowUpdate)(void *pContext);
    foc_result_t (*fnObserve)(void *pContext,
                              const foc_observer_input_t *ptInput);
    foc_result_t (*fnRead)(void *pContext,
                           motor_position_feedback_t *ptFeedback);
    foc_result_t (*fnCaptureElectricalZero)(void *pContext);
} motor_position_ops_t;

typedef struct {
    const motor_position_ops_t *ptOps;
    void *pContext;
} motor_position_t;

#endif /* MOTOR_POSITION_H */
