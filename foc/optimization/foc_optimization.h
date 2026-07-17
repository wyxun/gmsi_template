/*******************************************************************************
 * @file    foc_optimization.h
 * @brief   Per-motor current, voltage, and inverter compensation algorithms
 ******************************************************************************/

#ifndef FOC_OPTIMIZATION_H
#define FOC_OPTIMIZATION_H

#include "foc_modulation.h"
#include "foc_pid.h"

typedef struct {
    foc_scalar_t qA;
    foc_scalar_t qB;
    foc_scalar_t qC;
} foc_abc_t;

typedef struct {
    foc_pid_params_t tVoltagePid;
    foc_scalar_t qBaseSpeed;
    foc_scalar_t qVoltageLimit;
    foc_scalar_t qMinimumId;
} foc_field_weakening_params_t;

typedef struct {
    foc_field_weakening_params_t tParams;
    foc_pid_t tVoltagePid;
} foc_field_weakening_t;

typedef struct {
    foc_scalar_t qCompensation;
    foc_scalar_t qCurrentThreshold;
} foc_deadtime_params_t;

foc_result_t foc_mtpa_Calculate(foc_scalar_t qFlux,
                                foc_scalar_t qLd,
                                foc_scalar_t qLq,
                                foc_scalar_t qIq,
                                foc_scalar_t *pqId);
foc_result_t foc_field_weakening_Init(
    foc_field_weakening_t *ptWeakening,
    const foc_field_weakening_params_t *ptParams);
void foc_field_weakening_Reset(foc_field_weakening_t *ptWeakening);
foc_scalar_t foc_field_weakening_Step(
    foc_field_weakening_t *ptWeakening,
    foc_scalar_t qElectricalSpeed,
    const foc_dq_t *ptVoltage);
foc_result_t foc_deadtime_Compensate(
    const foc_deadtime_params_t *ptParams,
    const foc_abc_t *ptCurrent,
    foc_duty_abc_t *ptDuty);
foc_result_t foc_phase_delay_Compensate(
    foc_angle_t tAngle,
    foc_scalar_t qElectricalSpeed,
    const foc_gain_t *ptDelayTurnsPerSpeed,
    foc_scalar_t qDirectionOffset,
    foc_angle_t *ptCompensated);

#endif /* FOC_OPTIMIZATION_H */
