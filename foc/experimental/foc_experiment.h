/*******************************************************************************
 * @file    foc_experiment.h
 * @brief   Shared safety contract for motor-energizing experiments
 ******************************************************************************/

#ifndef FOC_EXPERIMENT_H
#define FOC_EXPERIMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_numeric.h"

typedef struct {
    bool bMotorStopped;
    bool bFault;
    foc_scalar_t qCurrentMagnitude;
    foc_scalar_t qElectricalSpeed;
    foc_scalar_t qBusVoltage;
} foc_experiment_guard_t;

typedef struct {
    foc_scalar_t qMaximumCurrent;
    foc_scalar_t qMaximumSpeed;
    foc_scalar_t qMinimumBusVoltage;
    foc_scalar_t qMaximumBusVoltage;
    uint32_t wTimeoutSamples;
    void *pContext;
    void (*fnEmergencyStop)(void *pContext);
} foc_experiment_safety_t;

foc_result_t foc_experiment_ValidateSafety(
    const foc_experiment_safety_t *ptSafety);
bool foc_experiment_IsSafe(const foc_experiment_safety_t *ptSafety,
                           const foc_experiment_guard_t *ptGuard);
void foc_experiment_EmergencyStop(
    const foc_experiment_safety_t *ptSafety);

#endif /* FOC_EXPERIMENT_H */
