/*******************************************************************************
 * @file    foc_experiment.c
 * @brief   Architecture-neutral experiment safety checks
 ******************************************************************************/

#include "foc_experiment.h"

#include <stddef.h>

foc_result_t foc_experiment_ValidateSafety(
    const foc_experiment_safety_t *ptSafety)
{
    if (ptSafety == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptSafety->qMaximumCurrent <= FOC_ZERO ||
        ptSafety->qMaximumSpeed < FOC_ZERO ||
        ptSafety->qMinimumBusVoltage < FOC_ZERO ||
        ptSafety->qMinimumBusVoltage >= ptSafety->qMaximumBusVoltage ||
        ptSafety->wTimeoutSamples == 0U ||
        ptSafety->fnEmergencyStop == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

bool foc_experiment_IsSafe(const foc_experiment_safety_t *ptSafety,
                           const foc_experiment_guard_t *ptGuard)
{
    if (ptSafety == NULL || ptGuard == NULL || ptGuard->bFault) {
        return false;
    }
    return foc_abs(ptGuard->qCurrentMagnitude) <=
               ptSafety->qMaximumCurrent &&
           foc_abs(ptGuard->qElectricalSpeed) <=
               ptSafety->qMaximumSpeed &&
           ptGuard->qBusVoltage >= ptSafety->qMinimumBusVoltage &&
           ptGuard->qBusVoltage <= ptSafety->qMaximumBusVoltage;
}

void foc_experiment_EmergencyStop(
    const foc_experiment_safety_t *ptSafety)
{
    if (ptSafety != NULL && ptSafety->fnEmergencyStop != NULL) {
        ptSafety->fnEmergencyStop(ptSafety->pContext);
    }
}
