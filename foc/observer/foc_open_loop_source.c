/*******************************************************************************
 * @file    foc_open_loop_source.c
 * @brief   Open-loop angle and speed generator implementing foc_position_source_if_t
 ******************************************************************************/

#include "foc_open_loop_source.h"

#include <stddef.h>

static void foc_open_loop_source_ResetImpl(void *pSourceContext)
{
    foc_open_loop_source_t *ptSource = (foc_open_loop_source_t *)pSourceContext;
    if (ptSource != NULL) {
        ptSource->tAngle = (foc_angle_t){0U};
        ptSource->qSpeed = FOC_ZERO;
    }
}

static foc_result_t foc_open_loop_source_StepImpl(void *pSourceContext,
                                                   const foc_position_input_t *ptInput,
                                                   foc_position_output_t *ptOutput)
{
    foc_open_loop_source_t *ptSource = (foc_open_loop_source_t *)pSourceContext;
    if (ptSource == NULL || ptInput == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptInput->qSamplePeriod <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    foc_scalar_t qAccel = (ptSource->qAcceleration < FOC_ZERO) ?
                          foc_sub_sat(FOC_ZERO, ptSource->qAcceleration) :
                          ptSource->qAcceleration;
    foc_scalar_t qDeltaSpeed = foc_mul_wide(qAccel, ptInput->qSamplePeriod);
    foc_scalar_t qTarget = ptSource->qTargetSpeed;
    foc_scalar_t qSpeed = ptSource->qSpeed;

    if (qSpeed < qTarget) {
        qSpeed = foc_add_sat(qSpeed, qDeltaSpeed);
        if (qSpeed > qTarget) {
            qSpeed = qTarget;
        }
    } else if (qSpeed > qTarget) {
        qSpeed = foc_sub_sat(qSpeed, qDeltaSpeed);
        if (qSpeed < qTarget) {
            qSpeed = qTarget;
        }
    }
    ptSource->qSpeed = qSpeed;

    ptSource->tAngle = foc_angle_add_scalar(
        ptSource->tAngle, foc_mul_wide(qSpeed, ptInput->qSamplePeriod));

    ptOutput->tElectricalAngle = ptSource->tAngle;
    ptOutput->qElectricalSpeed = qSpeed;
    ptOutput->tMechanicalAngle = (foc_angle_t){0U};
    ptOutput->qMechanicalSpeed = FOC_ZERO;
    ptOutput->nMultiTurn = 0;
    ptOutput->qConfidence = FOC_ONE;
    ptOutput->eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                            FOC_POSITION_VALID_ELECTRICAL_SPEED;
    ptOutput->wFaults = FOC_POSITION_FAULT_NONE;
    ptOutput->wTimestamp = ptInput->wTimestamp;
    return FOC_RESULT_OK;
}

foc_result_t foc_open_loop_source_Init(foc_open_loop_source_t *ptSource)
{
    if (ptSource == NULL) {
        return FOC_RESULT_NULL;
    }
    ptSource->tAngle = (foc_angle_t){0U};
    ptSource->qSpeed = FOC_ZERO;
    ptSource->qTargetSpeed = FOC_ZERO;
    ptSource->qAcceleration = FOC_ZERO;
    return FOC_RESULT_OK;
}

foc_result_t foc_open_loop_source_SetSpeed(foc_open_loop_source_t *ptSource,
                                            foc_scalar_t qSpeed)
{
    if (ptSource == NULL) {
        return FOC_RESULT_NULL;
    }
    ptSource->qSpeed = qSpeed;
    return FOC_RESULT_OK;
}

foc_result_t foc_open_loop_source_SetTargetSpeed(foc_open_loop_source_t *ptSource,
                                                  foc_scalar_t qTargetSpeed)
{
    if (ptSource == NULL) {
        return FOC_RESULT_NULL;
    }
    ptSource->qTargetSpeed = qTargetSpeed;
    return FOC_RESULT_OK;
}

foc_result_t foc_open_loop_source_SetAcceleration(foc_open_loop_source_t *ptSource,
                                                   foc_scalar_t qAcceleration)
{
    if (ptSource == NULL) {
        return FOC_RESULT_NULL;
    }
    ptSource->qAcceleration = qAcceleration;
    return FOC_RESULT_OK;
}

foc_result_t foc_open_loop_source_SetAngle(foc_open_loop_source_t *ptSource,
                                            foc_angle_t tAngle)
{
    if (ptSource == NULL) {
        return FOC_RESULT_NULL;
    }
    ptSource->tAngle = tAngle;
    return FOC_RESULT_OK;
}

foc_position_source_if_t foc_open_loop_source_GetInterface(foc_open_loop_source_t *ptSource)
{
    return (foc_position_source_if_t){
        .pSourceContext = ptSource,
        .fnReset = foc_open_loop_source_ResetImpl,
        .fnStep = foc_open_loop_source_StepImpl,
    };
}
