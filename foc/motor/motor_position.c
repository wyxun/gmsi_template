/*******************************************************************************
 * @file    motor_position.c
 * @brief   Common motor position-source dispatch and stateless helpers
 ******************************************************************************/

#include "motor_position.h"

#include <limits.h>
#include <stddef.h>

static foc_scalar_t motor_position_lerp_scalar(foc_scalar_t qFrom,
                                                foc_scalar_t qTo,
                                                foc_scalar_t qProgress)
{
    qProgress = foc_sat(qProgress, FOC_ZERO, FOC_ONE);
    if (qProgress == FOC_ZERO) {
        return qFrom;
    }
    if (qProgress == FOC_ONE) {
        return qTo;
    }
#if defined(FOC_NUMERIC_FIXED)
    {
        int64_t nDelta = (int64_t)qTo - (int64_t)qFrom;
        int64_t nValue = (int64_t)qFrom +
            (nDelta * (int64_t)qProgress) / (int64_t)FOC_ONE;

        if (nValue > INT32_MAX) {
            return INT32_MAX;
        }
        if (nValue < INT32_MIN) {
            return INT32_MIN;
        }
        return (foc_scalar_t)nValue;
    }
#else
    return qFrom + (qTo - qFrom) * qProgress;
#endif
}

bool foc_position_source_IsValid(const foc_position_source_if_t *ptSource)
{
    return ptSource != NULL && ptSource->fnStep != NULL;
}

void foc_position_source_Reset(const foc_position_source_if_t *ptSource)
{
    if (foc_position_source_IsValid(ptSource) && ptSource->fnReset != NULL) {
        ptSource->fnReset(ptSource->pSourceContext);
    }
}

foc_result_t foc_position_source_Step(
    const foc_position_source_if_t *ptSource,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput)
{
    if (!foc_position_source_IsValid(ptSource) || ptInput == NULL ||
        ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    return ptSource->fnStep(ptSource->pSourceContext, ptInput, ptOutput);
}

foc_result_t foc_position_ApplyMechanicalConfig(
    const foc_position_config_t *ptConfig,
    foc_position_output_t *ptOutput)
{
    foc_scalar_t qMechanical;
    foc_scalar_t qElectrical;

    if (ptConfig == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->chPolePairs == 0U ||
        (ptConfig->chDirection != 1 && ptConfig->chDirection != -1)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptOutput->eValidFlags = (foc_position_valid_flag_e)(
        ptOutput->eValidFlags &
        ~(FOC_POSITION_VALID_ELECTRICAL_ANGLE |
          FOC_POSITION_VALID_ELECTRICAL_SPEED));
    if ((ptOutput->eValidFlags & FOC_POSITION_VALID_MECHANICAL_ANGLE) !=
        0U) {
        qMechanical = foc_angle_diff(ptOutput->tMechanicalAngle,
                                     ptConfig->tMechanicalZero);
        if (ptConfig->chDirection < 0) {
            qMechanical = -qMechanical;
        }
        qElectrical = foc_mul_wide(
            qMechanical, foc_from_float((float)ptConfig->chPolePairs));
        ptOutput->tElectricalAngle = foc_angle_add(
            foc_angle_from_scalar(qElectrical), ptConfig->tElectricalOffset);
        ptOutput->eValidFlags = (foc_position_valid_flag_e)(
            ptOutput->eValidFlags |
            FOC_POSITION_VALID_ELECTRICAL_ANGLE);
    } else {
        ptOutput->tElectricalAngle = foc_angle_from_scalar(FOC_ZERO);
    }
    if ((ptOutput->eValidFlags & FOC_POSITION_VALID_MECHANICAL_SPEED) !=
        0U) {
        ptOutput->qElectricalSpeed = foc_mul_wide(
            ptOutput->qMechanicalSpeed,
            foc_from_float((float)ptConfig->chPolePairs));
        if (ptConfig->chDirection < 0) {
            ptOutput->qElectricalSpeed = foc_sub_sat(
                FOC_ZERO, ptOutput->qElectricalSpeed);
        }
        ptOutput->eValidFlags = (foc_position_valid_flag_e)(
            ptOutput->eValidFlags |
            FOC_POSITION_VALID_ELECTRICAL_SPEED);
    } else {
        ptOutput->qElectricalSpeed = FOC_ZERO;
    }
    return FOC_RESULT_OK;
}

bool foc_position_IsFresh(const foc_position_output_t *ptOutput,
                          foc_position_valid_flag_e eRequiredValid,
                          uint32_t wNow,
                          uint32_t wMaximumAge)
{
    return ptOutput != NULL && ptOutput->wFaults == 0U &&
           (ptOutput->eValidFlags & eRequiredValid) == eRequiredValid &&
           (wNow - ptOutput->wTimestamp) <= wMaximumAge;
}

bool foc_position_IsQualified(
    const foc_position_output_t *ptOutput,
    const foc_position_qualification_t *ptQualification)
{
    foc_scalar_t qSpeed;
    foc_scalar_t qAngleError;

    if (ptOutput == NULL || ptQualification == NULL ||
        !foc_position_IsFresh(
            ptOutput, ptQualification->eRequiredValid,
            ptQualification->wNow, ptQualification->wMaximumAge)) {
        return false;
    }
    qSpeed = ptOutput->qElectricalSpeed;
    qAngleError = foc_position_ShortestError(
        ptOutput->tElectricalAngle, ptQualification->tReferenceAngle);
    if (ptOutput->qConfidence < ptQualification->qMinimumConfidence ||
        foc_abs(qSpeed) < ptQualification->qMinimumSpeed ||
        foc_abs(qAngleError) > ptQualification->qMaximumAngleError) {
        return false;
    }
    if ((ptQualification->qReferenceSpeed > FOC_ZERO &&
         qSpeed <= FOC_ZERO) ||
        (ptQualification->qReferenceSpeed < FOC_ZERO &&
         qSpeed >= FOC_ZERO)) {
        return false;
    }
    return true;
}

foc_scalar_t foc_position_ShortestError(foc_angle_t tTarget,
                                        foc_angle_t tActual)
{
    return foc_angle_diff(tTarget, tActual);
}

foc_result_t foc_position_Blend(const foc_position_output_t *ptFrom,
                                const foc_position_output_t *ptTo,
                                foc_scalar_t qProgress,
                                foc_position_output_t *ptOutput)
{
    foc_scalar_t qError;

    if (ptFrom == NULL || ptTo == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qProgress < FOC_ZERO || qProgress > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    *ptOutput = (foc_position_output_t){0};
    if ((ptFrom->eValidFlags & ptTo->eValidFlags &
         FOC_POSITION_VALID_ELECTRICAL_ANGLE) != 0U) {
        qError = foc_angle_diff(ptTo->tElectricalAngle,
                                ptFrom->tElectricalAngle);
        ptOutput->tElectricalAngle = foc_angle_add_scalar(
            ptFrom->tElectricalAngle, foc_mul_pu(qError, qProgress));
        ptOutput->eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE;
    }
    if ((ptFrom->eValidFlags & ptTo->eValidFlags &
         FOC_POSITION_VALID_MECHANICAL_ANGLE) != 0U) {
        qError = foc_angle_diff(ptTo->tMechanicalAngle,
                                ptFrom->tMechanicalAngle);
        ptOutput->tMechanicalAngle = foc_angle_add_scalar(
            ptFrom->tMechanicalAngle, foc_mul_pu(qError, qProgress));
        ptOutput->eValidFlags = (foc_position_valid_flag_e)(
            ptOutput->eValidFlags |
            FOC_POSITION_VALID_MECHANICAL_ANGLE);
    }
    if ((ptFrom->eValidFlags & ptTo->eValidFlags &
         FOC_POSITION_VALID_MECHANICAL_SPEED) != 0U) {
        ptOutput->qMechanicalSpeed = motor_position_lerp_scalar(
            ptFrom->qMechanicalSpeed, ptTo->qMechanicalSpeed,
            qProgress);
        ptOutput->eValidFlags = (foc_position_valid_flag_e)(
            ptOutput->eValidFlags |
            FOC_POSITION_VALID_MECHANICAL_SPEED);
    }
    if ((ptFrom->eValidFlags & ptTo->eValidFlags &
         FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U) {
        ptOutput->qElectricalSpeed = motor_position_lerp_scalar(
            ptFrom->qElectricalSpeed, ptTo->qElectricalSpeed,
            qProgress);
        ptOutput->eValidFlags = (foc_position_valid_flag_e)(
            ptOutput->eValidFlags |
            FOC_POSITION_VALID_ELECTRICAL_SPEED);
    }
    ptOutput->qConfidence = motor_position_lerp_scalar(
        ptFrom->qConfidence, ptTo->qConfidence, qProgress);
    ptOutput->wFaults = ptFrom->wFaults | ptTo->wFaults;
    ptOutput->wTimestamp =
        (int32_t)(ptTo->wTimestamp - ptFrom->wTimestamp) >= 0
            ? ptFrom->wTimestamp : ptTo->wTimestamp;
    if (ptOutput->wFaults != 0U) {
        ptOutput->eValidFlags = FOC_POSITION_VALID_NONE;
    }
    return FOC_RESULT_OK;
}
