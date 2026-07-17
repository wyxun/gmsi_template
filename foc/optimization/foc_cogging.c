/*******************************************************************************
 * @file    foc_cogging.c
 * @brief   Periodic linear interpolation without global calibration storage
 ******************************************************************************/

#include "foc_cogging.h"

#include <stddef.h>

foc_result_t foc_cogging_Init(foc_cogging_t *ptCogging,
                              const foc_scalar_t *pqTable,
                              uint16_t hwCount)
{
    if (ptCogging == NULL || pqTable == NULL) {
        return FOC_RESULT_NULL;
    }
    if (hwCount < 2U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptCogging->pqTable = pqTable;
    ptCogging->hwCount = hwCount;
    return FOC_RESULT_OK;
}

foc_result_t foc_cogging_Get(const foc_cogging_t *ptCogging,
                             foc_angle_t tMechanicalAngle,
                             foc_scalar_t *pqCompensation)
{
    foc_angle_t tWrapped;
    uint16_t hwIndex;
    uint16_t hwNext;
    foc_scalar_t qFraction;
    foc_scalar_t qDelta;

    if (ptCogging == NULL || pqCompensation == NULL ||
        ptCogging->pqTable == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCogging->hwCount < 2U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tWrapped = foc_angle_wrap(tMechanicalAngle);
#if defined(FOC_NUMERIC_FLOAT)
    {
        float fPosition = tWrapped.qTurns * (float)ptCogging->hwCount;
        hwIndex = (uint16_t)fPosition;
        qFraction = foc_from_float(fPosition - (float)hwIndex);
    }
#else
    {
        uint32_t wPosition = (uint32_t)tWrapped.qTurns *
                             (uint32_t)ptCogging->hwCount;
        hwIndex = (uint16_t)(wPosition >> FOC_Q_FRACTION_BITS);
        qFraction = (foc_scalar_t)(wPosition & (FOC_Q_SCALE - 1U));
    }
#endif
    if (hwIndex >= ptCogging->hwCount) {
        hwIndex = 0U;
        qFraction = FOC_ZERO;
    }
    hwNext = (uint16_t)(hwIndex + 1U);
    if (hwNext >= ptCogging->hwCount) {
        hwNext = 0U;
    }
    qDelta = foc_sub_sat(ptCogging->pqTable[hwNext],
                         ptCogging->pqTable[hwIndex]);
    *pqCompensation = foc_add_sat(
        ptCogging->pqTable[hwIndex], foc_mul_pu(qDelta, qFraction));
    return FOC_RESULT_OK;
}
