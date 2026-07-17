/*******************************************************************************
 * @file    foc_hall.c
 * @brief   Hall observer adapted from SguanFOC v3.1.0 with safe code handling
 ******************************************************************************/

#include "foc_hall.h"

#include <stddef.h>
#include <string.h>

#define FOC_HALL_INVALID_SECTOR 0xFFU

static foc_scalar_t hall_sector_angle(uint8_t chSector)
{
#if defined(FOC_NUMERIC_FLOAT)
    return (foc_scalar_t)chSector / 6.0f;
#else
    return (foc_scalar_t)(((uint32_t)chSector * FOC_Q_SCALE) / 6U);
#endif
}

static foc_scalar_t hall_edge_speed(int8_t chDirection,
                                    uint16_t hwTicks)
{
    if (hwTicks == 0U) {
        return FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    return (foc_scalar_t)chDirection / (6.0f * (foc_scalar_t)hwTicks);
#else
    return (foc_scalar_t)(((int32_t)chDirection * FOC_Q_SCALE) /
                          (6 * (int32_t)hwTicks));
#endif
}

void foc_hall_DefaultParams(foc_hall_params_t *ptParams)
{
    static const uint8_t achDefaultMap[8] = {
        FOC_HALL_INVALID_SECTOR, 5U, 3U, 4U,
        1U, 0U, 2U, FOC_HALL_INVALID_SECTOR,
    };

    if (ptParams != NULL) {
        memcpy(ptParams->achSectorByCode, achDefaultMap,
               sizeof(achDefaultMap));
        ptParams->hwInvalidTimeout = 2U;
        ptParams->qSpeedFilterAlpha = FOC_SCALAR(0.25f);
    }
}

foc_result_t foc_hall_Init(foc_hall_t *ptHall,
                           const foc_hall_params_t *ptParams)
{
    uint8_t chIndex;

    if (ptHall == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qSpeedFilterAlpha < FOC_ZERO ||
        ptParams->qSpeedFilterAlpha > FOC_ONE ||
        ptParams->hwInvalidTimeout == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    for (chIndex = 1U; chIndex <= 6U; chIndex++) {
        if (ptParams->achSectorByCode[chIndex] > 5U) {
            return FOC_RESULT_INVALID_ARGUMENT;
        }
    }
    memset(ptHall, 0, sizeof(*ptHall));
    ptHall->tParams = *ptParams;
    return FOC_RESULT_OK;
}

void foc_hall_Reset(foc_hall_t *ptHall)
{
    foc_hall_params_t tParams;

    if (ptHall != NULL) {
        tParams = ptHall->tParams;
        memset(ptHall, 0, sizeof(*ptHall));
        ptHall->tParams = tParams;
    }
}

foc_result_t foc_hall_Step(foc_hall_t *ptHall,
                           uint8_t chHallCode,
                           foc_observer_output_t *ptOutput)
{
    uint8_t chSector;
    int8_t chDelta;
    foc_scalar_t qRawSpeed;
    foc_scalar_t qSpeedDelta;

    if (ptHall == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (chHallCode > 7U ||
        ptHall->tParams.achSectorByCode[chHallCode] > 5U) {
        if (ptHall->hwInvalidSamples < UINT16_MAX) {
            ptHall->hwInvalidSamples++;
        }
        if (ptHall->hwInvalidSamples >=
            ptHall->tParams.hwInvalidTimeout) {
            ptHall->bValid = false;
        }
        *ptOutput = (foc_observer_output_t){
            .tAngle = ptHall->tAngle,
            .qSpeed = ptHall->qSpeed,
            .qConfidence = FOC_ZERO,
            .bValid = false,
        };
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    chSector = ptHall->tParams.achSectorByCode[chHallCode];
    ptHall->hwInvalidSamples = 0U;
    if (!ptHall->bInitialized) {
        ptHall->bInitialized = true;
        ptHall->chPreviousSector = chSector;
        ptHall->tAngle = foc_angle_from_scalar(hall_sector_angle(chSector));
        ptHall->hwTicksSinceEdge = 0U;
    } else {
        if (ptHall->hwTicksSinceEdge < UINT16_MAX) {
            ptHall->hwTicksSinceEdge++;
        }
        if (chSector != ptHall->chPreviousSector) {
            chDelta = (int8_t)chSector -
                      (int8_t)ptHall->chPreviousSector;
            if (chDelta == 1 || chDelta == -5) {
                chDelta = 1;
            } else if (chDelta == -1 || chDelta == 5) {
                chDelta = -1;
            } else {
                ptHall->bValid = false;
                ptHall->qConfidence = FOC_ZERO;
                ptOutput->bValid = false;
                return FOC_RESULT_INVALID_ARGUMENT;
            }
            qRawSpeed = hall_edge_speed(chDelta,
                                        ptHall->hwTicksSinceEdge);
            qSpeedDelta = foc_sub_sat(qRawSpeed, ptHall->qSpeed);
            ptHall->qSpeed = foc_add_sat(
                ptHall->qSpeed,
                foc_mul_pu(qSpeedDelta,
                           ptHall->tParams.qSpeedFilterAlpha));
            ptHall->tAngle =
                foc_angle_from_scalar(hall_sector_angle(chSector));
            ptHall->chPreviousSector = chSector;
            ptHall->hwTicksSinceEdge = 0U;
            ptHall->bValid = true;
            ptHall->qConfidence = FOC_ONE;
        } else if (ptHall->bValid) {
            ptHall->tAngle = foc_angle_from_scalar(
                foc_add_sat(ptHall->tAngle.qTurns, ptHall->qSpeed));
        }
    }
    *ptOutput = (foc_observer_output_t){
        .tAngle = ptHall->tAngle,
        .qSpeed = ptHall->qSpeed,
        .qConfidence = ptHall->qConfidence,
        .bValid = ptHall->bValid,
    };
    return FOC_RESULT_OK;
}

static void hall_interface_reset(void *pContext)
{
    foc_hall_Reset((foc_hall_t *)pContext);
}

static foc_result_t hall_interface_step(
    void *pContext,
    const foc_observer_input_t *ptInput,
    foc_observer_output_t *ptOutput)
{
    if (ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    return foc_hall_Step((foc_hall_t *)pContext,
                         ptInput->chHallCode, ptOutput);
}

foc_observer_if_t foc_hall_ObserverInterface(foc_hall_t *ptHall)
{
    foc_observer_if_t tInterface = {
        .pContext = ptHall,
        .fnReset = hall_interface_reset,
        .fnStep = hall_interface_step,
    };
    return tInterface;
}
