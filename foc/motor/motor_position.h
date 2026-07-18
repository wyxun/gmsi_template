/*******************************************************************************
 * @file    motor_position.h
 * @brief   Common motor position-source contract and stateless helpers
 ******************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_core.h"

typedef enum {
    FOC_POSITION_VALID_NONE = 0U,
    FOC_POSITION_VALID_ELECTRICAL_ANGLE = 1U << 0,
    FOC_POSITION_VALID_ELECTRICAL_SPEED = 1U << 1,
    FOC_POSITION_VALID_MECHANICAL_ANGLE = 1U << 2,
    FOC_POSITION_VALID_MECHANICAL_SPEED = 1U << 3,
    FOC_POSITION_VALID_MULTI_TURN = 1U << 4,
} foc_position_valid_flag_e;

typedef enum {
    FOC_POSITION_FAULT_NONE = 0U,
    FOC_POSITION_FAULT_INVALID_DATA = 1U << 0,
    FOC_POSITION_FAULT_ILLEGAL_TRANSITION = 1U << 1,
} foc_position_fault_e;

typedef struct {
    foc_ab_t tCurrent;
    foc_ab_t tVoltage;
    foc_scalar_t qSamplePeriod;
    uint32_t wTimestamp;
} foc_position_input_t;

typedef struct {
    foc_angle_t tElectricalAngle;
    foc_angle_t tMechanicalAngle;
    foc_scalar_t qElectricalSpeed;
    foc_scalar_t qMechanicalSpeed;
    int32_t nMultiTurn;
    foc_scalar_t qConfidence;
    foc_position_valid_flag_e eValidFlags;
    uint32_t wFaults;
    uint32_t wTimestamp;
} foc_position_output_t;

typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;

typedef struct {
    foc_angle_t tMechanicalZero;
    foc_angle_t tElectricalOffset;
    uint8_t chPolePairs;
    int8_t chDirection;
} foc_position_config_t;

typedef struct {
    foc_position_valid_flag_e eRequiredValid;
    foc_angle_t tReferenceAngle;
    foc_scalar_t qReferenceSpeed;
    foc_scalar_t qMinimumConfidence;
    foc_scalar_t qMinimumSpeed;
    foc_scalar_t qMaximumAngleError;
    uint32_t wNow;
    uint32_t wMaximumAge;
} foc_position_qualification_t;

bool foc_position_source_IsValid(const foc_position_source_if_t *ptSource);
void foc_position_source_Reset(const foc_position_source_if_t *ptSource);
foc_result_t foc_position_source_Step(
    const foc_position_source_if_t *ptSource,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput);
foc_result_t foc_position_ApplyMechanicalConfig(
    const foc_position_config_t *ptConfig,
    foc_position_output_t *ptOutput);
bool foc_position_IsFresh(const foc_position_output_t *ptOutput,
                          foc_position_valid_flag_e eRequiredValid,
                          uint32_t wNow,
                          uint32_t wMaximumAge);
bool foc_position_IsQualified(
    const foc_position_output_t *ptOutput,
    const foc_position_qualification_t *ptQualification);
foc_scalar_t foc_position_ShortestError(foc_angle_t tTarget,
                                        foc_angle_t tActual);
foc_result_t foc_position_Blend(const foc_position_output_t *ptFrom,
                                const foc_position_output_t *ptTo,
                                foc_scalar_t qProgress,
                                foc_position_output_t *ptOutput);

#endif /* MOTOR_POSITION_H */
