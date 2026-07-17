/*******************************************************************************
 * @file    foc_cogging.h
 * @brief   External-table cogging torque compensation
 ******************************************************************************/

#ifndef FOC_COGGING_H
#define FOC_COGGING_H

#include <stdint.h>

#include "foc_angle.h"

typedef struct {
    const foc_scalar_t *pqTable;
    uint16_t hwCount;
} foc_cogging_t;

foc_result_t foc_cogging_Init(foc_cogging_t *ptCogging,
                              const foc_scalar_t *pqTable,
                              uint16_t hwCount);
foc_result_t foc_cogging_Get(const foc_cogging_t *ptCogging,
                             foc_angle_t tMechanicalAngle,
                             foc_scalar_t *pqCompensation);

#endif /* FOC_COGGING_H */
