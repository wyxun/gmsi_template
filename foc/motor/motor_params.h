/****************************************************************************
 * @file    motor_params.h
 * @brief   Physical motor parameters owned by the Motor domain object.
 * @author  Codex
 * @date    2026-09-08
 ****************************************************************************/

#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <stdint.h>

#include "foc_numeric.h"

typedef enum {
    MOTOR_PARAM_VALID_RS = (1UL << 0),
    MOTOR_PARAM_VALID_LD = (1UL << 1),
    MOTOR_PARAM_VALID_LQ = (1UL << 2),
    MOTOR_PARAM_VALID_FLUX = (1UL << 3),
} motor_param_valid_e;

typedef struct {
    foc_scalar_t qResistance;
    foc_scalar_t qInductanceD;
    foc_scalar_t qInductanceQ;
    foc_scalar_t qFlux;
    uint32_t wValidMask;
    uint8_t chPolePairs;
} motor_params_t;

#endif /* MOTOR_PARAMS_H */
