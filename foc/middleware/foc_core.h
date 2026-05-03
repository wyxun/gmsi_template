/*******************************************************************************
 * @file    foc_core.h
 * @brief   FOC 核心算法接口 — Clarke / Park / iPark / SVPWM
 ******************************************************************************/

#ifndef __FOC_CORE_H__
#define __FOC_CORE_H__

#include "foc_math_types.h"
#include "foc_math_port.h"

struct motor_handle_s;

typedef struct {
    q_type qAlphaOrD;
    q_type qBetaOrQ;
} foc_ab_t;

void foc_clarke(q_type qIu, q_type qIv, q_type qIw, foc_ab_t *ptAB);
void foc_park(const foc_ab_t *ptAB, q_type qTheta, foc_ab_t *ptDQ);
void foc_ipark(const foc_ab_t *ptDQ, q_type qTheta, foc_ab_t *ptAB);
void foc_svpwm(const foc_ab_t *ptAB, q_type qVbus,
               q_type *pqDu, q_type *pqDv, q_type *pqDw);

#endif /* __FOC_CORE_H__ */
