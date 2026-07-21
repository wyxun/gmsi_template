/*******************************************************************************
 * @file    foc_core.h
 * @brief   Architecture-independent Clarke and Park transforms
 ******************************************************************************/

#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_angle.h"
#include "foc_numeric.h"

typedef struct {
    foc_scalar_t qAlpha;
    foc_scalar_t qBeta;
} foc_ab_t;

typedef struct {
    foc_scalar_t qD;
    foc_scalar_t qQ;
} foc_dq_t;

foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB);
foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ);
foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ);
foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB);
foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB);

#endif /* FOC_CORE_H */
