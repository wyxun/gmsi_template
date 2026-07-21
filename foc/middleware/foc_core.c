/*******************************************************************************
 * @file    foc_core.c
 * @brief   Architecture-independent Clarke and Park transforms
 ******************************************************************************/

#include "foc_core.h"

#include <stddef.h>

foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB)
{
    const foc_scalar_t qInvSqrtThree = FOC_SCALAR(0.5773502692f);

    if (ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = qIu;
    ptAB->qBeta = foc_sub_sat(foc_mul_pu(qIv, qInvSqrtThree),
                              foc_mul_pu(qIw, qInvSqrtThree));
    return FOC_RESULT_OK;
}

foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ)
{
    if (ptAB == NULL || ptDQ == NULL) {
        return FOC_RESULT_NULL;
    }
    ptDQ->qD = foc_add_sat(foc_mul_pu(ptAB->qAlpha, qCos),
                           foc_mul_pu(ptAB->qBeta, qSin));
    ptDQ->qQ = foc_sub_sat(foc_mul_pu(ptAB->qBeta, qCos),
                           foc_mul_pu(ptAB->qAlpha, qSin));
    return FOC_RESULT_OK;
}

foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ)
{
    return foc_park_cached(ptAB, foc_angle_sin(tTheta), foc_angle_cos(tTheta), ptDQ);
}

foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB)
{
    if (ptDQ == NULL || ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = foc_sub_sat(foc_mul_pu(ptDQ->qD, qCos),
                               foc_mul_pu(ptDQ->qQ, qSin));
    ptAB->qBeta = foc_add_sat(foc_mul_pu(ptDQ->qD, qSin),
                              foc_mul_pu(ptDQ->qQ, qCos));
    return FOC_RESULT_OK;
}

foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB)
{
    return foc_ipark_cached(ptDQ, foc_angle_sin(tTheta), foc_angle_cos(tTheta), ptAB);
}
