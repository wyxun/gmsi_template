/*******************************************************************************
 * @file    foc_core.c
 * @brief   FOC 核心算法 — Clarke / Park / iPark / SVPWM
 ******************************************************************************/

#include <stddef.h>
#include "foc_core.h"
#include "foc_math.h"

void foc_clarke(q_type qIu, q_type qIv, q_type qIw, foc_ab_t *ptAB)
{
    if (ptAB == NULL) { return; }
    ptAB->qAlphaOrD = qIu;
    ptAB->qBetaOrQ  = M_MUL(_Q(0.57735f), qIv - qIw);
}

void foc_park(const foc_ab_t *ptAB, q_type qTheta, foc_ab_t *ptDQ)
{
    if (ptAB == NULL || ptDQ == NULL) { return; }
    q_type s = foc_sin(qTheta);
    q_type c = foc_cos(qTheta);
    q_type qAlpha = ptAB->qAlphaOrD;
    q_type qBeta  = ptAB->qBetaOrQ;

    ptDQ->qAlphaOrD = M_MAD(qAlpha, c, M_MUL(qBeta, s));
    ptDQ->qBetaOrQ  = M_MSB(qAlpha, s, M_MUL(qBeta, c));
}

void foc_ipark(const foc_ab_t *ptDQ, q_type qTheta, foc_ab_t *ptAB)
{
    if (ptDQ == NULL || ptAB == NULL) { return; }
    q_type s = foc_sin(qTheta);
    q_type c = foc_cos(qTheta);
    q_type qD = ptDQ->qAlphaOrD;
    q_type qQ = ptDQ->qBetaOrQ;

    ptAB->qAlphaOrD = M_MSB(qQ, s, M_MUL(qD, c));
    ptAB->qBetaOrQ  = M_MAD(qD, s, M_MUL(qQ, c));
}

void foc_svpwm(const foc_ab_t *ptAB, q_type qVbus,
               q_type *pqDu, q_type *pqDv, q_type *pqDw)
{
    if (ptAB == NULL || pqDu == NULL || pqDv == NULL || pqDw == NULL) { return; }

    q_type qValpha = ptAB->qAlphaOrD;
    q_type qVbeta  = ptAB->qBetaOrQ;

    q_type qU1 = qVbeta;
    q_type qU2 = M_MUL(_Q(0.5f), -qVbeta + M_MUL(_Q(1.73205f), qValpha));
    q_type qU3 = -qU1 - qU2;

    q_type qMax = qU1;
    q_type qMin = qU1;

    if (qU2 > qMax) { qMax = qU2; }
    if (qU2 < qMin) { qMin = qU2; }
    if (qU3 > qMax) { qMax = qU3; }
    if (qU3 < qMin) { qMin = qU3; }

    q_type qVoffset = M_MUL(_Q(-0.5f), qMax + qMin);

    *pqDu = M_SAT((qU1 + qVoffset) + Q_HALF, 0, Q_ONE);
    *pqDv = M_SAT((qU2 + qVoffset) + Q_HALF, 0, Q_ONE);
    *pqDw = M_SAT((qU3 + qVoffset) + Q_HALF, 0, Q_ONE);
}
