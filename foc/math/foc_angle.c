/*******************************************************************************
 * @file    foc_angle.c
 * @brief   Normalized electrical-angle implementation
 ******************************************************************************/

#include "foc_angle.h"

#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

#define FOC_TWO_PI_F 6.2831853071795864769f

static foc_scalar_t angle_wrap_scalar(foc_scalar_t qTurns)
{
#if defined(FOC_NUMERIC_FLOAT)
    qTurns -= floorf(qTurns);
    return qTurns < FOC_ZERO ? qTurns + FOC_ONE : qTurns;
#else
    qTurns %= FOC_ONE;
    return qTurns < FOC_ZERO ? qTurns + FOC_ONE : qTurns;
#endif
}

foc_angle_t foc_angle_from_turns(float fTurns)
{
    return foc_angle_from_scalar(foc_from_float(fTurns));
}

foc_angle_t foc_angle_from_scalar(foc_scalar_t qTurns)
{
    foc_angle_t tAngle = { angle_wrap_scalar(qTurns) };
    return tAngle;
}

float foc_angle_to_turns(foc_angle_t tAngle)
{
    return foc_to_float(angle_wrap_scalar(tAngle.qTurns));
}

foc_angle_t foc_angle_wrap(foc_angle_t tAngle)
{
    tAngle.qTurns = angle_wrap_scalar(tAngle.qTurns);
    return tAngle;
}

foc_scalar_t foc_angle_diff(foc_angle_t tTarget,
                            foc_angle_t tActual)
{
    foc_scalar_t qDifference = angle_wrap_scalar(
        foc_sub_sat(tTarget.qTurns, tActual.qTurns));

    if (qDifference >= FOC_HALF) {
        qDifference = foc_sub_sat(qDifference, FOC_ONE);
    }
    return qDifference;
}

#if defined(FOC_NUMERIC_FIXED)
static foc_scalar_t angle_sin_fixed(foc_scalar_t qTurns)
{
    int32_t nPhase = angle_wrap_scalar(qTurns);
    bool bNegative = nPhase >= (FOC_Q_SCALE / 2);
    int32_t nHalfPhase;
    int32_t nProduct;
    int32_t nValue;
    int32_t nCorrection;

    if (bNegative) {
        nPhase -= FOC_Q_SCALE / 2;
    }
    nHalfPhase = nPhase << 1;
    nProduct = nHalfPhase * (FOC_Q_SCALE - nHalfPhase);
    nValue = nProduct >> 13;
    nCorrection = (nValue * (FOC_Q_SCALE - nValue)) >>
                  FOC_Q_FRACTION_BITS;
    nValue -= (nCorrection * 7373) >> FOC_Q_FRACTION_BITS;
    nValue = foc_sat(nValue, FOC_ZERO, FOC_ONE);

    return bNegative ? -nValue : nValue;
}
#endif

foc_scalar_t foc_angle_sin(foc_angle_t tAngle)
{
#if defined(FOC_NUMERIC_FLOAT)
    return sinf(angle_wrap_scalar(tAngle.qTurns) * FOC_TWO_PI_F);
#else
    return angle_sin_fixed(tAngle.qTurns);
#endif
}

foc_scalar_t foc_angle_cos(foc_angle_t tAngle)
{
    tAngle.qTurns = foc_add_sat(tAngle.qTurns, FOC_SCALAR(0.25f));
    return foc_angle_sin(tAngle);
}

foc_angle_t foc_angle_atan2(foc_scalar_t qY,
                            foc_scalar_t qX)
{
#if defined(FOC_NUMERIC_FLOAT)
    float fTurns = atan2f(qY, qX) / FOC_TWO_PI_F;
    return foc_angle_from_turns(fTurns);
#else
    static const int16_t s_hwAtanTurnsQ16[] = {
        8192, 4836, 2555, 1297, 651, 326, 163,
        81, 41, 20, 10, 5, 3, 1,
    };
    int32_t nX = qX;
    int32_t nY = qY;
    int32_t nAngleQ16 = 0;
    uint8_t chIndex;

    if (nX == 0 && nY == 0) {
        return foc_angle_from_scalar(FOC_ZERO);
    }
    if (nX < 0) {
        bool bUpperHalf = nY >= 0;
        nX = -nX;
        nY = -nY;
        nAngleQ16 = bUpperHalf ? 32768 : -32768;
    }
    for (chIndex = 0; chIndex < 14; chIndex++) {
        int32_t nNextX;
        int32_t nNextY;
        if (nY > 0) {
            nNextX = nX + (nY >> chIndex);
            nNextY = nY - (nX >> chIndex);
            nAngleQ16 += s_hwAtanTurnsQ16[chIndex];
        } else {
            nNextX = nX - (nY >> chIndex);
            nNextY = nY + (nX >> chIndex);
            nAngleQ16 -= s_hwAtanTurnsQ16[chIndex];
        }
        nX = nNextX;
        nY = nNextY;
    }
    return foc_angle_from_scalar((foc_scalar_t)(nAngleQ16 / 2));
#endif
}
