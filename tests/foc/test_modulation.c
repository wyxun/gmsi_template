#include "test_common.h"
#include "foc_modulation.h"

static void check_duty_range(foc_duty_abc_t tDuty, int *pnFailures)
{
    int nFailures = 0;

    TEST_CHECK(tDuty.qU >= FOC_ZERO && tDuty.qU <= FOC_ONE);
    TEST_CHECK(tDuty.qV >= FOC_ZERO && tDuty.qV <= FOC_ONE);
    TEST_CHECK(tDuty.qW >= FOC_ZERO && tDuty.qW <= FOC_ONE);
    *pnFailures += nFailures;
}

int test_modulation(void)
{
    int nFailures = 0;
#if defined(FOC_NUMERIC_FLOAT)
    const float fTolerance = 0.0001f;
#else
    const float fTolerance = 0.0002f;
#endif
    foc_ab_t tZero = { FOC_ZERO, FOC_ZERO };
    foc_ab_t tAlpha = { FOC_SCALAR(0.6f), FOC_ZERO };
    foc_ab_t tSmallA = { FOC_SCALAR(0.1f), FOC_SCALAR(0.1f) };
    foc_ab_t tSmallB = { FOC_SCALAR(0.101f), FOC_SCALAR(0.1f) };
    foc_duty_abc_t tDuty;
    foc_duty_abc_t tDutyA;
    foc_duty_abc_t tDutyB;

    TEST_CHECK(foc_svpwm(&tZero, &tDuty) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(tDuty.qU), 0.5f, fTolerance);
    TEST_NEAR(foc_to_float(tDuty.qV), 0.5f, fTolerance);
    TEST_NEAR(foc_to_float(tDuty.qW), 0.5f, fTolerance);

    TEST_CHECK(foc_svpwm(&tAlpha, &tDuty) == FOC_RESULT_OK);
    check_duty_range(tDuty, &nFailures);
    TEST_CHECK(tDuty.qU > tDuty.qV && tDuty.qV == tDuty.qW);
    TEST_CHECK(foc_spwm(&tAlpha, &tDutyA) == FOC_RESULT_OK);
    TEST_CHECK(tDutyA.qU == FOC_ONE);
    TEST_CHECK(tDuty.qU < FOC_ONE);

    TEST_CHECK(foc_third_harmonic_spwm(&tAlpha, &tDutyA) ==
               FOC_RESULT_OK);
    check_duty_range(tDutyA, &nFailures);

    TEST_CHECK(foc_svpwm(&tSmallA, &tDutyA) == FOC_RESULT_OK);
    TEST_CHECK(foc_svpwm(&tSmallB, &tDutyB) == FOC_RESULT_OK);
    TEST_CHECK(foc_abs(foc_sub_sat(tDutyB.qU, tDutyA.qU)) <
               FOC_SCALAR(0.003f));
    TEST_CHECK(foc_svpwm(NULL, &tDuty) == FOC_RESULT_NULL);

    return nFailures;
}
