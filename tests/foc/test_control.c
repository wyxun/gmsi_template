#include "test_common.h"
#include "foc_filter.h"
#include "foc_feedforward.h"
#include "foc_ltd.h"
#include "foc_pid.h"
#include "foc_pll.h"

static int test_pid(void)
{
    int nFailures = 0;
#if defined(FOC_NUMERIC_FLOAT)
    const float fTolerance = 0.0001f;
#else
    const float fTolerance = 0.001f;
#endif
    foc_pid_params_t tParams;
    foc_pid_t tPidA;
    foc_pid_t tPidB;
    foc_scalar_t qOutput;

    TEST_CHECK(foc_gain_from_float(0.5f, &tParams.tKp) == FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.1f, &tParams.tKiTs) == FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.0f, &tParams.tKdOverTs) ==
               FOC_RESULT_OK);
    tParams.qOutputMinimum = FOC_NEG_ONE;
    tParams.qOutputMaximum = FOC_ONE;
    tParams.qIntegratorMinimum = FOC_SCALAR(-0.5f);
    tParams.qIntegratorMaximum = FOC_SCALAR(0.5f);

    TEST_CHECK(foc_pid_Init(&tPidA, &tParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_pid_Init(&tPidB, &tParams) == FOC_RESULT_OK);
    foc_pid_Track(&tPidA, FOC_SCALAR(0.20f),
                  FOC_SCALAR(0.50f), FOC_ZERO);
    qOutput = foc_pid_Step(&tPidA, FOC_SCALAR(0.50f), FOC_ZERO);
    TEST_NEAR(foc_to_float(qOutput), 0.20f, fTolerance);
    foc_pid_Reset(&tPidA);
    qOutput = foc_pid_Step(&tPidA, FOC_SCALAR(0.5f), FOC_ZERO);
    TEST_NEAR(foc_to_float(qOutput), 0.30f, fTolerance);
    TEST_NEAR(foc_to_float(tPidA.qIntegrator), 0.05f, fTolerance);
    TEST_NEAR(foc_to_float(tPidB.qIntegrator), 0.0f, fTolerance);

    TEST_CHECK(foc_gain_from_float(2.0f, &tParams.tKp) == FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.5f, &tParams.tKiTs) == FOC_RESULT_OK);
    TEST_CHECK(foc_pid_Init(&tPidA, &tParams) == FOC_RESULT_OK);
    qOutput = foc_pid_Step(&tPidA, FOC_ONE, FOC_ZERO);
    TEST_NEAR(foc_to_float(qOutput), 1.0f, fTolerance);
    TEST_NEAR(foc_to_float(tPidA.qIntegrator), 0.0f, fTolerance);
    foc_pid_Reset(&tPidA);
    TEST_CHECK(tPidA.qIntegrator == FOC_ZERO);

    return nFailures;
}

static int test_filters(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    unsigned int wType;
    foc_lpf1_t tFilterA;
    foc_lpf1_t tFilterB;
    foc_biquad_coeffs_t tCoefficients;
    foc_biquad_t tBiquad;
    foc_scalar_t qOutput = FOC_ZERO;

    TEST_CHECK(foc_lpf1_Init(&tFilterA, FOC_SCALAR(0.25f), FOC_ZERO) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_lpf1_Init(&tFilterB, FOC_SCALAR(0.25f), FOC_ZERO) ==
               FOC_RESULT_OK);
    for (wIndex = 0; wIndex < 20; wIndex++) {
        qOutput = foc_lpf1_Step(&tFilterA, FOC_ONE);
    }
    TEST_CHECK(qOutput > FOC_SCALAR(0.99f));
    TEST_CHECK(tFilterB.qState == FOC_ZERO);

    TEST_CHECK(foc_gain_from_float(0.25f, &tCoefficients.tB0) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.50f, &tCoefficients.tB1) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.25f, &tCoefficients.tB2) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.0f, &tCoefficients.tA1) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.0f, &tCoefficients.tA2) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_biquad_Init(&tBiquad, &tCoefficients) == FOC_RESULT_OK);
    qOutput = foc_biquad_Step(&tBiquad, FOC_ONE);
    TEST_CHECK(qOutput >= FOC_ZERO && qOutput <= FOC_ONE);
    qOutput = foc_biquad_Step(&tBiquad, FOC_ONE);
    TEST_CHECK(qOutput >= FOC_ZERO && qOutput <= FOC_ONE);
    foc_biquad_Reset(&tBiquad);
    TEST_CHECK(tBiquad.qState1 == FOC_ZERO && tBiquad.qState2 == FOC_ZERO);

    for (wType = FOC_FILTER_BUTTERWORTH;
         wType <= FOC_FILTER_BESSEL; wType++) {
        foc_scalar_t qPeak = FOC_ZERO;
        foc_scalar_t qPeakLimit;

        if (wType == FOC_FILTER_CHEBYSHEV_I_0P1_DB) {
            /* This prototype has Q ~= 1.354 and intentionally overshoots. */
            qPeakLimit = FOC_SCALAR(1.35f);
        } else {
            qPeakLimit = FOC_SCALAR(1.10f);
        }
        TEST_CHECK(foc_biquad_LowPassInit(
                       &tBiquad, (foc_filter_response_t)wType,
                       10000U, 500U) == FOC_RESULT_OK);
        for (wIndex = 0; wIndex < 500U; wIndex++) {
            qOutput = foc_biquad_Step(&tBiquad, FOC_ONE);
            if (foc_abs(qOutput) > qPeak) qPeak = foc_abs(qOutput);
        }
        if (qPeak >= qPeakLimit) {
            printf("filter type %u peak=%g\n", wType,
                   (double)foc_to_float(qPeak));
            nFailures++;
        }
        TEST_NEAR(foc_to_float(qOutput), 1.0f, 0.03f);
    }

    return nFailures;
}

static int test_pll(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    float fMeasuredTurns = 0.0f;
    foc_pll_params_t tParams;
    foc_pll_t tPll;

    TEST_CHECK(foc_gain_from_float(0.25f, &tParams.tKp) == FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.02f, &tParams.tKi) == FOC_RESULT_OK);
    tParams.qMaximumSpeed = FOC_SCALAR(0.02f);
    tParams.qLockError = FOC_SCALAR(0.02f);
    tParams.qUnlockError = FOC_SCALAR(0.08f);
    tParams.hwLockSamples = 20U;
    TEST_CHECK(foc_pll_Init(&tPll, &tParams) == FOC_RESULT_OK);

    for (wIndex = 0; wIndex < 1000U; wIndex++) {
        fMeasuredTurns += 0.001f;
        TEST_CHECK(foc_pll_Step(&tPll,
                                foc_angle_from_turns(fMeasuredTurns)) ==
                   FOC_RESULT_OK);
    }
    TEST_CHECK(tPll.bIsLocked);
    TEST_NEAR(foc_to_float(tPll.qSpeed), 0.001f, 0.0015f);
    TEST_NEAR(foc_to_float(foc_angle_diff(
                  foc_angle_from_turns(fMeasuredTurns), tPll.tAngle)),
              0.0f, 0.02f);
    foc_pll_Reset(&tPll, foc_angle_from_turns(0.0f));
    TEST_CHECK(!tPll.bIsLocked && tPll.qSpeed == FOC_ZERO);

    return nFailures;
}

static int test_ltd_and_feedforward(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    foc_ltd_params_t tLtdParams = {
        .qMaximumVelocity = FOC_SCALAR(0.05f),
        .qMaximumAcceleration = FOC_SCALAR(0.005f),
    };
    foc_ltd_t tLtd;
    foc_feedforward_params_t tFeedforwardParams;
    foc_feedforward_t tFeedforward;
    foc_dq_t tVoltage;

    TEST_CHECK(foc_ltd_Init(&tLtd, &tLtdParams, FOC_ZERO) == FOC_RESULT_OK);
    (void)foc_ltd_Step(&tLtd, FOC_ONE);
    TEST_CHECK(tLtd.qPosition > FOC_ZERO &&
               tLtd.qVelocity > FOC_ZERO);
    for (wIndex = 0; wIndex < 100U; wIndex++) {
        (void)foc_ltd_Step(&tLtd, FOC_ONE);
    }
    TEST_NEAR(foc_to_float(tLtd.qPosition), 1.0f, 0.01f);

    TEST_CHECK(foc_gain_from_float(0.2f, &tFeedforwardParams.tLq) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.1f, &tFeedforwardParams.tLd) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_gain_from_float(0.3f, &tFeedforwardParams.tFlux) ==
               FOC_RESULT_OK);
    tFeedforwardParams.qOutputLimit = FOC_ONE;
    TEST_CHECK(foc_feedforward_Init(&tFeedforward,
                                    &tFeedforwardParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_feedforward_Pmsm(&tFeedforward,
                                    FOC_SCALAR(0.5f),
                                    FOC_SCALAR(0.2f),
                                    FOC_SCALAR(0.4f),
                                    &tVoltage) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(tVoltage.qD), -0.04f, 0.002f);
    TEST_NEAR(foc_to_float(tVoltage.qQ), 0.16f, 0.002f);

    return nFailures;
}

int test_control(void)
{
    return test_pid() + test_filters() + test_pll() +
           test_ltd_and_feedforward();
}
