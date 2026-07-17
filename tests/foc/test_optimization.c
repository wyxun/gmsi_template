#include "test_common.h"

#include "foc_cogging.h"
#include "foc_optimization.h"

static foc_gain_t optimization_gain(float fValue, int *pnFailures)
{
    foc_gain_t tGain = {0};
    if (foc_gain_from_float(fValue, &tGain) != FOC_RESULT_OK) {
        printf("FAIL %s:%d: invalid optimization gain %g\n",
               __FILE__, __LINE__, (double)fValue);
        (*pnFailures)++;
    }
    return tGain;
}

static int test_mtpa_and_field_weakening(void)
{
    int nFailures = 0;
    foc_scalar_t qId = FOC_ZERO;
    foc_field_weakening_params_t tParams = {
        .tVoltagePid = {
            .tKp = optimization_gain(0.5f, &nFailures),
            .tKiTs = optimization_gain(0.1f, &nFailures),
            .tKdOverTs = optimization_gain(0.0f, &nFailures),
            .qOutputMinimum = FOC_SCALAR(-0.8f),
            .qOutputMaximum = FOC_ZERO,
            .qIntegratorMinimum = FOC_SCALAR(-0.8f),
            .qIntegratorMaximum = FOC_ZERO,
        },
        .qBaseSpeed = FOC_HALF,
        .qVoltageLimit = FOC_SCALAR(0.8f),
        .qMinimumId = FOC_SCALAR(-0.8f),
    };
    foc_field_weakening_t tWeakening;
    foc_dq_t tVoltage = {
        .qD = FOC_SCALAR(0.8f),
        .qQ = FOC_SCALAR(0.6f),
    };

    TEST_CHECK(foc_mtpa_Calculate(FOC_SCALAR(0.5f),
                                  FOC_SCALAR(0.2f),
                                  FOC_SCALAR(0.3f),
                                  FOC_SCALAR(0.4f), &qId) ==
               FOC_RESULT_OK);
    TEST_CHECK(qId < FOC_ZERO);
    TEST_CHECK(foc_mtpa_Calculate(FOC_ONE, FOC_ZERO, FOC_ONE,
                                  FOC_ONE, &qId) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(qId), -0.618034f, 0.003f);
    TEST_CHECK(foc_mtpa_Calculate(FOC_SCALAR(0.5f),
                                  FOC_SCALAR(0.2f),
                                  FOC_SCALAR(0.2f),
                                  FOC_SCALAR(0.4f), &qId) ==
               FOC_RESULT_OK);
    TEST_CHECK(qId == FOC_ZERO);
    TEST_CHECK(foc_field_weakening_Init(&tWeakening, &tParams) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_field_weakening_Step(&tWeakening, FOC_SCALAR(0.4f),
                                        &tVoltage) == FOC_ZERO);
    qId = foc_field_weakening_Step(&tWeakening, FOC_SCALAR(0.7f),
                                   &tVoltage);
    TEST_CHECK(qId < FOC_ZERO && qId >= tParams.qMinimumId);
    return nFailures;
}

static int test_compensation(void)
{
    int nFailures = 0;
    foc_deadtime_params_t tDeadtime = {
        .qCompensation = FOC_SCALAR(0.03f),
        .qCurrentThreshold = FOC_SCALAR(0.05f),
    };
    foc_duty_abc_t tDuty = {
        .qU = FOC_SCALAR(0.99f),
        .qV = FOC_HALF,
        .qW = FOC_SCALAR(0.01f),
    };
    foc_abc_t tCurrent = {
        .qA = FOC_SCALAR(0.2f),
        .qB = FOC_ZERO,
        .qC = FOC_SCALAR(-0.2f),
    };
    foc_angle_t tAngle = foc_angle_from_turns(0.95f);
    foc_angle_t tCompensated;
    foc_gain_t tDelay = optimization_gain(0.2f, &nFailures);

    TEST_CHECK(foc_deadtime_Compensate(&tDeadtime, &tCurrent, &tDuty) ==
               FOC_RESULT_OK);
    TEST_CHECK(tDuty.qU == FOC_ONE);
    TEST_CHECK(tDuty.qV == FOC_HALF);
    TEST_CHECK(tDuty.qW == FOC_ZERO);
    TEST_CHECK(foc_phase_delay_Compensate(
                   tAngle, FOC_HALF, &tDelay, FOC_SCALAR(0.02f),
                   &tCompensated) == FOC_RESULT_OK);
    TEST_NEAR(foc_angle_to_turns(tCompensated), 0.07f, 0.002f);
    return nFailures;
}

static int test_cogging(void)
{
    int nFailures = 0;
    const foc_scalar_t aqTable[] = {
        FOC_ZERO, FOC_SCALAR(0.2f), FOC_ZERO, FOC_SCALAR(-0.2f),
    };
    foc_cogging_t tCogging;
    foc_scalar_t qCompensation = FOC_ZERO;

    TEST_CHECK(foc_cogging_Init(&tCogging, aqTable, 4U) == FOC_RESULT_OK);
    TEST_CHECK(foc_cogging_Get(&tCogging, foc_angle_from_turns(0.125f),
                               &qCompensation) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(qCompensation), 0.1f, 0.002f);
    TEST_CHECK(foc_cogging_Get(&tCogging, foc_angle_from_turns(0.875f),
                               &qCompensation) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(qCompensation), -0.1f, 0.002f);
    return nFailures;
}

int test_optimization(void)
{
    return test_mtpa_and_field_weakening() + test_compensation() +
           test_cogging();
}
