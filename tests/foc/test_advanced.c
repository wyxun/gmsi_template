#include "test_common.h"
#include "foc_dob.h"
#include "foc_hfi.h"
#include "foc_ladrc.h"
#include "foc_smc.h"
#include "foc_sta.h"

static foc_gain_t test_gain(float fValue, int *pnFailures)
{
    foc_gain_t tGain = {0};
    if (foc_gain_from_float(fValue, &tGain) != FOC_RESULT_OK) {
        printf("FAIL %s:%d: invalid test gain %g\n", __FILE__, __LINE__,
               (double)fValue);
        (*pnFailures)++;
    }
    return tGain;
}

static int test_advanced_controllers(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    foc_ladrc_params_t tLadrcParams = {
        .tTrackingPosition = test_gain(0.05f, &nFailures),
        .tTrackingVelocity = test_gain(0.20f, &nFailures),
        .tTrackingIntegrator = test_gain(0.05f, &nFailures),
        .tObserverBeta1 = test_gain(0.30f, &nFailures),
        .tObserverBeta2 = test_gain(0.10f, &nFailures),
        .tObserverBeta3 = test_gain(0.02f, &nFailures),
        .tPlantGain = test_gain(0.10f, &nFailures),
        .tKp = test_gain(0.80f, &nFailures),
        .tKd = test_gain(0.10f, &nFailures),
        .tPlantInverse = test_gain(1.0f, &nFailures),
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
    };
    foc_smc_params_t tSmcParams = {
        .tDerivative = test_gain(0.5f, &nFailures),
        .tSurface = test_gain(0.5f, &nFailures),
        .tDiscontinuous = test_gain(0.1f, &nFailures),
        .tReach = test_gain(0.2f, &nFailures),
        .tPlant = test_gain(0.1f, &nFailures),
        .tIntegrator = test_gain(0.05f, &nFailures),
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
    };
    foc_sta_params_t tStaParams = {
        .tK1 = test_gain(0.4f, &nFailures),
        .tK2Ts = test_gain(0.05f, &nFailures),
        .tBoundaryInverse = test_gain(10.0f, &nFailures),
        .qIntegratorMinimum = FOC_SCALAR(-0.5f),
        .qIntegratorMaximum = FOC_SCALAR(0.5f),
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
    };
    foc_ladrc_t tLadrc;
    foc_smc_t tSmc;
    foc_sta_t tSta;
    foc_controller_if_t tController;
    foc_scalar_t qOutput = FOC_ZERO;

    TEST_CHECK(foc_ladrc_Init(&tLadrc, &tLadrcParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_smc_Init(&tSmc, &tSmcParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_sta_Init(&tSta, &tStaParams) == FOC_RESULT_OK);
    tController = foc_ladrc_ControllerInterface(&tLadrc);
    TEST_CHECK(foc_controller_IsValid(&tController));
    TEST_CHECK(foc_controller_CanTrack(&tController));
    foc_controller_Track(&tController, FOC_SCALAR(2.0f),
                         FOC_SCALAR(0.4f), FOC_SCALAR(0.1f));
    TEST_NEAR(foc_to_float(tLadrc.qTrackingPosition), 0.4f, 0.002f);
    TEST_NEAR(foc_to_float(tLadrc.qObserverPosition), 0.1f, 0.002f);
    TEST_NEAR(foc_to_float(tLadrc.qOutput), 1.0f, 0.002f);
    for (wIndex = 0U; wIndex < 100U; wIndex++) {
        qOutput = foc_controller_Step(&tController, FOC_HALF, FOC_ZERO);
    }
    TEST_CHECK(qOutput > FOC_ZERO && qOutput <= FOC_ONE);
    qOutput = foc_smc_Step(&tSmc, FOC_HALF, FOC_ZERO);
    TEST_CHECK(qOutput > FOC_ZERO && qOutput <= FOC_ONE);
    qOutput = foc_sta_Step(&tSta, FOC_HALF, FOC_ZERO);
    TEST_CHECK(qOutput > FOC_ZERO && qOutput <= FOC_ONE);
    return nFailures;
}

static int test_hfi_and_dob(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    foc_hfi_params_t tHfiParams = {
        .qPhaseStep = FOC_SCALAR(0.125f),
        .qInjectionAmplitude = FOC_SCALAR(0.1f),
        .qHighPassAlpha = FOC_SCALAR(0.8f),
        .qDemodAlpha = FOC_SCALAR(0.2f),
        .tDemodGain = test_gain(2.0f, &nFailures),
        .qMinimumResponse = FOC_SCALAR(0.01f),
    };
    foc_dob_params_t tDobParams = {
        .tTorqueGain = test_gain(0.5f, &nFailures),
        .tModelGain = test_gain(0.1f, &nFailures),
        .tK1 = test_gain(0.2f, &nFailures),
        .tK2Ts = test_gain(0.02f, &nFailures),
        .tBoundaryInverse = test_gain(10.0f, &nFailures),
        .qDisturbanceMinimum = FOC_SCALAR(-0.5f),
        .qDisturbanceMaximum = FOC_SCALAR(0.5f),
        .qSpeedMinimum = FOC_NEG_ONE,
        .qSpeedMaximum = FOC_ONE,
    };
    foc_hfi_t tHfi;
    foc_hfi_output_t tHfiOutput;
    foc_dob_t tDob;
    foc_dob_output_t tDobOutput;

    TEST_CHECK(foc_hfi_Init(&tHfi, &tHfiParams) == FOC_RESULT_OK);
    tHfiParams.tDemodGain.qFraction = FOC_SCALAR(1.1f);
    TEST_CHECK(foc_hfi_Init(&tHfi, &tHfiParams) ==
               FOC_RESULT_INVALID_ARGUMENT);
    tHfiParams.tDemodGain = test_gain(2.0f, &nFailures);
    TEST_CHECK(foc_hfi_Init(&tHfi, &tHfiParams) == FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 32U; wIndex++) {
        TEST_CHECK(foc_hfi_Step(&tHfi,
                                (wIndex & 1U) ? FOC_SCALAR(0.1f) :
                                               FOC_SCALAR(-0.1f),
                                &tHfiOutput) == FOC_RESULT_OK);
        TEST_CHECK(foc_abs(tHfiOutput.qInjectionD) <=
                   tHfiParams.qInjectionAmplitude);
    }
    tHfiParams.qPhaseStep = FOC_SCALAR(0.25f);
    TEST_CHECK(foc_hfi_Init(&tHfi, &tHfiParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_hfi_Step(&tHfi, FOC_ZERO, &tHfiOutput) ==
               FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 16U; wIndex++) {
        foc_scalar_t qPreviousInjection = tHfiOutput.qInjectionD;
        TEST_CHECK(foc_hfi_Step(&tHfi, qPreviousInjection,
                                &tHfiOutput) == FOC_RESULT_OK);
    }
    TEST_CHECK(tHfiOutput.bValid);
    TEST_CHECK(tHfiOutput.qResponse >= tHfiParams.qMinimumResponse);
    TEST_CHECK(foc_dob_Init(&tDob, &tDobParams) == FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 100U; wIndex++) {
        TEST_CHECK(foc_dob_Step(&tDob, FOC_SCALAR(0.2f),
                                FOC_SCALAR(0.05f), &tDobOutput) ==
                   FOC_RESULT_OK);
    }
    TEST_CHECK(tDobOutput.qEstimatedSpeed >= FOC_NEG_ONE &&
               tDobOutput.qEstimatedSpeed <= FOC_ONE);
    TEST_CHECK(tDobOutput.qDisturbance >= FOC_SCALAR(-0.5f) &&
               tDobOutput.qDisturbance <= FOC_SCALAR(0.5f));
    return nFailures;
}

int test_advanced(void)
{
    return test_advanced_controllers() + test_hfi_and_dob();
}
