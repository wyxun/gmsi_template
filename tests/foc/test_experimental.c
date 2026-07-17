#include "test_common.h"

#include "foc_identify.h"
#include "foc_nsd.h"

#if FOC_ENABLE_EXPERIMENTAL_NSD && FOC_ENABLE_EXPERIMENTAL_IDENTIFY
static void test_emergency_stop(void *pContext)
{
    unsigned int *pwStops = (unsigned int *)pContext;
    (*pwStops)++;
}

static foc_experiment_guard_t test_safe_guard(void)
{
    foc_experiment_guard_t tGuard = {
        .bMotorStopped = true,
        .bFault = false,
        .qCurrentMagnitude = FOC_SCALAR(0.1f),
        .qElectricalSpeed = FOC_ZERO,
        .qBusVoltage = FOC_HALF,
    };
    return tGuard;
}

static foc_experiment_safety_t test_safety(unsigned int *pwStops)
{
    foc_experiment_safety_t tSafety = {
        .qMaximumCurrent = FOC_SCALAR(0.8f),
        .qMaximumSpeed = FOC_SCALAR(0.1f),
        .qMinimumBusVoltage = FOC_SCALAR(0.2f),
        .qMaximumBusVoltage = FOC_ONE,
        .wTimeoutSamples = 40U,
        .pContext = pwStops,
        .fnEmergencyStop = test_emergency_stop,
    };
    return tSafety;
}

static int test_nsd_safety_and_sequence(void)
{
    int nFailures = 0;
    unsigned int wStops = 0U;
    unsigned int wIndex;
    foc_nsd_params_t tParams = {
        .tSafety = test_safety(&wStops),
        .qTestVoltage = FOC_SCALAR(0.1f),
        .wSettleSamples = 2U,
        .wPositiveSamples = 1U,
        .wZeroSamples = 1U,
        .wNegativeSamples = 2U,
    };
    foc_experiment_guard_t tGuard = test_safe_guard();
    foc_nsd_t tNsd;
    foc_nsd_output_t tOutput;

    TEST_CHECK(foc_nsd_Init(&tNsd, &tParams) == FOC_RESULT_OK);
    tParams.tSafety.fnEmergencyStop = NULL;
    TEST_CHECK(foc_nsd_Init(&tNsd, &tParams) ==
               FOC_RESULT_INVALID_ARGUMENT);
    tParams.tSafety.fnEmergencyStop = test_emergency_stop;
    TEST_CHECK(foc_nsd_Init(&tNsd, &tParams) == FOC_RESULT_OK);
    tGuard.bMotorStopped = false;
    TEST_CHECK(foc_nsd_Start(&tNsd, &tGuard) ==
               FOC_RESULT_INVALID_ARGUMENT);
    tGuard.bMotorStopped = true;
    TEST_CHECK(foc_nsd_Start(&tNsd, &tGuard) == FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 12U && !tNsd.bComplete; wIndex++) {
        foc_scalar_t qResponse =
            tNsd.eState == FOC_NSD_NEGATIVE ? FOC_SCALAR(0.4f) :
                                              FOC_SCALAR(0.2f);
        TEST_CHECK(foc_nsd_Step(&tNsd, &tGuard, qResponse, &tOutput) ==
                   FOC_RESULT_OK);
    }
    TEST_CHECK(tNsd.bComplete);
    TEST_CHECK(tOutput.qVoltageD == FOC_ZERO);
    TEST_CHECK(tOutput.bReversePolarity);

    TEST_CHECK(foc_nsd_Start(&tNsd, &tGuard) == FOC_RESULT_OK);
    tGuard.bFault = true;
    TEST_CHECK(foc_nsd_Step(&tNsd, &tGuard, FOC_ZERO, &tOutput) ==
               FOC_RESULT_SAFETY);
    TEST_CHECK(tOutput.qVoltageD == FOC_ZERO);
    TEST_CHECK(wStops == 1U);
    return nFailures;
}

static int test_identification(void)
{
    int nFailures = 0;
    unsigned int wStops = 0U;
    unsigned int wIndex;
    foc_identify_params_t tParams = {
        .tSafety = test_safety(&wStops),
        .eMode = FOC_IDENTIFY_RS_LD_LQ,
        .qHalfVoltage = FOC_SCALAR(0.1f),
        .qFullVoltage = FOC_SCALAR(0.2f),
        .qFluxVoltage = FOC_SCALAR(0.2f),
        .qCurrentRiseRatio = FOC_SCALAR(0.9f),
        .qResetCurrentThreshold = FOC_SCALAR(0.02f),
        .qInductanceTimeStep = FOC_SCALAR(0.01f),
        .qMinimumFluxSpeed = FOC_SCALAR(0.1f),
        .wSettleSamples = 2U,
        .wResetSamples = 1U,
        .wMaximumRiseSamples = 10U,
    };
    foc_experiment_guard_t tGuard = test_safe_guard();
    foc_identify_t tIdentify;
    foc_identify_input_t tInput = {0};
    foc_identify_output_t tOutput;

    TEST_CHECK(foc_identify_Init(&tIdentify, &tParams) == FOC_RESULT_OK);
    tParams.eMode = (foc_identify_mode_t)-1;
    TEST_CHECK(foc_identify_Init(&tIdentify, &tParams) ==
               FOC_RESULT_INVALID_ARGUMENT);
    tParams.eMode = FOC_IDENTIFY_FLUX;
    tParams.qFluxVoltage = FOC_SCALAR(1.1f);
    TEST_CHECK(foc_identify_Init(&tIdentify, &tParams) ==
               FOC_RESULT_INVALID_ARGUMENT);
    tParams.eMode = FOC_IDENTIFY_RS_LD_LQ;
    tParams.qFluxVoltage = FOC_SCALAR(0.2f);
    TEST_CHECK(foc_identify_Init(&tIdentify, &tParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_identify_Start(&tIdentify, &tGuard) == FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 30U && !tIdentify.bComplete; wIndex++) {
        if (tIdentify.eState == FOC_IDENTIFY_RS_HALF) {
            tInput.qCurrentD = FOC_SCALAR(0.1f);
            tInput.qCurrentQ = FOC_ZERO;
        } else if (tIdentify.eState == FOC_IDENTIFY_LD_ZERO ||
                   tIdentify.eState == FOC_IDENTIFY_LQ_ZERO) {
            tInput.qCurrentD = FOC_ZERO;
            tInput.qCurrentQ = FOC_ZERO;
        } else {
            tInput.qCurrentD = FOC_SCALAR(0.2f);
            tInput.qCurrentQ = FOC_SCALAR(0.2f);
        }
        TEST_CHECK(foc_identify_Step(&tIdentify, &tGuard, &tInput,
                                     &tOutput) == FOC_RESULT_OK);
    }
    TEST_CHECK(tIdentify.bComplete);
    TEST_NEAR(foc_to_float(tOutput.qResistance), 1.0f, 0.03f);
    TEST_CHECK(tOutput.qInductanceD > FOC_ZERO);
    TEST_CHECK(tOutput.qInductanceQ > FOC_ZERO);
    TEST_CHECK(tOutput.tVoltage.qD == FOC_ZERO &&
               tOutput.tVoltage.qQ == FOC_ZERO);
    return nFailures;
}

int test_experimental(void)
{
    return test_nsd_safety_and_sequence() + test_identification();
}
#else
int test_experimental(void)
{
    int nFailures = 0;
    foc_nsd_output_t tNsdOutput = {
        .qVoltageD = FOC_HALF,
    };
    foc_identify_output_t tIdentifyOutput = {
        .tVoltage = { .qD = FOC_HALF, .qQ = FOC_HALF },
    };

    TEST_CHECK(foc_nsd_Step(NULL, NULL, FOC_ZERO, &tNsdOutput) ==
               FOC_RESULT_DISABLED);
    TEST_CHECK(tNsdOutput.qVoltageD == FOC_ZERO);
    TEST_CHECK(foc_identify_Step(NULL, NULL, NULL, &tIdentifyOutput) ==
               FOC_RESULT_DISABLED);
    TEST_CHECK(tIdentifyOutput.tVoltage.qD == FOC_ZERO &&
               tIdentifyOutput.tVoltage.qQ == FOC_ZERO);
    return nFailures;
}
#endif
