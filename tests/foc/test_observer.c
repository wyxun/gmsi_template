#include "test_common.h"
#include "foc_hall.h"
#include "foc_nlfo.h"
#include "foc_observer_selector.h"
#include "foc_smo.h"

typedef struct {
    foc_position_output_t tOutput;
    unsigned int wSteps;
} fake_observer_t;

static void fake_observer_reset(void *pContext)
{
    ((fake_observer_t *)pContext)->wSteps = 0U;
}

static foc_result_t fake_observer_step(
    void *pContext,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput)
{
    fake_observer_t *ptFake = (fake_observer_t *)pContext;
    (void)ptInput;
    ptFake->wSteps++;
    *ptOutput = ptFake->tOutput;
    return FOC_RESULT_OK;
}

static int test_hall(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    foc_hall_params_t tParams;
    foc_hall_t tHall;
    foc_position_output_t tOutput;

    foc_hall_DefaultParams(&tParams);
    tParams.qSpeedFilterAlpha = FOC_ONE;
    TEST_CHECK(foc_hall_Init(&tHall, &tParams) == FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 6U; wIndex++) {
        TEST_CHECK(foc_hall_Step(&tHall, 5U, &tOutput) == FOC_RESULT_OK);
    }
    TEST_CHECK(foc_hall_Step(&tHall, 4U, &tOutput) == FOC_RESULT_OK);
    TEST_CHECK((tOutput.eValidFlags &
                FOC_POSITION_VALID_ELECTRICAL_ANGLE) != 0U);
    TEST_NEAR(foc_to_float(tOutput.qElectricalSpeed),
              1.0f / 36.0f, 0.002f);
    TEST_CHECK(tOutput.tElectricalAngle.wBam32 < 0xFFFFFFFFU);
    TEST_CHECK(foc_hall_Step(&tHall, 0U, &tOutput) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    return nFailures;
}

static int test_observer_initialization(void)
{
    int nFailures = 0;
    foc_smo_params_t tSmoParams = {
        .qModelGain = FOC_SCALAR(0.1f),
        .qResistance = FOC_SCALAR(0.1f),
        .qSlidingGain = FOC_SCALAR(0.5f),
        .qBoundaryInverse = FOC_SCALAR(4.0f),
        .qEmfFilterAlpha = FOC_SCALAR(0.1f),
        .qMinimumBemf = FOC_SCALAR(0.05f),
    };
    foc_nlfo_params_t tNlfoParams = {
        .qIntegratorGain = FOC_SCALAR(0.01f),
        .qResistance = FOC_SCALAR(0.1f),
        .qAverageInductance = FOC_SCALAR(0.2f),
        .qFlux = FOC_SCALAR(0.3f),
        .qCorrectionGain = FOC_SCALAR(0.2f),
        .qMinimumFluxRatio = FOC_SCALAR(0.5f),
    };
    foc_smo_t tSmo;
    foc_nlfo_t tNlfo;
    foc_position_input_t tInput = {0};
    foc_position_output_t tOutput;

    TEST_CHECK(foc_smo_Init(&tSmo, &tSmoParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_smo_Step(&tSmo, &tInput, &tOutput) == FOC_RESULT_OK);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    TEST_CHECK(tOutput.tElectricalAngle.wBam32 < 0xFFFFFFFFU);
    TEST_CHECK(foc_nlfo_Init(&tNlfo, &tNlfoParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_nlfo_Step(&tNlfo, &tInput, &tOutput) == FOC_RESULT_OK);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    tNlfoParams.qFlux = FOC_ZERO;
    TEST_CHECK(foc_nlfo_Init(&tNlfo, &tNlfoParams) ==
               FOC_RESULT_INVALID_ARGUMENT);
    return nFailures;
}

static int test_selector(void)
{
    int nFailures = 0;
    unsigned int wIndex;
    fake_observer_t tPrimary = {
        .tOutput = {
            .tElectricalAngle = { 429496729U },
            .qElectricalSpeed = FOC_SCALAR(0.10f),
            .qConfidence = FOC_ONE,
            .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                           FOC_POSITION_VALID_ELECTRICAL_SPEED,
        },
    };
    fake_observer_t tTarget = {
        .tOutput = {
            .tElectricalAngle = { 515396075U },
            .qElectricalSpeed = FOC_SCALAR(0.11f),
            .qConfidence = FOC_ONE,
            .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                           FOC_POSITION_VALID_ELECTRICAL_SPEED |
                           FOC_POSITION_VALID_MULTI_TURN,
            .nMultiTurn = 7,
        },
    };
    foc_position_source_if_t tPrimaryIf = {
        .pContext = &tPrimary,
        .fnReset = fake_observer_reset,
        .fnStep = fake_observer_step,
    };
    foc_position_source_if_t tTargetIf = {
        .pContext = &tTarget,
        .fnReset = fake_observer_reset,
        .fnStep = fake_observer_step,
    };
    foc_observer_selector_params_t tParams = {
        .qMinimumConfidence = FOC_SCALAR(0.8f),
        .qMinimumSpeed = FOC_SCALAR(0.05f),
        .qMaximumAngleError = FOC_SCALAR(0.10f),
        .hwStableSamples = 2U,
        .hwBlendSamples = 4U,
    };
    foc_observer_selector_t tSelector;
    foc_position_input_t tInput = {0};
    foc_position_output_t tOutput;

    TEST_CHECK(foc_observer_selector_Init(&tSelector, &tParams,
                                           &tPrimaryIf) == FOC_RESULT_OK);
    TEST_CHECK(foc_observer_selector_Request(&tSelector, &tTargetIf) ==
               FOC_RESULT_OK);
    for (wIndex = 0U; wIndex < 8U; wIndex++) {
        TEST_CHECK(foc_observer_selector_Step(&tSelector, &tInput,
                                              &tOutput) == FOC_RESULT_OK);
    }
    TEST_CHECK(tSelector.ptActive == &tTargetIf);
    TEST_NEAR(foc_angle_to_turns(tOutput.tElectricalAngle),
              0.12f, 0.003f);
    TEST_CHECK((tOutput.eValidFlags & FOC_POSITION_VALID_MULTI_TURN) !=
               0U);
    TEST_CHECK(tOutput.nMultiTurn == 7);
    TEST_CHECK(tPrimary.wSteps > 0U && tTarget.wSteps > 0U);
    return nFailures;
}

int test_observer(void)
{
    return test_hall() + test_observer_initialization() + test_selector();
}
