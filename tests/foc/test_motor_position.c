#include "test_common.h"

#include <limits.h>

#include "motor_position.h"
#include "foc_hall.h"
#include "foc_smo.h"

static foc_result_t fake_step(void *pContext,
                              const foc_position_input_t *ptInput,
                              foc_position_output_t *ptOutput)
{
    (void)pContext;
    ptOutput->tMechanicalAngle = foc_angle_from_scalar(
        foc_from_float(0.20f));
    ptOutput->qMechanicalSpeed = foc_from_float(0.10f);
    ptOutput->wTimestamp = ptInput->wTimestamp;
    ptOutput->eValidFlags = FOC_POSITION_VALID_MECHANICAL_ANGLE |
                            FOC_POSITION_VALID_MECHANICAL_SPEED;
    return FOC_RESULT_OK;
}

static uint8_t fake_read_hall(void *pContext)
{
    return *(uint8_t *)pContext;
}

static int test_position_helpers(void)
{
    int nFailures = 0;
    foc_position_output_t tFrom = {
        .tElectricalAngle = { FOC_SCALAR(0.99f) },
        .qElectricalSpeed = FOC_SCALAR(0.2f),
        .qConfidence = FOC_SCALAR(0.8f),
        .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                       FOC_POSITION_VALID_ELECTRICAL_SPEED,
        .wFaults = FOC_POSITION_FAULT_INVALID_DATA,
        .wTimestamp = 90U,
    };
    foc_position_output_t tTo = {
        .tElectricalAngle = { FOC_SCALAR(0.01f) },
        .qElectricalSpeed = FOC_SCALAR(0.4f),
        .qConfidence = FOC_ONE,
        .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                       FOC_POSITION_VALID_ELECTRICAL_SPEED,
        .wTimestamp = 100U,
    };
    foc_position_output_t tOutput;

    TEST_NEAR(foc_to_float(foc_position_ShortestError(
                  tTo.tElectricalAngle, tFrom.tElectricalAngle)),
              0.02f, 0.002f);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_HALF, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_angle_to_turns(tOutput.tElectricalAngle) < 0.002f ||
               foc_angle_to_turns(tOutput.tElectricalAngle) > 0.998f);
    TEST_CHECK(tOutput.wTimestamp == 90U);
    TEST_CHECK(tOutput.wFaults == FOC_POSITION_FAULT_INVALID_DATA);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    tFrom.tElectricalAngle = foc_angle_from_turns(0.01f);
    tTo.tElectricalAngle = foc_angle_from_turns(0.99f);
    tFrom.wFaults = FOC_POSITION_FAULT_NONE;
    TEST_NEAR(foc_to_float(foc_position_ShortestError(
                  tTo.tElectricalAngle, tFrom.tElectricalAngle)),
              -0.02f, 0.002f);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_HALF, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_angle_to_turns(tOutput.tElectricalAngle) < 0.002f ||
               foc_angle_to_turns(tOutput.tElectricalAngle) > 0.998f);
    tFrom.wFaults = FOC_POSITION_FAULT_INVALID_DATA;
    TEST_CHECK(foc_position_IsFresh(
        &tTo, FOC_POSITION_VALID_ELECTRICAL_ANGLE, 105U, 5U));
    TEST_CHECK(!foc_position_IsFresh(
        &tTo, FOC_POSITION_VALID_MECHANICAL_ANGLE, 105U, 5U));
    TEST_CHECK(!foc_position_IsFresh(
        &tFrom, FOC_POSITION_VALID_ELECTRICAL_ANGLE, 90U, 5U));
    tFrom.wFaults = FOC_POSITION_FAULT_NONE;
    tFrom.eValidFlags = (foc_position_valid_flag_e)(
        tFrom.eValidFlags | FOC_POSITION_VALID_MECHANICAL_ANGLE |
        FOC_POSITION_VALID_MECHANICAL_SPEED |
        FOC_POSITION_VALID_MULTI_TURN);
    tTo.eValidFlags = tFrom.eValidFlags;
    tFrom.tMechanicalAngle = foc_angle_from_turns(0.3f);
    tTo.tMechanicalAngle = foc_angle_from_turns(0.4f);
    tFrom.qMechanicalSpeed = FOC_SCALAR(0.1f);
    tTo.qMechanicalSpeed = FOC_SCALAR(0.2f);
    tFrom.nMultiTurn = 3;
    tTo.nMultiTurn = 4;
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_HALF, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.eValidFlags ==
               (FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                FOC_POSITION_VALID_ELECTRICAL_SPEED |
                FOC_POSITION_VALID_MECHANICAL_ANGLE |
                FOC_POSITION_VALID_MECHANICAL_SPEED));
    TEST_NEAR(foc_angle_to_turns(tOutput.tMechanicalAngle),
              0.35f, 0.002f);
    TEST_NEAR(foc_to_float(tOutput.qMechanicalSpeed), 0.15f, 0.002f);
    TEST_CHECK(tOutput.nMultiTurn == 0);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_ZERO, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.wFaults == tFrom.wFaults);
    TEST_CHECK((tOutput.eValidFlags & FOC_POSITION_VALID_MULTI_TURN) ==
               0U);
    TEST_CHECK(tOutput.nMultiTurn == 0);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_ONE, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.wTimestamp == tFrom.wTimestamp);
    TEST_CHECK((tOutput.eValidFlags & FOC_POSITION_VALID_MULTI_TURN) ==
               0U);
    tFrom.wTimestamp = UINT32_C(0xfffffff0);
    tTo.wTimestamp = UINT32_C(0x10);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_HALF, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.wTimestamp == UINT32_C(0xfffffff0));
    return nFailures;
}

static int test_fixed_full_range_blend(void)
{
    int nFailures = 0;
#if defined(FOC_NUMERIC_FIXED)
    foc_position_output_t tFrom = {
        .qElectricalSpeed = INT32_MIN,
        .qMechanicalSpeed = INT32_MIN,
        .qConfidence = INT32_MIN,
        .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_SPEED |
                       FOC_POSITION_VALID_MECHANICAL_SPEED,
    };
    foc_position_output_t tTo = {
        .qElectricalSpeed = INT32_MAX,
        .qMechanicalSpeed = INT32_MAX,
        .qConfidence = INT32_MAX,
        .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_SPEED |
                       FOC_POSITION_VALID_MECHANICAL_SPEED,
    };
    foc_position_output_t tOutput;

    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_ZERO, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.qElectricalSpeed == INT32_MIN);
    TEST_CHECK(tOutput.qMechanicalSpeed == INT32_MIN);
    TEST_CHECK(tOutput.qConfidence == INT32_MIN);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_HALF, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.qElectricalSpeed > INT32_MIN &&
               tOutput.qElectricalSpeed < INT32_MAX);
    TEST_CHECK(tOutput.qMechanicalSpeed > INT32_MIN &&
               tOutput.qMechanicalSpeed < INT32_MAX);
    TEST_CHECK(tOutput.qConfidence > INT32_MIN &&
               tOutput.qConfidence < INT32_MAX);
    TEST_CHECK(foc_position_Blend(&tFrom, &tTo, FOC_ONE, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.qElectricalSpeed == INT32_MAX);
    TEST_CHECK(tOutput.qMechanicalSpeed == INT32_MAX);
    TEST_CHECK(tOutput.qConfidence == INT32_MAX);
#endif
    return nFailures;
}

static int test_hall_source_adapter(void)
{
    int nFailures = 0;
    uint8_t chCode = 5U;
    foc_hall_params_t tParams;
    foc_hall_t tHall;
    foc_hall_source_adapter_t tAdapter;
    foc_position_source_if_t tSource;
    foc_position_input_t tInput = { .wTimestamp = 77U };
    foc_position_output_t tOutput = {0};

    foc_hall_DefaultParams(&tParams);
    TEST_CHECK(foc_hall_Init(&tHall, &tParams) == FOC_RESULT_OK);
    TEST_CHECK(foc_hall_source_Init(&tAdapter, &tHall, &chCode,
                                    fake_read_hall) == FOC_RESULT_OK);
    tSource = foc_hall_PositionSourceInterface(&tAdapter);
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.wTimestamp == 77U);
    chCode = 0U;
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    TEST_CHECK(tOutput.wFaults == FOC_POSITION_FAULT_INVALID_DATA);
    TEST_CHECK(!foc_position_IsFresh(
        &tOutput, FOC_POSITION_VALID_ELECTRICAL_ANGLE, 77U, 0U));
    foc_position_source_Reset(&tSource);
    chCode = 5U;
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_OK);
    chCode = 6U;
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(tOutput.wFaults ==
               FOC_POSITION_FAULT_ILLEGAL_TRANSITION);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    return nFailures;
}

static int test_observer_source_adapter(void)
{
    int nFailures = 0;
    foc_smo_params_t tParams = {
        .qModelGain = FOC_SCALAR(0.1f),
        .qResistance = FOC_SCALAR(0.1f),
        .qSlidingGain = FOC_SCALAR(0.5f),
        .qBoundaryInverse = FOC_SCALAR(4.0f),
        .qEmfFilterAlpha = FOC_SCALAR(0.1f),
        .qMinimumBemf = FOC_SCALAR(0.05f),
    };
    foc_smo_t tSmo;
    foc_position_source_if_t tSource;
    foc_position_input_t tInput = { .wTimestamp = 44U };
    foc_position_output_t tOutput = {0};

    TEST_CHECK(foc_smo_Init(&tSmo, &tParams) == FOC_RESULT_OK);
    tSource = foc_smo_PositionSourceInterface(&tSmo);
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.wTimestamp == 44U);
    TEST_CHECK(tOutput.eValidFlags == FOC_POSITION_VALID_NONE);
    TEST_CHECK(tOutput.qConfidence >= FOC_ZERO);
    return nFailures;
}

int test_motor_position(void)
{
    int nFailures = 0;
    foc_position_source_if_t tInvalid = {0};
    foc_position_source_if_t tSource = {
        .fnStep = fake_step,
    };
    foc_position_input_t tInput = {
        .wTimestamp = 123U,
        .qSamplePeriod = foc_from_float(0.001f),
    };
    foc_position_output_t tOutput = {0};
    foc_position_config_t tConfig = {
        .chPolePairs = 4U,
        .chDirection = -1,
        .tMechanicalZero = foc_angle_from_scalar(
            foc_from_float(0.05f)),
        .tElectricalOffset = foc_angle_from_scalar(
            foc_from_float(0.10f)),
    };

    TEST_CHECK(!foc_position_source_IsValid(&tInvalid));
    TEST_CHECK(foc_position_source_IsValid(&tSource));
    TEST_CHECK(foc_position_source_Step(&tSource, &tInput, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(foc_position_ApplyMechanicalConfig(&tConfig, &tOutput) ==
               FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(tOutput.tElectricalAngle.qTurns),
              0.50f, 0.002f);
    TEST_NEAR(foc_to_float(tOutput.qElectricalSpeed), -0.40f,
              0.002f);
    TEST_CHECK(tOutput.wTimestamp == 123U);
    TEST_CHECK((tOutput.eValidFlags & FOC_POSITION_VALID_ELECTRICAL_ANGLE)
               != 0U);
    tOutput.eValidFlags = FOC_POSITION_VALID_MECHANICAL_SPEED |
                          FOC_POSITION_VALID_ELECTRICAL_ANGLE;
    tOutput.tElectricalAngle = foc_angle_from_turns(0.7f);
    TEST_CHECK(foc_position_ApplyMechanicalConfig(&tConfig, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK((tOutput.eValidFlags &
                FOC_POSITION_VALID_ELECTRICAL_ANGLE) == 0U);
    TEST_CHECK((tOutput.eValidFlags &
                FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U);
    TEST_CHECK(tOutput.tElectricalAngle.qTurns == FOC_ZERO);
#if defined(FOC_NUMERIC_FIXED)
    tOutput = (foc_position_output_t){
        .qMechanicalSpeed = INT32_MIN,
        .eValidFlags = FOC_POSITION_VALID_MECHANICAL_SPEED,
    };
    tConfig.chPolePairs = 1U;
    TEST_CHECK(foc_position_ApplyMechanicalConfig(&tConfig, &tOutput) ==
               FOC_RESULT_OK);
    TEST_CHECK(tOutput.qElectricalSpeed == INT32_MAX);
#endif
    return nFailures + test_position_helpers() +
           test_fixed_full_range_blend() + test_hall_source_adapter() +
           test_observer_source_adapter();
}
