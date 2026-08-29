#include <stdio.h>

#include "foc_core.h"

static int test_null_arguments(void)
{
    foc_core_input_t tInput = {0};
    foc_core_command_t tCommand = {0};
    foc_core_state_t tState = {0};

    if (foc_core_step(NULL, &tCommand, &tInput) != FOC_RESULT_NULL) {
        return 1;
    }
    if (foc_core_step(&tState, NULL, &tInput) != FOC_RESULT_NULL) {
        return 1;
    }
    return foc_core_step(&tState, &tCommand, NULL) == FOC_RESULT_NULL ? 0 : 1;
}

static int test_invalid_angle(void)
{
    foc_core_input_t tInput = {0};
    foc_core_command_t tCommand = {0};
    foc_core_state_t tState = {0};

    tInput.bAngleValid = false;
    return foc_core_step(&tState, &tCommand, &tInput) == FOC_RESULT_SAFETY ?
           0 : 1;
}

static int test_voltage_mode(void)
{
    foc_core_input_t tInput = {0};
    foc_core_command_t tCommand = {0};
    foc_core_state_t tState = {0};

    tInput.bAngleValid = true;
    tCommand.eMode = FOC_MODE_VOLTAGE;
    tCommand.tVoltageReference.qD = FOC_SCALAR(0.2f);
    tCommand.tVoltageReference.qQ = FOC_SCALAR(0.1f);
    if (foc_core_step(&tState, &tCommand, &tInput) != FOC_RESULT_OK) {
        return 1;
    }
    if (tState.tVoltage.qD != tCommand.tVoltageReference.qD ||
        tState.tVoltage.qQ != tCommand.tVoltageReference.qQ) {
        return 1;
    }
    return (tState.tDuty.qU >= FOC_ZERO &&
            tState.tDuty.qU <= FOC_ONE &&
            tState.tDuty.qV >= FOC_ZERO &&
            tState.tDuty.qV <= FOC_ONE &&
            tState.tDuty.qW >= FOC_ZERO &&
            tState.tDuty.qW <= FOC_ONE) ? 0 : 1;
}

static int test_reset(void)
{
    foc_core_state_t tState = {0};

    tState.tIdPi.qIntegrator = FOC_ONE;
    tState.tIqPi.qIntegrator = FOC_ONE;
    foc_core_Reset(&tState);
    return (tState.tIdPi.qIntegrator == FOC_ZERO &&
            tState.tIqPi.qIntegrator == FOC_ZERO &&
            tState.tDuty.qU == FOC_HALF &&
            tState.tDuty.qV == FOC_HALF &&
            tState.tDuty.qW == FOC_HALF) ? 0 : 1;
}

static int test_current_mode(void)
{
    const foc_pid_params_t tParams = {
        .tKp = {1, FOC_ZERO},
        .tKiTs = {0, FOC_ZERO},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_NEG_ONE,
        .qOutputMaximum = FOC_ONE,
        .qIntegratorMinimum = FOC_NEG_ONE,
        .qIntegratorMaximum = FOC_ONE
    };
    foc_core_input_t tInput = {0};
    foc_core_command_t tCommand = {0};
    foc_core_state_t tState = {0};

    if (foc_pid_Init(&tState.tIdPi, &tParams) != FOC_RESULT_OK ||
        foc_pid_Init(&tState.tIqPi, &tParams) != FOC_RESULT_OK) {
        return 1;
    }
    tInput.bAngleValid = true;
    tCommand.eMode = FOC_MODE_CURRENT;
    tCommand.tCurrentReference.qQ = FOC_SCALAR(0.2f);
    if (foc_core_step(&tState, &tCommand, &tInput) != FOC_RESULT_OK) {
        return 1;
    }
    return tState.tVoltage.qQ == FOC_SCALAR(0.2f) ? 0 : 1;
}

int main(void)
{
    int nFailures = 0;

    nFailures += test_null_arguments();
    nFailures += test_invalid_angle();
    nFailures += test_voltage_mode();
    nFailures += test_current_mode();
    nFailures += test_reset();
    printf("minimal core: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
