#include "test_common.h"
#include "motor.h"
#include "motor_control.h"
#include "foc_controller.h"

typedef struct {
    foc_scalar_t qDutyU;
    foc_scalar_t qDutyV;
    foc_scalar_t qDutyW;
    foc_scalar_t qCurrent;
    unsigned int wEnableCalls;
    unsigned int wSampleCalls;
    unsigned int wStopCalls;
    bool bEnabled;
} fake_motor_hw_t;

static foc_result_t fake_set_duty(void *pContext,
                                  foc_scalar_t qDutyU,
                                  foc_scalar_t qDutyV,
                                  foc_scalar_t qDutyW)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    ptHw->qDutyU = qDutyU;
    ptHw->qDutyV = qDutyV;
    ptHw->qDutyW = qDutyW;
    return FOC_RESULT_OK;
}

static foc_result_t fake_enable(void *pContext, bool bEnable)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    ptHw->bEnabled = bEnable;
    ptHw->wEnableCalls++;
    return FOC_RESULT_OK;
}

static void fake_stop(void *pContext)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    ptHw->bEnabled = false;
    ptHw->wStopCalls++;
}

static foc_result_t fake_reconstruct(void *pContext,
                                     phase_current_handle_t *ptCurrent)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    ptCurrent->qIu = ptHw->qCurrent;
    ptCurrent->qIv = foc_sub_sat(FOC_ZERO, ptHw->qCurrent);
    ptCurrent->qIw = FOC_ZERO;
    ptHw->wSampleCalls++;
    return FOC_RESULT_OK;
}

static motor_config_t fake_config(fake_motor_hw_t *ptHw)
{
    motor_config_t tConfig = {0};

    tConfig.tParams.chPolePairs = 4U;
    tConfig.eTopology = SENSING_TOPOLOGY_3P;
    tConfig.tHal.tPwm.pContext = ptHw;
    tConfig.tHal.tPwm.fnSetDuty = fake_set_duty;
    tConfig.tHal.tPwm.fnEnable = fake_enable;
    tConfig.tHal.tPwm.fnEmergencyStop = fake_stop;
    tConfig.tHal.tAdc.pContext = ptHw;
    tConfig.tHal.tAdc.fnReconstruct = fake_reconstruct;
    return tConfig;
}

int test_motor(void)
{
    int nFailures = 0;
    fake_motor_hw_t tHwA = { .qCurrent = FOC_SCALAR(0.25f) };
    fake_motor_hw_t tHwB = { .qCurrent = FOC_SCALAR(-0.50f) };
    motor_config_t tConfigA = fake_config(&tHwA);
    motor_config_t tConfigB = fake_config(&tHwB);
    motor_handle_t tMotorA;
    motor_handle_t tMotorB;

    TEST_CHECK(motor_Init(&tMotorA, &tConfigA) == FOC_RESULT_OK);
    TEST_CHECK(motor_Init(&tMotorB, &tConfigB) == FOC_RESULT_OK);
    TEST_CHECK(tMotorA.tHal.tPwm.pContext == &tHwA);
    TEST_CHECK(tMotorB.tHal.tPwm.pContext == &tHwB);

    TEST_CHECK(motor_SetDuty(&tMotorA, FOC_SCALAR(0.1f),
                             FOC_SCALAR(0.2f), FOC_SCALAR(0.3f)) ==
               FOC_RESULT_OK);
    TEST_CHECK(motor_SetDuty(&tMotorB, FOC_SCALAR(0.7f),
                             FOC_SCALAR(0.8f), FOC_SCALAR(0.9f)) ==
               FOC_RESULT_OK);
    TEST_CHECK(tHwA.qDutyU == FOC_SCALAR(0.1f));
    TEST_CHECK(tHwB.qDutyU == FOC_SCALAR(0.7f));

    TEST_CHECK(motor_Enable(&tMotorA, true) == FOC_RESULT_OK);
    TEST_CHECK(tHwA.bEnabled && !tHwB.bEnabled);
    TEST_CHECK(motor_SampleCurrent(&tMotorB) == FOC_RESULT_OK);
    TEST_CHECK(motor_SampleCurrent(&tMotorA) == FOC_RESULT_OK);
    TEST_CHECK(tMotorA.tCurrent.qIu == FOC_SCALAR(0.25f));
    TEST_CHECK(tMotorB.tCurrent.qIu == FOC_SCALAR(-0.50f));
    TEST_CHECK(tHwA.wSampleCalls == 1U && tHwB.wSampleCalls == 1U);

    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(tHwA.wStopCalls == 1U && tHwB.wStopCalls == 0U);
    TEST_CHECK(tMotorA.tRt.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK((tMotorA.tRt.wFaults & MOTOR_FAULT_HARDWARE) != 0U);
    TEST_CHECK(tMotorB.tRt.eRunState == MOTOR_STATE_IDLE);
    TEST_CHECK(tMotorB.tRt.wFaults == MOTOR_FAULT_NONE);

    motor_Reset(&tMotorB);
    TEST_CHECK(tMotorA.tRt.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK(tHwA.wEnableCalls == 1U && tHwB.wEnableCalls == 0U);

    {
        foc_pid_params_t tPidParams;
        foc_pid_t tIdPidA;
        foc_pid_t tIqPidA;
        foc_pid_t tIdPidB;
        foc_pid_t tIqPidB;

        TEST_CHECK(foc_gain_from_float(1.0f, &tPidParams.tKp) ==
                   FOC_RESULT_OK);
        TEST_CHECK(foc_gain_from_float(0.0f, &tPidParams.tKiTs) ==
                   FOC_RESULT_OK);
        TEST_CHECK(foc_gain_from_float(0.0f, &tPidParams.tKdOverTs) ==
                   FOC_RESULT_OK);
        tPidParams.qOutputMinimum = FOC_SCALAR(-0.8f);
        tPidParams.qOutputMaximum = FOC_SCALAR(0.8f);
        tPidParams.qIntegratorMinimum = FOC_ZERO;
        tPidParams.qIntegratorMaximum = FOC_ZERO;
        TEST_CHECK(foc_pid_Init(&tIdPidA, &tPidParams) == FOC_RESULT_OK);
        TEST_CHECK(foc_pid_Init(&tIqPidA, &tPidParams) == FOC_RESULT_OK);
        TEST_CHECK(foc_pid_Init(&tIdPidB, &tPidParams) == FOC_RESULT_OK);
        TEST_CHECK(foc_pid_Init(&tIqPidB, &tPidParams) == FOC_RESULT_OK);

        tConfigA.tControl.tIdController = foc_controller_FromPid(&tIdPidA);
        tConfigA.tControl.tIqController = foc_controller_FromPid(&tIqPidA);
        tConfigB.tControl.tIdController = foc_controller_FromPid(&tIdPidB);
        tConfigB.tControl.tIqController = foc_controller_FromPid(&tIqPidB);
        tHwA.qCurrent = FOC_ZERO;
        tHwB.qCurrent = FOC_ZERO;
        TEST_CHECK(motor_Init(&tMotorA, &tConfigA) == FOC_RESULT_OK);
        TEST_CHECK(motor_Init(&tMotorB, &tConfigB) == FOC_RESULT_OK);
        TEST_CHECK(motor_ControlStart(&tMotorA, MOTOR_CONTROL_CURRENT) ==
                   FOC_RESULT_OK);
        TEST_CHECK(motor_ControlStart(&tMotorB, MOTOR_CONTROL_CURRENT) ==
                   FOC_RESULT_OK);
        motor_ControlSetCurrentReference(&tMotorA, FOC_SCALAR(0.2f),
                                         FOC_SCALAR(0.3f));
        motor_ControlSetCurrentReference(&tMotorB, FOC_SCALAR(-0.1f),
                                         FOC_SCALAR(-0.2f));
        TEST_CHECK(motor_ControlHighFrequencyStep(&tMotorB) ==
                   FOC_RESULT_OK);
        TEST_CHECK(motor_ControlHighFrequencyStep(&tMotorA) ==
                   FOC_RESULT_OK);
        TEST_NEAR(foc_to_float(tMotorA.tControl.tVoltage.qD),
                  0.2f, 0.003f);
        TEST_NEAR(foc_to_float(tMotorA.tControl.tVoltage.qQ),
                  0.3f, 0.003f);
        TEST_NEAR(foc_to_float(tMotorB.tControl.tVoltage.qD),
                  -0.1f, 0.003f);
        TEST_NEAR(foc_to_float(tMotorB.tControl.tVoltage.qQ),
                  -0.2f, 0.003f);
        TEST_CHECK(tHwA.qDutyU != tHwB.qDutyU ||
                   tHwA.qDutyV != tHwB.qDutyV ||
                   tHwA.qDutyW != tHwB.qDutyW);
        motor_ControlStop(&tMotorA);
        TEST_CHECK(!tHwA.bEnabled && tHwB.bEnabled);
    }

    return nFailures;
}
