#include "test_common.h"
#include "motor.h"

typedef struct {
    unsigned int wInstance;
} fake_encapsulation_hw_t;

static foc_result_t fake_set_duty(void *pContext,
                                  foc_scalar_t qDutyU,
                                  foc_scalar_t qDutyV,
                                  foc_scalar_t qDutyW)
{
    (void)pContext;
    (void)qDutyU;
    (void)qDutyV;
    (void)qDutyW;
    return FOC_RESULT_OK;
}

static foc_result_t fake_enable(void *pContext, bool bEnable)
{
    (void)pContext;
    (void)bEnable;
    return FOC_RESULT_OK;
}

static void fake_stop(void *pContext)
{
    (void)pContext;
}

static foc_result_t fake_reconstruct(void *pContext,
                                     phase_current_handle_t *ptCurrent)
{
    (void)pContext;
    (void)ptCurrent;
    return FOC_RESULT_OK;
}

static motor_config_t fake_config(fake_encapsulation_hw_t *ptHw)
{
    motor_config_t tConfig = {0};

    tConfig.tParams.chPolePairs = 4U;
    tConfig.qHighFrequencyPeriod = FOC_SCALAR(0.001f);
    tConfig.qLowFrequencyPeriod = FOC_SCALAR(0.01f);
    tConfig.tPosition.chPolePairs = 4U;
    tConfig.tPosition.chDirection = 1;
    tConfig.eTopology = SENSING_TOPOLOGY_3P;
    tConfig.tHal.tPwm.pContext = ptHw;
    tConfig.tHal.tPwm.fnSetDuty = fake_set_duty;
    tConfig.tHal.tPwm.fnEnable = fake_enable;
    tConfig.tHal.tPwm.fnEmergencyStop = fake_stop;
    tConfig.tHal.tAdc.pContext = ptHw;
    tConfig.tHal.tAdc.fnReconstruct = fake_reconstruct;
    return tConfig;
}

int test_motor_encapsulation(void)
{
    int nFailures = 0;
    fake_encapsulation_hw_t tHwA = { .wInstance = 1U };
    fake_encapsulation_hw_t tHwB = { .wInstance = 2U };
    motor_config_t tConfigA = fake_config(&tHwA);
    motor_config_t tConfigB = fake_config(&tHwB);
    motor_handle_t tMotorA;
    motor_handle_t tMotorB;
    motor_snapshot_t tSnapshotA;
    motor_snapshot_t tSnapshotB;

    TEST_CHECK(motor_Init(&tMotorA, &tConfigA) == FOC_RESULT_OK);
    TEST_CHECK(motor_Init(&tMotorB, &tConfigB) == FOC_RESULT_OK);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotorB, &tSnapshotB) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshotA.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK(tSnapshotA.wFaults == MOTOR_FAULT_HARDWARE);
    TEST_CHECK(tSnapshotB.eRunState == MOTOR_STATE_IDLE);
    TEST_CHECK(tSnapshotB.wFaults == MOTOR_FAULT_NONE);

    return nFailures;
}
