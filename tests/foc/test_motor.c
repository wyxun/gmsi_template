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
    bool bFailEnable;
    bool bFailDuty;
    bool bFailSample;
} fake_motor_hw_t;

static foc_result_t fake_set_duty(void *pContext,
                                  foc_scalar_t qDutyU,
                                  foc_scalar_t qDutyV,
                                  foc_scalar_t qDutyW)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    if (ptHw->bFailDuty) return FOC_RESULT_INVALID_ARGUMENT;
    ptHw->qDutyU = qDutyU;
    ptHw->qDutyV = qDutyV;
    ptHw->qDutyW = qDutyW;
    return FOC_RESULT_OK;
}

static foc_result_t fake_enable(void *pContext, bool bEnable)
{
    fake_motor_hw_t *ptHw = (fake_motor_hw_t *)pContext;

    if (ptHw->bFailEnable) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptHw->bEnabled = bEnable;
    ptHw->wEnableCalls++;
    return FOC_RESULT_OK;
}

static foc_result_t fake_calibrate(void *pContext,
                                   foc_adc_calib_t *ptCalib)
{
    (void)pContext;
    ptCalib->wOffsetU = 101U;
    ptCalib->wOffsetV = 202U;
    ptCalib->wOffsetW = 303U;
    ptCalib->bIsCalibrated = true;
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

    if (ptHw->bFailSample) return FOC_RESULT_INVALID_ARGUMENT;

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
    tConfig.tHal.tAdc.fnOffsetCalib = fake_calibrate;
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
    motor_snapshot_t tSnapshotA;
    motor_snapshot_t tSnapshotB;
    motor_event_t tEvent;
    motor_handle_t tUninitialized = {0};

    TEST_CHECK(motor_GetSnapshot(NULL, &tSnapshotA) == FOC_RESULT_NULL);
    TEST_CHECK(!motor_DebugReadEvent(NULL, &tEvent));
    TEST_CHECK(motor_GetRawCurrent(NULL, NULL, NULL, NULL) ==
               FOC_RESULT_NULL);
    TEST_CHECK(motor_Start(NULL, NULL) ==
               FOC_RESULT_NULL);
    TEST_CHECK(motor_LowFrequencyStep(NULL) == FOC_RESULT_NULL);
    TEST_CHECK(motor_HighFrequencyStep(NULL) == FOC_RESULT_NULL);
    motor_Reset(NULL);
    motor_EmergencyStop(NULL, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(motor_Stop(NULL) == FOC_RESULT_NULL);
    motor_SetVoltageReference(NULL, FOC_ONE, FOC_ONE);
    motor_SetCurrentReference(NULL, FOC_ONE, FOC_ONE);
    motor_SetSpeedReference(NULL, FOC_ONE);
    motor_SetPositionReference(NULL, FOC_ONE);
    TEST_CHECK(motor_GetSnapshot(&tUninitialized, &tSnapshotA) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_GetRawCurrent(&tUninitialized, NULL, NULL, NULL) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_Start(&tUninitialized, NULL) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_LowFrequencyStep(&tUninitialized) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_HighFrequencyStep(&tUninitialized) ==
               FOC_RESULT_INVALID_ARGUMENT);
    motor_Reset(&tUninitialized);
    motor_EmergencyStop(&tUninitialized, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(motor_Stop(&tUninitialized) == FOC_RESULT_INVALID_ARGUMENT);
    motor_SetVoltageReference(&tUninitialized, FOC_ONE, FOC_ONE);
    motor_SetCurrentReference(&tUninitialized, FOC_ONE, FOC_ONE);
    motor_SetSpeedReference(&tUninitialized, FOC_ONE);
    motor_SetPositionReference(&tUninitialized, FOC_ONE);
    TEST_CHECK(motor_GetSnapshot(&tUninitialized, &tSnapshotA) ==
               FOC_RESULT_INVALID_ARGUMENT);

    TEST_CHECK(motor_Init(&tMotorA, &tConfigA) == FOC_RESULT_OK);
    TEST_CHECK(motor_Init(&tMotorB, &tConfigB) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshotA.eRunState == MOTOR_STATE_IDLE);
    TEST_CHECK(tSnapshotA.eControlMode ==
               MOTOR_CONTROL_VOLTAGE_OPEN_LOOP);
    TEST_CHECK(tSnapshotA.eActiveSourceValidFlags ==
               FOC_POSITION_VALID_NONE);
    TEST_CHECK(tSnapshotA.eCandidateSourceValidFlags ==
               FOC_POSITION_VALID_NONE);
    TEST_CHECK(tSnapshotA.tOpenLoopAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshotA.tActiveAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshotA.tCandidateAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshotA.qAngleError == FOC_ZERO);
    TEST_CHECK(tSnapshotA.qBlendFactor == FOC_ZERO);
    TEST_CHECK(tSnapshotA.wEventOverwriteCount == 0U);
    TEST_CHECK(tSnapshotA.tCurrentReference.qD == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tCurrentReference.qQ == FOC_ZERO);
    TEST_CHECK(tSnapshotA.wFaults == MOTOR_FAULT_NONE);
    TEST_CHECK(tSnapshotA.tPhaseCurrent.qIu == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tPhaseCurrent.qIv == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tPhaseCurrent.qIw == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tCurrent.qD == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tCurrent.qQ == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tVoltage.qD == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tVoltage.qQ == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tDuty.qU == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tDuty.qV == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tDuty.qW == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tElectricalAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshotA.qElectricalSpeed == FOC_ZERO);
    TEST_CHECK(tSnapshotA.qVbus == FOC_ZERO);
    TEST_CHECK(tSnapshotA.tCurrentCalibration.wOffsetU == 2048U);
    TEST_CHECK(tSnapshotA.tCurrentCalibration.wOffsetV == 2048U);
    TEST_CHECK(tSnapshotA.tCurrentCalibration.wOffsetW == 2048U);
    TEST_CHECK(!tSnapshotA.tCurrentCalibration.bIsCalibrated);
    TEST_CHECK(!tSnapshotA.bPwmEnabled);

    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(motor_DebugReadEvent(&tMotorA, &tEvent));
    TEST_CHECK(tEvent.eType == MOTOR_EVENT_FAULT);
    TEST_CHECK(tEvent.wSequence == 1U);
    TEST_CHECK(tEvent.eFromState == MOTOR_STATE_IDLE);
    TEST_CHECK(tEvent.eToState == MOTOR_STATE_FAULT);
    TEST_CHECK((tEvent.wFaults & MOTOR_FAULT_HARDWARE) != 0U);
    TEST_CHECK(!motor_DebugReadEvent(&tMotorA, &tEvent));
    TEST_CHECK(tHwA.wStopCalls == 1U && tHwB.wStopCalls == 0U);
    TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotorB, &tSnapshotB) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshotA.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK((tSnapshotA.wFaults & MOTOR_FAULT_HARDWARE) != 0U);
    TEST_CHECK(!tSnapshotA.bPwmEnabled);
    TEST_CHECK(tSnapshotB.eRunState == MOTOR_STATE_IDLE);
    TEST_CHECK(tSnapshotB.wFaults == MOTOR_FAULT_NONE);
    TEST_CHECK(tSnapshotB.qVbus == FOC_ZERO);

    motor_Reset(&tMotorB);
    TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshotA.eRunState == MOTOR_STATE_FAULT);
    TEST_CHECK(tHwB.wEnableCalls == 0U);

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
        motor_run_config_t tRun = {
            .eControlMode = MOTOR_CONTROL_CURRENT,
            .qInitialAngle = FOC_SCALAR(0.1f),
            .qOpenLoopSpeed = FOC_SCALAR(0.2f),
            .qAcceleration = FOC_SCALAR(100.0f),
        };
        TEST_CHECK(motor_Start(&tMotorA, &tRun) == FOC_RESULT_OK);
        TEST_CHECK(motor_Start(&tMotorB, &tRun) == FOC_RESULT_OK);
        TEST_CHECK(motor_RunFSM(&tMotorA) == fsm_rt_on_going);
        TEST_CHECK(motor_RunFSM(&tMotorB) == fsm_rt_on_going);
        TEST_CHECK(motor_RunFSM(&tMotorA) == fsm_rt_on_going);
        TEST_CHECK(motor_RunFSM(&tMotorB) == fsm_rt_on_going);
        TEST_CHECK(motor_RunFSM(&tMotorA) == fsm_rt_cpl);
        TEST_CHECK(motor_RunFSM(&tMotorB) == fsm_rt_cpl);
        motor_SetCurrentReference(&tMotorA, FOC_SCALAR(0.2f),
                                         FOC_SCALAR(0.3f));
        motor_SetCurrentReference(&tMotorB, FOC_SCALAR(-0.1f),
                                         FOC_SCALAR(-0.2f));
        TEST_CHECK(motor_HighFrequencyStep(&tMotorB) ==
                   FOC_RESULT_OK);
        TEST_CHECK(motor_HighFrequencyStep(&tMotorA) ==
                   FOC_RESULT_OK);
        TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
        TEST_CHECK(motor_GetSnapshot(&tMotorB, &tSnapshotB) == FOC_RESULT_OK);
        TEST_NEAR(foc_to_float(tSnapshotA.tVoltage.qD),
                  0.2f, 0.003f);
        TEST_NEAR(foc_to_float(tSnapshotA.tVoltage.qQ),
                  0.3f, 0.003f);
        TEST_NEAR(foc_to_float(tSnapshotB.tVoltage.qD),
                  -0.1f, 0.003f);
        TEST_NEAR(foc_to_float(tSnapshotB.tVoltage.qQ),
                  -0.2f, 0.003f);
        TEST_CHECK(tSnapshotA.tCurrent.qD == FOC_ZERO);
        TEST_CHECK(tSnapshotA.tCurrent.qQ == FOC_ZERO);
        TEST_CHECK(tSnapshotA.tDuty.qU == tHwA.qDutyU);
        TEST_CHECK(tSnapshotA.tDuty.qV == tHwA.qDutyV);
        TEST_CHECK(tSnapshotA.tDuty.qW == tHwA.qDutyW);
        TEST_CHECK(tSnapshotA.bPwmEnabled);
        TEST_CHECK(foc_angle_to_turns(tSnapshotA.tElectricalAngle) > 0.1f);
        TEST_CHECK(tHwA.qDutyU != tHwB.qDutyU ||
                   tHwA.qDutyV != tHwB.qDutyV ||
                   tHwA.qDutyW != tHwB.qDutyW);
        tHwB.bFailSample = true;
        TEST_CHECK(motor_HighFrequencyStep(&tMotorB) != FOC_RESULT_OK);
        TEST_CHECK(motor_GetSnapshot(&tMotorB, &tSnapshotB) == FOC_RESULT_OK);
        TEST_CHECK(tSnapshotB.eRunState == MOTOR_STATE_FAULT);
        TEST_CHECK(!tSnapshotB.bPwmEnabled && tHwB.wStopCalls == 1U);
        tHwA.bFailDuty = true;
        TEST_CHECK(motor_HighFrequencyStep(&tMotorA) != FOC_RESULT_OK);
        TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
        TEST_CHECK(tSnapshotA.eRunState == MOTOR_STATE_FAULT);
        TEST_CHECK(tSnapshotA.wFaults == MOTOR_FAULT_HARDWARE);
        TEST_CHECK(!tSnapshotA.bPwmEnabled && tHwA.wStopCalls == 2U);
    }

    /*
     * Event ring: fixed capacity 4 with overwrite-oldest policy. Six
     * fault stops produce independent monotonic sequences 1..6; the two
     * oldest records are dropped and counted, remaining events drain in
     * FIFO order. Fault payloads accumulate: 1, 3, 7, 15, 31, 31.
     */
    TEST_CHECK(motor_Init(&tMotorA, &tConfigA) == FOC_RESULT_OK);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_HARDWARE);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_CURRENT_SAMPLE);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_INVALID_COMMAND);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_POSITION_SOURCE);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_TRANSITION_TIMEOUT);
    motor_EmergencyStop(&tMotorA, MOTOR_FAULT_HARDWARE);
    TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshotA.wEventSequence == 6U);
    TEST_CHECK(tSnapshotA.wEventOverwriteCount == 2U);
    {
        static const uint32_t awExpectedFaults[4] = {
            MOTOR_FAULT_HARDWARE | MOTOR_FAULT_CURRENT_SAMPLE |
                MOTOR_FAULT_INVALID_COMMAND,
            MOTOR_FAULT_HARDWARE | MOTOR_FAULT_CURRENT_SAMPLE |
                MOTOR_FAULT_INVALID_COMMAND | MOTOR_FAULT_POSITION_SOURCE,
            MOTOR_FAULT_HARDWARE | MOTOR_FAULT_CURRENT_SAMPLE |
                MOTOR_FAULT_INVALID_COMMAND | MOTOR_FAULT_POSITION_SOURCE |
                MOTOR_FAULT_TRANSITION_TIMEOUT,
            MOTOR_FAULT_HARDWARE | MOTOR_FAULT_CURRENT_SAMPLE |
                MOTOR_FAULT_INVALID_COMMAND | MOTOR_FAULT_POSITION_SOURCE |
                MOTOR_FAULT_TRANSITION_TIMEOUT,
        };
        uint32_t wPreviousSequence = 0U;

        for (uint32_t wIndex = 0U; wIndex < 4U; wIndex++) {
            TEST_CHECK(motor_DebugReadEvent(&tMotorA, &tEvent));
            TEST_CHECK(tEvent.eType == MOTOR_EVENT_FAULT);
            TEST_CHECK(tEvent.wSequence == 3U + wIndex);
            TEST_CHECK(tEvent.wSequence > wPreviousSequence);
            wPreviousSequence = tEvent.wSequence;
            TEST_CHECK(tEvent.eFromState == MOTOR_STATE_FAULT);
            TEST_CHECK(tEvent.eToState == MOTOR_STATE_FAULT);
            TEST_CHECK(tEvent.wFaults == awExpectedFaults[wIndex]);
        }
        TEST_CHECK(!motor_DebugReadEvent(&tMotorA, &tEvent));
    }

    {
        motor_hf_profile_snapshot_t tProfileA = {0};
        motor_hf_profile_snapshot_t tProfileB = {0};
        foc_result_t eProfA = motor_GetHighFrequencyProfileSnapshot(&tMotorA, &tProfileA);
        foc_result_t eProfB = motor_GetHighFrequencyProfileSnapshot(&tMotorB, &tProfileB);
#if FOC_HF_PROFILE
        TEST_CHECK(eProfA == FOC_RESULT_OK);
        TEST_CHECK(eProfB == FOC_RESULT_OK);
#else
        TEST_CHECK(eProfA == FOC_RESULT_DISABLED);
        TEST_CHECK(eProfB == FOC_RESULT_DISABLED);
        TEST_CHECK(tProfileA.wSampleSequence == 0U);
        TEST_CHECK(tProfileB.wSampleSequence == 0U);
#endif
    }

    return nFailures;
}
