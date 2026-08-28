/**
 * @file    test_motor_observation.c
 * @brief   并行观测源（只观测不参与控制）主机侧行为测试
 *
 * 覆盖 Task 3 的接口契约：
 *   1. validate_run 拒绝无效观测源；
 *   2. 观测源绑定时候选角度/速度/有效性来自观测输出，误差为观测 vs 当前角度；
 *   3. 观测源失败（返回值/故障字）不影响电机运行，候选遥测清零；
 *   4. 未绑定时候选遥测回退到位置源输出（原行为保持）。
 */

#include "test_common.h"

#include "motor.h"
#include "motor_control.h"

typedef struct {
    unsigned wEnableCalls;
    unsigned wStopCalls;
    unsigned wDutyCalls;
    bool bEnabled;
    uint32_t wNow;
} obs_hw_t;

typedef struct {
    unsigned calls;
    foc_result_t eResult;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    foc_position_valid_flag_e eFlags;
    uint32_t wFaults;
} obs_source_t;

typedef struct {
    unsigned calls;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
} active_source_t;

typedef struct {
    unsigned calls;
    foc_angle_t tMechanicalAngle;
    foc_scalar_t qMechanicalSpeed;
} mech_source_t;

typedef struct {
    unsigned calls;
    foc_scalar_t qOutput;
} obs_controller_t;

static foc_result_t obs_set_duty(void *pContext, foc_scalar_t qDutyU,
                                 foc_scalar_t qDutyV, foc_scalar_t qDutyW)
{
    obs_hw_t *ptHw = (obs_hw_t *)pContext;
    (void)qDutyU; (void)qDutyV; (void)qDutyW;
    ptHw->wDutyCalls++;
    return FOC_RESULT_OK;
}

static foc_result_t obs_enable(void *pContext, bool bEnable)
{
    obs_hw_t *ptHw = (obs_hw_t *)pContext;
    ptHw->wEnableCalls++;
    ptHw->bEnabled = bEnable;
    return FOC_RESULT_OK;
}

static void obs_emergency(void *pContext)
{
    obs_hw_t *ptHw = (obs_hw_t *)pContext;
    ptHw->wStopCalls++;
    ptHw->bEnabled = false;
}

static foc_result_t obs_calibrate(void *pContext, foc_adc_calib_t *ptCalib)
{
    (void)pContext;
    ptCalib->bIsCalibrated = true;
    return FOC_RESULT_OK;
}

static uint32_t obs_now(void *pContext)
{
    return ((obs_hw_t *)pContext)->wNow;
}

static uintptr_t obs_enter(void *pContext)
{
    (void)pContext;
    return 0U;
}

static void obs_exit(void *pContext, uintptr_t wState)
{
    (void)pContext;
    (void)wState;
}

static foc_scalar_t obs_controller_step(void *pContext, foc_scalar_t qReference,
                                        foc_scalar_t qFeedback)
{
    obs_controller_t *ptController = (obs_controller_t *)pContext;
    (void)qReference; (void)qFeedback;
    ptController->calls++;
    return ptController->qOutput;
}

static void obs_controller_track(void *pContext, foc_scalar_t qOutput,
                                 foc_scalar_t qReference,
                                 foc_scalar_t qFeedback)
{
    (void)pContext; (void)qOutput; (void)qReference; (void)qFeedback;
}

static foc_result_t obs_active_step(void *pContext,
    const foc_position_input_t *ptInput, foc_position_output_t *ptOutput)
{
    active_source_t *ptSource = (active_source_t *)pContext;
    ptSource->calls++;
    ptOutput->tElectricalAngle = ptSource->tElectricalAngle;
    ptOutput->qElectricalSpeed = ptSource->qElectricalSpeed;
    ptOutput->eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                            FOC_POSITION_VALID_ELECTRICAL_SPEED;
    ptOutput->wTimestamp = ptInput->wTimestamp;
    return FOC_RESULT_OK;
}

static foc_result_t obs_observation_step(void *pContext,
    const foc_position_input_t *ptInput, foc_position_output_t *ptOutput)
{
    obs_source_t *ptObs = (obs_source_t *)pContext;
    ptObs->calls++;
    if (ptObs->eResult != FOC_RESULT_OK) return ptObs->eResult;
    ptOutput->tElectricalAngle = ptObs->tElectricalAngle;
    ptOutput->qElectricalSpeed = ptObs->qElectricalSpeed;
    ptOutput->eValidFlags = ptObs->eFlags;
    ptOutput->wFaults = ptObs->wFaults;
    ptOutput->wTimestamp = ptInput->wTimestamp;
    return FOC_RESULT_OK;
}

static foc_result_t obs_mech_step(void *pContext,
    const foc_position_input_t *ptInput, foc_position_output_t *ptOutput)
{
    mech_source_t *ptSource = (mech_source_t *)pContext;
    ptSource->calls++;
    ptOutput->tMechanicalAngle = ptSource->tMechanicalAngle;
    ptOutput->qMechanicalSpeed = ptSource->qMechanicalSpeed;
    ptOutput->eValidFlags = FOC_POSITION_VALID_MECHANICAL_ANGLE |
                            FOC_POSITION_VALID_MECHANICAL_SPEED;
    ptOutput->wTimestamp = ptInput->wTimestamp;
    return FOC_RESULT_OK;
}

static motor_config_t obs_config(obs_hw_t *ptHw,
                                 obs_controller_t *ptId,
                                 obs_controller_t *ptIq)
{
    motor_config_t tConfig = {0};

    tConfig.tParams.chPolePairs = 4U;
    tConfig.qHighFrequencyPeriod = FOC_SCALAR(0.01f);
    tConfig.qLowFrequencyPeriod = FOC_SCALAR(0.1f);
    tConfig.tPosition.chPolePairs = 4U;
    tConfig.tPosition.chDirection = 1;
    tConfig.eTopology = SENSING_TOPOLOGY_3P;
    tConfig.tHal.tPwm = (foc_pwm_if_t){ptHw, obs_set_duty, obs_enable,
                                       obs_emergency};
    tConfig.tHal.tAdc.pHalContext = ptHw;
    tConfig.tHal.tAdc.fnOffsetCalib = obs_calibrate;
    tConfig.tHal.tAdc.fnReconstruct = test_hf_sample_ok;
    tConfig.tHal.tHfIo = test_hf_io(ptHw);
    tConfig.tTime = (motor_time_if_t){ptHw, obs_now};
    tConfig.tSync = (motor_sync_if_t){ptHw, obs_enter, obs_exit};
    tConfig.tControl.tId = (foc_controller_if_t){
        .pController = ptId, .fnStep = obs_controller_step,
        .fnTrack = obs_controller_track,
    };
    tConfig.tControl.tIq = (foc_controller_if_t){
        .pController = ptIq, .fnStep = obs_controller_step,
        .fnTrack = obs_controller_track,
    };
    return tConfig;
}

static motor_run_config_t obs_run(active_source_t *ptActive,
                                  obs_source_t *ptObs,
                                  bool bBindObservation)
{
    motor_run_config_t tRun = {0};
    static foc_position_source_if_t s_tActiveSource;
    static foc_position_source_if_t s_tObsSource;

    s_tActiveSource = (foc_position_source_if_t){
        .pSourceContext = ptActive, .fnStep = obs_active_step,
    };
    tRun.eControlMode = MOTOR_CONTROL_CURRENT;
    tRun.ptInitialPositionSource = &s_tActiveSource;
    tRun.tCurrentReference = (foc_dq_t){FOC_ZERO, FOC_SCALAR(0.05f)};
    if (bBindObservation) {
        s_tObsSource = (foc_position_source_if_t){
            .pSourceContext = ptObs, .fnStep = obs_observation_step,
        };
        tRun.ptObservationPositionSource = &s_tObsSource;
    }
    return tRun;
}

static int test_invalid_observation_source(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    active_source_t tActive = {0};
    motor_run_config_t tRun = obs_run(&tActive, NULL, false);
    foc_position_source_if_t tInvalid = {0};

    tActive.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.2f));
    tActive.qElectricalSpeed = FOC_SCALAR(0.4f);
    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    tRun.ptObservationPositionSource = &tInvalid;
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_INVALID_ARGUMENT);
    return nFailures;
}

static int test_observation_candidate_telemetry(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    motor_snapshot_t tSnapshot;
    active_source_t tActive = {0};
    obs_source_t tObs = {0};
    motor_run_config_t tRun;

    tActive.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.2f));
    tActive.qElectricalSpeed = FOC_SCALAR(0.4f);
    tObs.eResult = FOC_RESULT_OK;
    tObs.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.30f));
    tObs.qElectricalSpeed = FOC_SCALAR(0.50f);
    tObs.eFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                  FOC_POSITION_VALID_ELECTRICAL_SPEED;
    tRun = obs_run(&tActive, &tObs, true);

    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    TEST_CHECK(tHw.bEnabled);
    /* HF 提供电流/电压；观测器由 motor_ObservationStep 步进，再经 HF 发布候选 */
    for (unsigned i = 0U; i < 5U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    }
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(tObs.calls == 2U);
    TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotor, &tSnapshot) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshot.eRunState == MOTOR_STATE_RUNNING);
    TEST_CHECK(tSnapshot.wFaults == MOTOR_FAULT_NONE);
    TEST_NEAR(foc_angle_to_turns(tSnapshot.tActiveAngle), 0.2f, 0.001f);
    TEST_NEAR(foc_angle_to_turns(tSnapshot.tCandidateAngle), 0.3f, 0.001f);
    TEST_NEAR(foc_to_float(tSnapshot.qCandidateSpeed), 0.5f, 0.001f);
    TEST_NEAR(foc_to_float(tSnapshot.qAngleError), 0.1f, 0.001f);
    TEST_CHECK(tSnapshot.eCandidateSourceValidFlags ==
               (FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                FOC_POSITION_VALID_ELECTRICAL_SPEED));
    TEST_CHECK(motor_Stop(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    return nFailures;
}

static int test_observation_failure_is_benign(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    motor_snapshot_t tSnapshot;
    active_source_t tActive = {0};
    obs_source_t tObs = {0};
    motor_run_config_t tRun;

    tActive.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.2f));
    tActive.qElectricalSpeed = FOC_SCALAR(0.4f);
    tObs.eResult = FOC_RESULT_INVALID_ARGUMENT;
    tObs.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.30f));
    tObs.qElectricalSpeed = FOC_SCALAR(0.50f);
    tObs.eFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE;
    tRun = obs_run(&tActive, &tObs, true);

    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    for (unsigned i = 0U; i < 5U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    }
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(tObs.calls == 2U);
    TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotor, &tSnapshot) == FOC_RESULT_OK);
    /* 观测器失败不得影响电机：仍在运行、无故障 */
    TEST_CHECK(tSnapshot.eRunState == MOTOR_STATE_RUNNING);
    TEST_CHECK(tSnapshot.wFaults == MOTOR_FAULT_NONE);
    /* 候选遥测清零 */
    TEST_CHECK(tSnapshot.tCandidateAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshot.qCandidateSpeed == FOC_ZERO);
    TEST_CHECK(tSnapshot.eCandidateSourceValidFlags ==
               FOC_POSITION_VALID_NONE);
    /* 控制角度仍来自编码器（活动源） */
    TEST_NEAR(foc_angle_to_turns(tSnapshot.tActiveAngle), 0.2f, 0.001f);
    TEST_CHECK(motor_Stop(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    return nFailures;
}

static int test_observation_fault_word_is_benign(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    motor_snapshot_t tSnapshot;
    active_source_t tActive = {0};
    obs_source_t tObs = {0};
    motor_run_config_t tRun;

    tActive.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.2f));
    tActive.qElectricalSpeed = FOC_SCALAR(0.4f);
    tObs.eResult = FOC_RESULT_OK;
    tObs.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.30f));
    tObs.qElectricalSpeed = FOC_SCALAR(0.50f);
    tObs.eFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE;
    tObs.wFaults = FOC_POSITION_FAULT_INVALID_DATA;
    tRun = obs_run(&tActive, &tObs, true);

    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    for (unsigned i = 0U; i < 5U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    }
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_ObservationStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotor, &tSnapshot) == FOC_RESULT_OK);
    TEST_CHECK(tSnapshot.eRunState == MOTOR_STATE_RUNNING);
    TEST_CHECK(tSnapshot.wFaults == MOTOR_FAULT_NONE);
    TEST_CHECK(tSnapshot.tCandidateAngle.wBam32 == 0U);
    TEST_CHECK(tSnapshot.eCandidateSourceValidFlags ==
               FOC_POSITION_VALID_NONE);
    return nFailures;
}

static int test_no_observation_falls_back(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    motor_snapshot_t tSnapshot;
    active_source_t tActive = {0};
    motor_run_config_t tRun;

    tActive.tElectricalAngle = foc_angle_from_scalar(FOC_SCALAR(0.2f));
    tActive.qElectricalSpeed = FOC_SCALAR(0.4f);
    tRun = obs_run(&tActive, NULL, false);

    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    for (unsigned i = 0U; i < 3U; i++) {
        TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    }
    TEST_CHECK(motor_GetSnapshot(&tMotor, &tSnapshot) == FOC_RESULT_OK);
    /* 未绑定观测源：候选遥测回退到位置源输出（原行为） */
    TEST_NEAR(foc_angle_to_turns(tSnapshot.tCandidateAngle), 0.2f, 0.001f);
    TEST_NEAR(foc_to_float(tSnapshot.qCandidateSpeed), 0.4f, 0.001f);
    TEST_NEAR(foc_to_float(tSnapshot.qAngleError), 0.0f, 0.001f);
    return nFailures;
}

static int test_set_position_offset(void)
{
    int nFailures = 0;
    obs_hw_t tHw = {0};
    obs_controller_t tId = {0};
    obs_controller_t tIq = {0};
    motor_config_t tConfig = obs_config(&tHw, &tId, &tIq);
    motor_handle_t tMotor;
    motor_handle_t tUninitialized = {0};
    motor_snapshot_t tSnapshot;
    mech_source_t tMech = {0};
    foc_position_source_if_t tMechSource = {
        .pSourceContext = &tMech, .fnStep = obs_mech_step,
    };
    motor_run_config_t tRun = {
        .eControlMode = MOTOR_CONTROL_CURRENT,
        .ptInitialPositionSource = &tMechSource,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    TEST_CHECK(motor_SetPositionOffset(NULL, (foc_angle_t){0}) ==
               FOC_RESULT_NULL);
    TEST_CHECK(motor_SetPositionOffset(&tUninitialized, (foc_angle_t){0}) ==
               FOC_RESULT_INVALID_ARGUMENT);
    TEST_CHECK(motor_Init(&tMotor, &tConfig) == FOC_RESULT_OK);
    tMech.tMechanicalAngle = foc_angle_from_turns(0.05f);
    tMech.qMechanicalSpeed = FOC_SCALAR(0.10f);
    /* 偏移 0.10 圈：θe = 0.05*4 + 0.10 = 0.30 圈；ωe = 0.10*4 = 0.40 */
    TEST_CHECK(motor_SetPositionOffset(&tMotor,
               foc_angle_from_turns(0.10f)) == FOC_RESULT_OK);
    TEST_CHECK(motor_Start(&tMotor, &tRun) == FOC_RESULT_OK);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_on_going);
    TEST_CHECK(motor_RunFSM(&tMotor) == fsm_rt_cpl);
    TEST_CHECK(motor_HighFrequencyStep(&tMotor) == FOC_RESULT_OK);
    TEST_CHECK(motor_GetSnapshot(&tMotor, &tSnapshot) == FOC_RESULT_OK);
    TEST_NEAR(foc_angle_to_turns(tSnapshot.tActiveAngle), 0.30f, 0.001f);
    TEST_NEAR(foc_to_float(tSnapshot.qActiveSpeed), 0.40f, 0.001f);
    return nFailures;
}

int test_motor_observation(void)
{
    int nFailures = 0;

    nFailures += test_invalid_observation_source();
    nFailures += test_observation_candidate_telemetry();
    nFailures += test_observation_failure_is_benign();
    nFailures += test_observation_fault_word_is_benign();
    nFailures += test_no_observation_falls_back();
    nFailures += test_set_position_offset();
    return nFailures;
}
