/*******************************************************************************
 * @file    phase_test.c
 * @brief   FOC 阶段性验证测试程序
 *
 * 安全边界（Task 8 重构后）：
 *  - phase_testC 与波形监控只读 motor_GetSnapshot()，正常构建保留；
 *  - phase_testA（坐标变换数学自检）与 phase_testB（固定占空比直测）
 *    仅在 FOC_ENABLE_DIAGNOSTIC=1 的诊断构建中编译，正常初始化不会
 *    执行任何固定占空比旁路；phase_testB 只通过
 *    foc/diagnostic/motor_diagnostic.c 的门控诊断 API 访问硬件。
 ******************************************************************************/

#include "foc/foc.h"
#include "foc/math/foc_math_types.h"

#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC
#include "diagnostic/motor_diagnostic.h"

void phase_testA(void)
{
    MLOG(I, "\r\n=== FOC Transform Test ===\r\n");

    foc_angle_t tTheta = foc_angle_from_turns(1.0f / 12.0f);
    q_type qIu = _Q(1.0f);
    q_type qIv = _Q(-0.5f);
    q_type qIw = _Q(-0.5f);

    foc_ab_t tAB, tAB2;
    foc_dq_t tDQ;

    foc_clarke(qIu, qIv, qIw, &tAB);
    (void)foc_park(&tAB, tTheta, &tDQ);
    (void)foc_ipark(&tDQ, tTheta, &tAB2);

    MLOGF(I, "Clarke: Ia=%.3f Ib=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(qIu), _D(qIv), _D(tAB.qAlpha), _D(tAB.qBeta));

    MLOGF(I, "Park:   Alpha=%.3f Beta=%.3f theta=%.3f -> Id=%.3f Iq=%.3f\r\n",
          _D(tAB.qAlpha), _D(tAB.qBeta),
          (double)foc_angle_to_turns(tTheta) * 360.0,
          _D(tDQ.qD), _D(tDQ.qQ));

    MLOGF(I, "iPark:  Id=%.3f Iq=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(tDQ.qD), _D(tDQ.qQ),
          _D(tAB2.qAlpha), _D(tAB2.qBeta));

    q_type qErrAlpha = tAB2.qAlpha - tAB.qAlpha;
    q_type qErrBeta  = tAB2.qBeta  - tAB.qBeta;
    MLOGF(I, "Round-trip error: dAlpha=%.6f dBeta=%.6f\r\n",
          _D(qErrAlpha), _D(qErrBeta));
}

void phase_testB(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        MLOGF(E, "phase_testB: ptMotor is NULL\r\n");
        return;
    }
    /* 固定占空比直测由门控诊断模块执行：IDLE/无故障/占空比/电流/超时
     * 检查全部在 motor 诊断 API 与 motor_diagnostic.c 内完成。 */
    (void)motor_diagnostic_FixedDutyTest(ptMotor);
}

#else /* FOC_ENABLE_DIAGNOSTIC == 0 */

void phase_testA(void) {}
void phase_testB(motor_handle_t *ptMotor)
{
    /* 量产构建：固定占空比旁路已禁用。 */
    (void)ptMotor;
}

#endif /* FOC_ENABLE_DIAGNOSTIC */

void phase_testC(foc_app_t *ptApp)
{
    if (ptApp == NULL || ptApp->ptMotor == NULL) {
        MLOGF(E, "phase_testC: ptApp or ptMotor is NULL\r\n");
        return;
    }

    MLOG(I, "\r\n=== Phase C: SVPWM & Open-Loop Test ===\r\n");
    MLOG(I, "[Test] Motor waiting for PA12 button press to start...\r\n");

    /* foc_app_Start(ptApp); // 默认不自动启动 */
}

/* ---------------------------------------------------------------------------
 *  Waveform Demo — per-unit duty/current monitoring via mwaveform
 *  Data source: motor_GetSnapshot() only (no private motor members).
 * ------------------------------------------------------------------------- */
#if MWAVEFORM_ENABLE
#include "mdebug/mwaveform.h"

static uint8_t s_chDutyU, s_chDutyV, s_chDutyW;
static uint8_t s_chIu, s_chIv, s_chIw;
static uint8_t s_chIqRef, s_chIq;

void phase_test_waveform_init(void)
{
    mwaveform.Init(NULL);
    /* s_chDutyU = mwaveform.AddChannel("DutyU", 1000.0f); */
    /* s_chDutyV = mwaveform.AddChannel("DutyV", 1000.0f); */
    /* s_chDutyW = mwaveform.AddChannel("DutyW", 1000.0f); */
    /* s_chIu    = mwaveform.AddChannel("Iu", 1000.0f);    */
    /* s_chIv    = mwaveform.AddChannel("Iv", 1000.0f);    */
    /* s_chIw    = mwaveform.AddChannel("Iw", 1000.0f);    */
    s_chIqRef = mwaveform.AddChannel("IqRef", 1000.0f);
    s_chIq    = mwaveform.AddChannel("Iq", 1000.0f);
    mwaveform.Start();
    MLOG(I, "[Waveform] FOC App Dynamic Monitoring started (IqRef + Iq)\r\n");
}

void phase_test_waveform_step(void)
{
    extern foc_app_t tFocApp;
    motor_snapshot_t tSnapshot;

#if 1
    if (tFocApp.ptMotor == NULL ||
        motor_GetSnapshot(tFocApp.ptMotor, &tSnapshot) != FOC_RESULT_OK) {
        return;
    }

    /* Push SVPWM Duties from the coherent motor snapshot */
    /* mwaveform.Push(s_chDutyU, _D(tSnapshot.tDuty.qU)); */
    /* mwaveform.Push(s_chDutyV, _D(tSnapshot.tDuty.qV)); */
    /* mwaveform.Push(s_chDutyW, _D(tSnapshot.tDuty.qW)); */

    /* Push reconstructed phase currents */
    /* mwaveform.Push(s_chIu, _D(tSnapshot.tPhaseCurrent.qIu)); */
    /* mwaveform.Push(s_chIv, _D(tSnapshot.tPhaseCurrent.qIv)); */
    /* mwaveform.Push(s_chIw, _D(tSnapshot.tPhaseCurrent.qIw)); */

    /* Push Iq Reference and Real Iq Feedback */
    mwaveform.Push(s_chIqRef, _D(tSnapshot.tCurrentReference.qQ));
    mwaveform.Push(s_chIq, _D(tSnapshot.tCurrent.qQ));
#else
    /* TEMP DEBUG: fixed test pattern to verify the waveform data path.
     * DutyU/V/W = 0.1/0.2/0.3 (constant), Iu = ramping counter. */
    static float s_fRamp = -1.0f;
    s_fRamp += 0.001f;
    if (s_fRamp > 1.0f) {
        s_fRamp = -1.0f;
    }
    mwaveform.Push(s_chDutyU, 0.1f);
    mwaveform.Push(s_chDutyV, 0.2f);
    mwaveform.Push(s_chDutyW, 0.3f);
    mwaveform.Push(s_chIu, s_fRamp);
    mwaveform.Push(s_chIv, -s_fRamp);
    mwaveform.Push(s_chIw, 0.5f);
#endif
    /* mwaveform.Step() is called automatically via modus_Clock() */
}

#else /* MWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(void) {}

#endif /* MWAVEFORM_ENABLE */
