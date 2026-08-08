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
#include "mdebug/util_debug.h"
#include <string.h>

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

static uint8_t s_chAngle;
static uint8_t s_chSin50;
static uint8_t s_chId;
static uint8_t s_chIq;
static uint8_t s_chMotorAngle;
static uint8_t s_chSpeed;
static volatile int16_t s_hwIu;
static volatile int16_t s_hwIv;
static volatile int16_t s_hwIw;

void phase_test_waveform_init(void)
{
    mwaveform.Init(NULL);
    s_chAngle = mwaveform.AddChannel("Angle", 1000.0f);
    s_chSin50 = mwaveform.AddChannel("TestSin", 1000.0f);
    s_chId = mwaveform.AddChannel("Id", 1000.0f);
    s_chIq = mwaveform.AddChannel("Iq", 1000.0f);
    s_chMotorAngle = mwaveform.AddChannel("M_Angle", 1000.0f);
    s_chSpeed = mwaveform.AddChannel("Speed", 1000.0f);
    (void)mwaveform.AddVariable("Iu", 1.0f, (void *)&s_hwIu,
                                MWAVEFORM_VAR_RAW);
    (void)mwaveform.AddVariable("Iv", 1.0f, (void *)&s_hwIv,
                                MWAVEFORM_VAR_RAW);
    (void)mwaveform.AddVariable("Iw", 1.0f, (void *)&s_hwIw,
                                MWAVEFORM_VAR_RAW);
    mwaveform.SetRate(0);
    (void)mwaveform.SetStreamRate(50000, 10000);
    (void)mwaveform.SetChannelRate(s_chAngle, 1000);
    (void)mwaveform.SetChannelRate(s_chSin50, 0);
    mwaveform.Start();
    mwaveform.SetRate(0);
    MLOG(I, "[Waveform] FOC App high-rate monitoring started\r\n");
}

void phase_test_waveform_step(void)
{
    static foc_angle_t s_tAngle = { 0 };

    /* 1 kHz step rate, 50 Hz signal -> 0.05 turns per step */
    s_tAngle = foc_angle_add_scalar(s_tAngle, 0.05f);

    /* Push 1 kHz Angle reference; TestSin is generated in the 20 kHz HF path */
    mwaveform.Push(s_chAngle, foc_angle_to_turns(s_tAngle));
}

void phase_test_waveform_hf_step(motor_handle_t *ptMotor)
{
    uint32_t wRawU = 0u;
    uint32_t wRawV = 0u;
    uint32_t wRawW = 0u;
    static foc_angle_t s_tTestAngle = { 0 };

    if (ptMotor == NULL) return;

    /* 100 Hz test sine, generated at 20 kHz and sampled by the 10 kHz stream:
     * 0.005 turns per 50 us = 100 Hz. */
    s_tTestAngle = foc_angle_add_scalar(s_tTestAngle, 0.005f);
    mwaveform.Push(s_chSin50, _D(foc_angle_sin(s_tTestAngle)));

    if (motor_GetRawCurrent(ptMotor, &wRawU, &wRawV, &wRawW) ==
        FOC_RESULT_OK) {
        s_hwIu = (int16_t)(wRawU & 0xFFFFu);
        s_hwIv = (int16_t)(wRawV & 0xFFFFu);
        s_hwIw = (int16_t)(wRawW & 0xFFFFu);
    }

    if (mwaveform.SnapshotIsArmed()) {
        motor_telemetry_t tTelemetry;
        if (motor_GetTelemetry(ptMotor, &tTelemetry) == FOC_RESULT_OK) {
            mwaveform.Push(s_chId,
                           (float)foc_to_float(tTelemetry.tCurrent.qD));
            mwaveform.Push(s_chIq,
                           (float)foc_to_float(tTelemetry.tCurrent.qQ));
            mwaveform.Push(s_chMotorAngle,
                           (float)foc_angle_to_turns(
                               tTelemetry.tActiveAngle));
            mwaveform.Push(s_chSpeed,
                           (float)foc_to_float(tTelemetry.qActiveSpeed));
        }
    }

    mwaveform.Step();
    mwaveform.SnapshotFeed();
}

#else /* MWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(void) {}
void phase_test_waveform_hf_step(motor_handle_t *ptMotor) { (void)ptMotor; }

#endif /* MWAVEFORM_ENABLE */
