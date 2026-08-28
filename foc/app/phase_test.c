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

static uint8_t s_chId;
static uint8_t s_chIq;
static uint8_t s_chAngle;
static uint8_t s_chSpeed;
static uint8_t s_chVd;
static uint8_t s_chVq;
static uint8_t s_chCandAngle;
static uint8_t s_chAngleErr;
static volatile int16_t s_hwIu;
static volatile int16_t s_hwIv;
static volatile int16_t s_hwIw;
static uint32_t s_wOffsetU;     /* 上电校准后的三相零偏（用于波形归零显示） */
static uint32_t s_wOffsetV;
static uint32_t s_wOffsetW;
static uint8_t s_chCalibLoaded; /* 零偏已缓存 */

void phase_test_waveform_init(void)
{
    mwaveform.Init(NULL);
    s_chId = mwaveform.AddChannel("Id", 1000.0f);
    s_chIq = mwaveform.AddChannel("Iq", 1000.0f);
    s_chAngle = mwaveform.AddChannel("Angle", 1000.0f);
    s_chSpeed = mwaveform.AddChannel("Speed", 1000.0f);
    s_chVd = mwaveform.AddChannel("Vd", 1000.0f);
    s_chVq = mwaveform.AddChannel("Vq", 1000.0f);
    s_chCandAngle = mwaveform.AddChannel("CandAngle", 1000.0f);
    s_chAngleErr = mwaveform.AddChannel("AngleErr", 1000.0f);
    (void)mwaveform.AddVariable("Iu", 1.0f, (void *)&s_hwIu,
                                MWAVEFORM_VAR_RAW);
    (void)mwaveform.AddVariable("Iv", 1.0f, (void *)&s_hwIv,
                                MWAVEFORM_VAR_RAW);
    (void)mwaveform.AddVariable("Iw", 1.0f, (void *)&s_hwIw,
                                MWAVEFORM_VAR_RAW);
    s_wOffsetU = 0U;
    s_wOffsetV = 0U;
    s_wOffsetW = 0U;
    s_chCalibLoaded = 0U;
    mwaveform.SetRate(0);
    (void)mwaveform.SetStreamRate(50000, 10000);
    mwaveform.Start();
    mwaveform.SetRate(0);
    MLOG(I, "[Waveform] FOC App high-rate monitoring started\r\n");
}

/* 低频步（1 kHz，foc_app_Clock 上下文，非 ISR）：
   推送快照/遥测类变量（motor_GetSnapshot/GetTelemetry 较重，
   放在 20 kHz ISR 里会撑爆 ISR 预算）。 */
void phase_test_waveform_step(motor_handle_t *ptMotor)
{
    motor_telemetry_t tTelemetry;
    motor_snapshot_t tSnapshot;

    if (ptMotor == NULL) return;

    /* 上电校准完成后缓存一次三相零偏，用于把原始 ADC 归零显示 */
    if (s_chCalibLoaded == 0U) {
        foc_adc_calib_t tCalib;
        if (motor_GetCurrentCalibration(ptMotor, &tCalib) == FOC_RESULT_OK &&
            tCalib.bIsCalibrated) {
            s_wOffsetU = tCalib.wOffsetU;
            s_wOffsetV = tCalib.wOffsetV;
            s_wOffsetW = tCalib.wOffsetW;
            s_chCalibLoaded = 1U;
        }
    }

    if (motor_GetTelemetry(ptMotor, &tTelemetry) == FOC_RESULT_OK) {
        mwaveform.Push(s_chId,
                       (float)foc_to_float(tTelemetry.tCurrent.qD));
        mwaveform.Push(s_chIq,
                       (float)foc_to_float(tTelemetry.tCurrent.qQ));
        mwaveform.Push(s_chAngle,
                       (float)foc_angle_to_turns(
                           tTelemetry.tActiveAngle));
        mwaveform.Push(s_chSpeed,
                       (float)foc_to_float(tTelemetry.qActiveSpeed));
    }
    if (motor_GetSnapshot(ptMotor, &tSnapshot) == FOC_RESULT_OK) {
        mwaveform.Push(s_chVd,
                       (float)foc_to_float(tSnapshot.tVoltage.qD));
        mwaveform.Push(s_chVq,
                       (float)foc_to_float(tSnapshot.tVoltage.qQ));
        mwaveform.Push(s_chCandAngle,
                       (float)foc_angle_to_turns(
                           tSnapshot.tCandidateAngle));
        mwaveform.Push(s_chAngleErr,
                       (float)foc_to_float(tSnapshot.qAngleError));
    }
}

/* 高频步（20 kHz ISR）：只推送轻量三相电流原始偏置值，
   避免在 ISR 里做加锁快照拷贝。 */
void phase_test_waveform_hf_step(motor_handle_t *ptMotor)
{
    uint32_t wRawU = 0u;
    uint32_t wRawV = 0u;
    uint32_t wRawW = 0u;

    if (ptMotor == NULL) return;

    if (motor_GetRawCurrent(ptMotor, &wRawU, &wRawV, &wRawW) ==
        FOC_RESULT_OK) {
        /* 显示偏置后电流：零电流=0，正电流为正（与电机电流符号一致） */
        if (s_chCalibLoaded != 0U) {
            s_hwIu = (int16_t)((int32_t)s_wOffsetU - (int32_t)wRawU);
            s_hwIv = (int16_t)((int32_t)s_wOffsetV - (int32_t)wRawV);
            s_hwIw = (int16_t)((int32_t)s_wOffsetW - (int32_t)wRawW);
        } else {
            s_hwIu = 0;
            s_hwIv = 0;
            s_hwIw = 0;
        }
    }

    mwaveform.Step();
    mwaveform.SnapshotFeed();
}

#else /* MWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(motor_handle_t *ptMotor) { (void)ptMotor; }
void phase_test_waveform_hf_step(motor_handle_t *ptMotor) { (void)ptMotor; }

#endif /* MWAVEFORM_ENABLE */
