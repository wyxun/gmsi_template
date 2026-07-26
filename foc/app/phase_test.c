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
#include "mshell.h"
#include <stdio.h>

/* Throughput stress test: up to WVTEST_MAX extra sawtooth channels pushed
 * at the 1 kHz Step rate to find the loss-free transport ceiling.
 * Channel count is set at runtime via shell `wvtest <k>` (0 = off). */
#define WVTEST_MAX   (MWAVEFORM_MAX_CHANNELS - 2)

static uint8_t s_chAngle;
static uint8_t s_chSin50;
static uint8_t s_achTest[WVTEST_MAX];
static volatile uint8_t s_chTestCount = 0;

void phase_test_waveform_init(void)
{
    mwaveform.Init(NULL);
    s_chAngle = mwaveform.AddChannel("Angle", 1000.0f);
    s_chSin50 = mwaveform.AddChannel("Sin50Hz", 1000.0f);
    for (uint8_t i = 0; i < WVTEST_MAX; i++) {
        char achName[8];
        snprintf(achName, sizeof(achName), "T%u", (unsigned)i);
        s_achTest[i] = mwaveform.AddChannel(achName, 1.0f);
    }
    mwaveform.Start();
    MLOG(I, "[Waveform] FOC App Dynamic Monitoring started (Angle + Sin50Hz)\r\n");
}

void phase_test_waveform_step(void)
{
    static foc_angle_t s_tAngle = { 0 };

    /* Stress-test channels: sawtooth, independent of motor state. */
    if (s_chTestCount != 0) {
        static int16_t s_hwRamp = 0;
        s_hwRamp += 8;
        for (uint8_t i = 0; i < s_chTestCount; i++) {
            mwaveform.PushRaw(s_achTest[i], s_hwRamp);
        }
    }

    /* 1 kHz step rate, 50 Hz signal -> 0.05 turns per step */
    s_tAngle = foc_angle_add_scalar(s_tAngle, 0.05f);

    /* Push 50Hz Angle (0.0 ~ 1.0 pu normalized from BAM32) and corresponding 50Hz Sin wave */
    mwaveform.Push(s_chAngle, foc_angle_to_turns(s_tAngle));
    mwaveform.Push(s_chSin50, _D(foc_angle_sin(s_tAngle)));
    /* mwaveform.Step() is called automatically via modus_Clock() */
}

/* wvtest <k> — push k sawtooth test channels (0..WVTEST_MAX) at 1 kHz. */
static void cmd_wvtest(const char *args)
{
    uint32_t wK = 0;
    const char *p = args;
    while (*p >= '0' && *p <= '9') {
        wK = wK * 10u + (uint32_t)(*p++ - '0');
    }
    if (wK > WVTEST_MAX) {
        wK = WVTEST_MAX;
    }
    s_chTestCount = (uint8_t)wK;
    MLOGF(I, "wvtest: %lu test ch, frame %lu B @1kHz\r\n",
          (unsigned long)wK, (unsigned long)(10 + 2 * wK));
}

MODUS_SHELL_CMD(wvtest, cmd_wvtest, "Waveform throughput stress test <0-14>");

#else /* MWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(void) {}

#endif /* MWAVEFORM_ENABLE */
