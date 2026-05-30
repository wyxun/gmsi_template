/*******************************************************************************
 * @file    phase_test.c
 * @brief   FOC 阶段性验证测试程序
 ******************************************************************************/

#include "foc/foc.h"
#include "foc/math/foc_math_types.h"

void phase_testA(void)
{
    MLOG(I, "\r\n=== FOC Transform Test ===\r\n");

#if FOC_USE_FPU_HARDWARE
    q_type qTheta = _Q(0.5236f);
#else
    q_type qTheta = (q_type)5461;
#endif
    q_type qIu = _Q(1.0f);
    q_type qIv = _Q(-0.5f);
    q_type qIw = _Q(-0.5f);

    foc_ab_t tAB, tDQ, tAB2;

    foc_clarke(qIu, qIv, qIw, &tAB);
    foc_park(&tAB, qTheta, &tDQ);
    foc_ipark(&tDQ, qTheta, &tAB2);

    MLOGF(I, "Clarke: Ia=%.3f Ib=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(qIu), _D(qIv), _D(tAB.qAlphaOrD), _D(tAB.qBetaOrQ));

    MLOGF(I, "Park:   Alpha=%.3f Beta=%.3f theta=%.3f -> Id=%.3f Iq=%.3f\r\n",
          _D(tAB.qAlphaOrD), _D(tAB.qBetaOrQ), _D_ANG(qTheta),
          _D(tDQ.qAlphaOrD), _D(tDQ.qBetaOrQ));

    MLOGF(I, "iPark:  Id=%.3f Iq=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(tDQ.qAlphaOrD), _D(tDQ.qBetaOrQ),
          _D(tAB2.qAlphaOrD), _D(tAB2.qBetaOrQ));

    q_type qErrAlpha = tAB2.qAlphaOrD - tAB.qAlphaOrD;
    q_type qErrBeta  = tAB2.qBetaOrQ  - tAB.qBetaOrQ;
    MLOGF(I, "Round-trip error: dAlpha=%.6f dBeta=%.6f\r\n",
          _D(qErrAlpha), _D(qErrBeta));
}

void phase_testB(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        MLOGF(E, "phase_testB: ptMotor is NULL\r\n");
        return;
    }

    MLOG(I, "\r\n=== Phase B: PWM & ADC Test (Instance based) ===\r\n");

    q_type dutyU = _Q(0.1f);
    q_type dutyV = _Q(0.5f);
    q_type dutyW = _Q(0.9f);

    if (ptMotor->tPwm.fnSetDuty) {
        ptMotor->tPwm.fnSetDuty(dutyU, dutyV, dutyW);
    }
    if (ptMotor->tPwm.fnEnable) {
        ptMotor->tPwm.fnEnable(true);
    }

    if (ptMotor->tCurrent.tOps.fnOffsetCalib) {
        ptMotor->tCurrent.tOps.fnOffsetCalib(&ptMotor->tCurrent.tCalib);
    }

    MLOGF(I, "[PWM] Set Duty: U=%.1f%%, V=%.1f%%, W=%.1f%%\r\n",
          _D(dutyU)*100.0, _D(dutyV)*100.0, _D(dutyW)*100.0);

    if (ptMotor->tCurrent.tOps.fnReconstruct) {
        ptMotor->tCurrent.tOps.fnReconstruct(&ptMotor->tCurrent);
    }

    MLOGF(I, "[ADC] Reconstructed Current: Iu=%.3f, Iv=%.3f, Iw=%.3f\r\n",
          _D(ptMotor->tCurrent.qIu), _D(ptMotor->tCurrent.qIv), _D(ptMotor->tCurrent.qIw));

    if (ptMotor->tPwm.fnEmergencyStop) {
        ptMotor->tPwm.fnEmergencyStop();
    }
    MLOG(I, "[PWM] Emergency Stop triggers successfully.\r\n");
}

void phase_testC(foc_app_t *ptApp)
{
    if (ptApp == NULL || ptApp->ptMotor == NULL) {
        MLOGF(E, "phase_testC: ptApp or ptMotor is NULL\r\n");
        return;
    }

    MLOG(I, "\r\n=== Phase C: SVPWM & Open-Loop Test ===\r\n");
    MLOG(I, "[Test] Requesting Motor Start (IDLE -> START -> OPEN_LOOP)...\r\n");

    foc_app_Start(ptApp);
}

/* ---------------------------------------------------------------------------
 *  Waveform Demo — per-unit sine/cosine via mwaveform
 *  Uses hardware FPU (sinf/cosf) for generation.
 *
 *  Hook:
 *    phase_test_waveform_init()  → call once after modus_Init()
 *    phase_test_waveform_step()  → call from peripheral_Clock() or main loop
 * ------------------------------------------------------------------------- */
#if MWAVEFORM_ENABLE
#include "mdebug/mwaveform.h"

static uint8_t s_chDutyU, s_chDutyV, s_chDutyW;
static uint8_t s_chIu, s_chIv, s_chIw;

void phase_test_waveform_init(void)
{
    mwaveform.Init(NULL);
    s_chDutyU = mwaveform.AddChannel("DutyU", 1000.0f);
    s_chDutyV = mwaveform.AddChannel("DutyV", 1000.0f);
    s_chDutyW = mwaveform.AddChannel("DutyW", 1000.0f);
    s_chIu    = mwaveform.AddChannel("Iu", 1000.0f);
    s_chIv    = mwaveform.AddChannel("Iv", 1000.0f);
    s_chIw    = mwaveform.AddChannel("Iw", 1000.0f);
    mwaveform.Start();
    MLOG(I, "[Waveform] FOC App Dynamic Monitoring started (Duties + Currents)\r\n");
}

void phase_test_waveform_step(void)
{
    /* Push SVPWM Duties from the FOC app object */
    extern foc_app_t tFocApp;
    mwaveform.Push(s_chDutyU, _D(tFocApp.qDutyU));
    mwaveform.Push(s_chDutyV, _D(tFocApp.qDutyV));
    mwaveform.Push(s_chDutyW, _D(tFocApp.qDutyW));

    /* Push reconstructed phase currents */
    if (tFocApp.ptMotor) {
        mwaveform.Push(s_chIu, _D(tFocApp.ptMotor->tCurrent.qIu));
        mwaveform.Push(s_chIv, _D(tFocApp.ptMotor->tCurrent.qIv));
        mwaveform.Push(s_chIw, _D(tFocApp.ptMotor->tCurrent.qIw));
    }

    /* mwaveform.Step() is called automatically via modus_Clock() */
}

#else /* MWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(void) {}

#endif /* MWAVEFORM_ENABLE */
