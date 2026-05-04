/*******************************************************************************
 * @file    phase_test.c
 * @brief   FOC 阶段性验证测试程序
 ******************************************************************************/

#include "foc/foc.h"
#include "foc/math/foc_math_types.h"

void phase_testA(void)
{
    GLOG(I, "\r\n=== FOC Transform Test ===\r\n");

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

    GLOGF(I, "Clarke: Ia=%.3f Ib=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(qIu), _D(qIv), _D(tAB.qAlphaOrD), _D(tAB.qBetaOrQ));

    GLOGF(I, "Park:   Alpha=%.3f Beta=%.3f theta=%.3f -> Id=%.3f Iq=%.3f\r\n",
          _D(tAB.qAlphaOrD), _D(tAB.qBetaOrQ), _D_ANG(qTheta),
          _D(tDQ.qAlphaOrD), _D(tDQ.qBetaOrQ));

    GLOGF(I, "iPark:  Id=%.3f Iq=%.3f -> Alpha=%.3f Beta=%.3f\r\n",
          _D(tDQ.qAlphaOrD), _D(tDQ.qBetaOrQ),
          _D(tAB2.qAlphaOrD), _D(tAB2.qBetaOrQ));

    q_type qErrAlpha = tAB2.qAlphaOrD - tAB.qAlphaOrD;
    q_type qErrBeta  = tAB2.qBetaOrQ  - tAB.qBetaOrQ;
    GLOGF(I, "Round-trip error: dAlpha=%.6f dBeta=%.6f\r\n",
          _D(qErrAlpha), _D(qErrBeta));
}

void phase_testB(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        GLOGF(E, "phase_testB: ptMotor is NULL\r\n");
        return;
    }

    GLOG(I, "\r\n=== Phase B: PWM & ADC Test (Instance based) ===\r\n");

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

    GLOGF(I, "[PWM] Set Duty: U=%.1f%%, V=%.1f%%, W=%.1f%%\r\n",
          _D(dutyU)*100.0, _D(dutyV)*100.0, _D(dutyW)*100.0);

    if (ptMotor->tCurrent.tOps.fnReconstruct) {
        ptMotor->tCurrent.tOps.fnReconstruct(&ptMotor->tCurrent);
    }

    GLOGF(I, "[ADC] Reconstructed Current: Iu=%.3f, Iv=%.3f, Iw=%.3f\r\n",
          _D(ptMotor->tCurrent.qIu), _D(ptMotor->tCurrent.qIv), _D(ptMotor->tCurrent.qIw));

    if (ptMotor->tPwm.fnEmergencyStop) {
        ptMotor->tPwm.fnEmergencyStop();
    }
    GLOG(I, "[PWM] Emergency Stop triggers successfully.\r\n");
}

void phase_testC(foc_app_t *ptApp)
{
    if (ptApp == NULL || ptApp->ptMotor == NULL) {
        GLOGF(E, "phase_testC: ptApp or ptMotor is NULL\r\n");
        return;
    }

    GLOG(I, "\r\n=== Phase C: SVPWM & Open-Loop Test ===\r\n");
    GLOG(I, "[Test] Requesting Motor Start (IDLE -> START -> OPEN_LOOP)...\r\n");

    foc_app_Start(ptApp);
}

/* ---------------------------------------------------------------------------
 *  Waveform Demo — per-unit sine/cosine via gwaveform
 *  Uses hardware FPU (sinf/cosf) for generation.
 *
 *  Hook:
 *    phase_test_waveform_init()  → call once after gmsi_Init()
 *    phase_test_waveform_step()  → call from peripheral_Clock() or main loop
 * ------------------------------------------------------------------------- */
#if GWAVEFORM_ENABLE
#include <math.h>
#include "gdebug/gwaveform.h"

static uint8_t s_chSinID, s_chCosID;
static uint32_t s_wPhaseAcc = 0;
static const uint32_t s_wPhaseStep = 42949673;  /* ~10Hz @ 1kHz sample rate */

void phase_test_waveform_init(void)
{
    gwaveform.Init(NULL);
    s_chSinID = gwaveform.AddChannel("Sin", 32767.0f);
    s_chCosID = gwaveform.AddChannel("Cos", 32767.0f);
    gwaveform.Start();
    GLOG(I, "[Waveform] Sin/Cos demo started (10 Hz)\r\n");
}

void phase_test_waveform_step(void)
{
    s_wPhaseAcc += s_wPhaseStep;
    float fAngle = (float)(s_wPhaseAcc >> 24) * 0.0245437f;  /* 2*pi/256 */

    gwaveform.Push(s_chSinID, sinf(fAngle));
    gwaveform.Push(s_chCosID, cosf(fAngle));
    /* gwaveform.Step() is called automatically via gmsi_Clock() */
}

#else /* GWAVEFORM_ENABLE == 0 */

void phase_test_waveform_init(void) {}
void phase_test_waveform_step(void) {}

#endif /* GWAVEFORM_ENABLE */
