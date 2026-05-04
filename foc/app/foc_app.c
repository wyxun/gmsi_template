/*******************************************************************************
 * @file    foc_app.c
 * @brief   FOC 应用层 — GMSI 挂载实现
 ******************************************************************************/

#include <stddef.h>
#include <string.h>
#include "foc_app.h"
#include "foc_core.h"
#include "foc_hal.h"
#include "motor.h"

#undef  this
#define this (*ptThis)

extern foc_pwm_ops_t s_tGdiPwmOps;
extern foc_adc_ops_t s_tGdiAdcOps;

static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run  (uintptr_t wObjectAddr);

gcoroutine_handle_t tGcoroutineFocAppHandle = {
    .bIsRunning = false,
    .pfcn       = NULL,
};

static gmsi_base_t     s_tFocAppBase;

static motor_config_t s_tMotorConfig = {
    .tParams = {
        .qRs            = 0.1f,
        .qLd            = 0.0001f,
        .qLq            = 0.0001f,
        .qKe            = 0.01f,
        .qJ             = 0.0001f,
        .qRatedVoltage  = 24.0f,
        .qRatedCurrent  = 10.0f,
        .chPolePairs    = 4,
    },
    .eTopology = SENSING_TOPOLOGY_3P,
};

static gmsi_base_cfg_t s_tFocAppBaseCfg = {
    .wId     = FOC_APP,
    .wParent = 0,
    .FcnInterface = {
        .Clock = foc_app_Clock,
        .Run   = foc_app_Run,
    },
};

fsm_rt_t foc_app_RunFSM(foc_app_t *ptThis)
{
    motor_handle_t *ptMotor = this.ptMotor;
    static int64_t s_lCounter = 0;

PERFC_PT_BEGIN(this.chState)

    PERFC_PT_WAIT_UNTIL(
        ptMotor != NULL && ptMotor->tRt.eRunState == MOTOR_STATE_START
    )

    PERFC_PT_ENTRY(
        ptMotor->tRt.qThetaE = 0;
        ptMotor->tRt.qId     = _Q(0.1f);
        ptMotor->tRt.qIq     = 0;
    )

    PERFC_PT_DELAY_MS(200)

    PERFC_PT_ENTRY(
        ptMotor->tRt.eRunState = MOTOR_STATE_OPEN_LOOP;
    )

    while (ptMotor->tRt.eRunState == MOTOR_STATE_OPEN_LOOP) {

        if(get_system_ms() - s_lCounter >= 1)
        {
            ptMotor->tRt.qThetaE += _Q(0.01f);
            if (ptMotor->tRt.qThetaE >= _Q(6.283185f)) {
                ptMotor->tRt.qThetaE -= _Q(6.283185f);
            }
            s_lCounter = get_system_ms();
        }

        foc_ab_t tVdq = { .qAlphaOrD = 0, .qBetaOrQ = _Q(0.05f) };
        foc_ab_t tVab;
        q_type   qDu, qDv, qDw;

        foc_ipark(&tVdq, ptMotor->tRt.qThetaE, &tVab);
        foc_svpwm(&tVab, ptMotor->tRt.qVbus, &qDu, &qDv, &qDw);

        if (ptMotor->tPwm.fnSetDuty) {
            ptMotor->tPwm.fnSetDuty(qDu, qDv, qDw);
        }

        {
            static q_type   s_qPeakDuty = 0;
            static q_type   s_qMinDuty  = Q_ONE;
            static uint32_t s_wLastPrintTick = 0;

            q_type qCurrentMax = qDu;
            q_type qCurrentMin = qDu;

            if (qDv > qCurrentMax) qCurrentMax = qDv;
            if (qDv < qCurrentMin) qCurrentMin = qDv;
            if (qDw > qCurrentMax) qCurrentMax = qDw;
            if (qDw < qCurrentMin) qCurrentMin = qDw;

            if (qCurrentMax > s_qPeakDuty) s_qPeakDuty = qCurrentMax;
            if (qCurrentMin < s_qMinDuty)  s_qMinDuty  = qCurrentMin;

            if (get_system_ms() - s_wLastPrintTick >= 500UL) {
                s_wLastPrintTick = get_system_ms();
                GLOGF(T, "[SVPWM] Peak: %.2f%%, Valley: %.2f%%, Vq: %.3f\r\n",
                      _D(s_qPeakDuty)*100.0, _D(s_qMinDuty)*100.0, _D(tVdq.qBetaOrQ));
                s_qPeakDuty = 0;
                s_qMinDuty  = Q_ONE;
            }
        }

        PERFC_PT_YIELD(fsm_rt_on_going);
    }

    do {
    PERFC_PT_ENTRY(
        if (ptMotor->tRt.eRunState == MOTOR_STATE_FAULT) {
            PERFC_PT_RETURN(fsm_rt_cpl);
        }
    )
        PERFC_PT_YIELD(fsm_rt_on_going);
    } while (ptMotor->tRt.eRunState == MOTOR_STATE_CLOSE_LOOP);

PERFC_PT_END()

    return fsm_rt_cpl;
}

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    uint32_t   wEvent;

    if (ptThis == NULL) { return GMSI_EFAIL; }

    wEvent = gbase_EventPend(ptThis->ptBase);
    (void)wEvent;

    foc_app_RunFSM(ptThis);

    if (get_system_ms() - this.lLastHeartbeat >= 1000) {
        this.lLastHeartbeat = get_system_ms();
        GLOGF(T, "[Heartbeat] foc_app is alive, motor_state: %d\r\n",
              (int)ptThis->ptMotor->tRt.eRunState);
    }

    return GMSI_SUCCESS;
}

static int foc_app_Clock(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    if (ptThis == NULL) { return GMSI_EFAIL; }

    phase_test_waveform_step();

    return GMSI_SUCCESS;
}

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t     *ptThis = (foc_app_t *)wObjectAddr;
    foc_app_cfg_t *ptCfg  = (foc_app_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        GLOGF(E, "foc_app_Init: NULL pointer.\n");
        return GMSI_EFAIL;
    }

    memset(ptThis, 0, sizeof(foc_app_t));
    ptThis->ptMotor = ptCfg->ptMotor;
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;

    motor_Init(ptThis->ptMotor, &s_tMotorConfig);
    foc_hal_gdi_register();

    ptThis->ptMotor->tPwm          = s_tGdiPwmOps;
    ptThis->ptMotor->tCurrent.tOps = s_tGdiAdcOps;

    phase_test_waveform_init();
    
    phase_testA();
    phase_testB(ptThis->ptMotor);
    phase_testC(ptThis);

    return gbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
}

void foc_app_SetSpeedRef(foc_app_t *ptThis, q_type qRef)
{
    if (ptThis == NULL) { return; }
    this.qSpeedRef = qRef;
}

void foc_app_Start(foc_app_t *ptThis)
{
    if (ptThis == NULL || this.ptMotor == NULL) { return; }
    if (this.ptMotor->tRt.eRunState == MOTOR_STATE_IDLE) {
        this.ptMotor->tRt.eRunState = MOTOR_STATE_START;
    }
}

void foc_app_Stop(foc_app_t *ptThis)
{
    if (ptThis == NULL) { return; }
    this.chState = 0;
    if (this.ptMotor != NULL) {
        motor_Reset(this.ptMotor);
        if (this.ptMotor->tPwm.fnEnable) {
            this.ptMotor->tPwm.fnEnable(false);
        }
    }
}

#if FOC_SUPPORT
/* FOC motor object */
static motor_handle_t s_tMotor;
GMSI_DECLARE_OBJECT(foc_app, FocApp, .ptMotor = &s_tMotor);
#endif
