/**
 * @file  foc_hal_mdi_adapter.c
 * @brief FOC HAL ↔ AT32F413 MDI bridge (Motor EVB V1)
 *
 * Binds the chip-specific MDI peripherals to the chip-agnostic FOC HAL ops.
 * Follows the STM32G431 pattern.
 */

#include <stddef.h>
#include <stdint.h>
#include "foc_hal.h"
#include "foc_config.h"
#include "halpwm.h"
#include "haladc.h"
#include "mdi_hw.h"
#include "mdi/mdi.h"

static void mdi_pwm_set_duty(q_type duty_u, q_type duty_v, q_type duty_w);
static void mdi_pwm_enable(bool en);
static void mdi_pwm_emergency_stop(void);
static void mdi_adc_start_conversion(void);
static void mdi_adc_get_raw(uint32_t *raw_u, uint32_t *raw_v, uint32_t *raw_w);
static void mdi_adc_offset_calib(foc_adc_calib_t *ptCalib);
static void mdi_adc_reconstruct(phase_current_handle_t *ptHandle);

foc_pwm_ops_t s_tMdiPwmOps = {
    .fnSetDuty       = mdi_pwm_set_duty,
    .fnEnable        = mdi_pwm_enable,
    .fnEmergencyStop = mdi_pwm_emergency_stop,
};

foc_adc_ops_t s_tMdiAdcOps = {
    .fnStartConversion = mdi_adc_start_conversion,
    .fnOffsetCalib     = mdi_adc_offset_calib,
    .fnGetRaw          = mdi_adc_get_raw,
    .fnReconstruct     = mdi_adc_reconstruct,
};

void foc_hal_mdi_register(void)
{
    foc_hal_pwm_register(&s_tMdiPwmOps);
    foc_hal_adc_register(&s_tMdiAdcOps);
}

static void mdi_pwm_set_duty(q_type duty_u, q_type duty_v, q_type duty_w)
{
#if FOC_USE_FPU_HARDWARE
    halpwm_SetDuty((uint32_t)(duty_u * 1000.0f),
                   (uint32_t)(duty_v * 1000.0f),
                   (uint32_t)(duty_w * 1000.0f));
#else
    halpwm_SetDuty((uint32_t)((duty_u * 1000) >> Q_SHIFT),
                   (uint32_t)((duty_v * 1000) >> Q_SHIFT),
                   (uint32_t)((duty_w * 1000) >> Q_SHIFT));
#endif
}

static void mdi_pwm_enable(bool en)
{
    MDI_Enable(HW.ptMotorU, en);
}

static void mdi_pwm_emergency_stop(void)
{
    MDI_Enable(HW.ptMotorU, false);
}

static void mdi_adc_start_conversion(void)
{
    /* Preempt ADC triggered by TMR1 CH4 hardware — no software start needed */
}

static void mdi_adc_get_raw(uint32_t *raw_u, uint32_t *raw_v, uint32_t *raw_w)
{
    uint16_t hwU = 0, hwV = 0, hwW = 0;
    haladc_GetPreemptRaw(&hwU, &hwV, &hwW);
    if (raw_u) *raw_u = hwU;
    if (raw_v) *raw_v = hwV;
    if (raw_w) *raw_w = hwW;
}

static void mdi_adc_offset_calib(foc_adc_calib_t *ptCalib)
{
    uint64_t sumU = 0, sumV = 0, sumW = 0;
    uint32_t raw_u = 0, raw_v = 0, raw_w = 0;
    uint16_t i;

    for (i = 0; i < FOC_OFFSET_CALIB_TIMES; i++) {
        mdi_adc_get_raw(&raw_u, &raw_v, &raw_w);
        sumU += raw_u;
        sumV += raw_v;
        sumW += raw_w;
    }
    ptCalib->wOffsetU      = (uint32_t)(sumU / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetV      = (uint32_t)(sumV / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetW      = (uint32_t)(sumW / FOC_OFFSET_CALIB_TIMES);
    ptCalib->bIsCalibrated = true;
}

static void mdi_adc_reconstruct(phase_current_handle_t *ptHandle)
{
    uint32_t raw_u = 0, raw_v = 0, raw_w = 0;
    mdi_adc_get_raw(&raw_u, &raw_v, &raw_w);

    foc_adc_calib_t *ptCalib = &ptHandle->tCalib;

    uint32_t baseU = ptCalib->bIsCalibrated ? ptCalib->wOffsetU : 2048U;
    uint32_t baseV = ptCalib->bIsCalibrated ? ptCalib->wOffsetV : 2048U;
    uint32_t baseW = ptCalib->bIsCalibrated ? ptCalib->wOffsetW : 2048U;

    int32_t diu = (int32_t)raw_u - (int32_t)baseU;
    int32_t div = (int32_t)raw_v - (int32_t)baseV;
    int32_t diw = (int32_t)raw_w - (int32_t)baseW;

    switch (ptHandle->eTopology) {
        case SENSING_TOPOLOGY_3P:
            ptHandle->qIu = _Q((float)diu / (float)baseU);
            ptHandle->qIv = _Q((float)div / (float)baseV);
            ptHandle->qIw = _Q((float)diw / (float)baseW);
            break;

        case SENSING_TOPOLOGY_2P:
            ptHandle->qIu = _Q((float)diu / (float)baseU);
            ptHandle->qIv = _Q((float)div / (float)baseV);
            ptHandle->qIw = -ptHandle->qIu - ptHandle->qIv;
            break;

        case SENSING_TOPOLOGY_1P:
        default:
            ptHandle->qIu = Q_ZERO;
            ptHandle->qIv = Q_ZERO;
            ptHandle->qIw = Q_ZERO;
            break;
    }
}
