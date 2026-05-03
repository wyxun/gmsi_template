/**
 * @file  foc_hal_gdi_adapter.c
 * @brief GDI backend adapter — binds GDI peripheral drivers to FOC HAL ops
 *
 * Place under peripheral/<chip>/ as chip-specific binding layer.
 * Chip-agnostic FOC code under foc/ does not need modification.
 */

#include <stddef.h>
#include <stdint.h>
#include "foc_hal.h"
#include "foc_config.h"
#include "haltim1.h"
#include "haladc.h"
#include "gdi_hw.h"
#include "gdi/gdi.h"

static void gdi_pwm_set_duty(q_type duty_u, q_type duty_v, q_type duty_w);
static void gdi_pwm_enable(bool en);
static void gdi_pwm_emergency_stop(void);
static void gdi_adc_start_conversion(void);
static void gdi_adc_get_raw(uint32_t *raw_u, uint32_t *raw_v, uint32_t *raw_w);
static void gdi_adc_offset_calib(foc_adc_calib_t *ptCalib);
static void gdi_adc_reconstruct(phase_current_handle_t *ptHandle);

foc_pwm_ops_t s_tGdiPwmOps = {
    .fnSetDuty       = gdi_pwm_set_duty,
    .fnEnable        = gdi_pwm_enable,
    .fnEmergencyStop = gdi_pwm_emergency_stop,
};

foc_adc_ops_t s_tGdiAdcOps = {
    .fnStartConversion = gdi_adc_start_conversion,
    .fnOffsetCalib     = gdi_adc_offset_calib,
    .fnGetRaw          = gdi_adc_get_raw,
    .fnReconstruct     = gdi_adc_reconstruct,
};

void foc_hal_gdi_register(void)
{
    foc_hal_pwm_register(&s_tGdiPwmOps);
    foc_hal_adc_register(&s_tGdiAdcOps);
}

static void gdi_pwm_set_duty(q_type duty_u, q_type duty_v, q_type duty_w)
{
#if FOC_USE_FPU_HARDWARE
    GDI_Write(HW.ptMotorU, (int32_t)(duty_u * 1000.0f));
    GDI_Write(HW.ptMotorV, (int32_t)(duty_v * 1000.0f));
    GDI_Write(HW.ptMotorW, (int32_t)(duty_w * 1000.0f));
#else
    GDI_Write(HW.ptMotorU, (int32_t)((duty_u * 1000) >> Q_SHIFT));
    GDI_Write(HW.ptMotorV, (int32_t)((duty_v * 1000) >> Q_SHIFT));
    GDI_Write(HW.ptMotorW, (int32_t)((duty_w * 1000) >> Q_SHIFT));
#endif
}

static void gdi_pwm_enable(bool en)
{
    GDI_Enable(HW.ptMotorU, en);
}

static void gdi_pwm_emergency_stop(void)
{
    GDI_Enable(HW.ptMotorU, false);
}

static void gdi_adc_start_conversion(void)
{
    /* Injected ADC triggered by TIM1 CH4 hardware — no software start needed */
}

static void gdi_adc_get_raw(uint32_t *raw_u, uint32_t *raw_v, uint32_t *raw_w)
{
    if (raw_u) *raw_u = haladc_GetInjected(HALADC_ADC1, 0);
    if (raw_v) *raw_v = haladc_GetInjected(HALADC_ADC2, 1);
    if (raw_w) *raw_w = haladc_GetInjected(HALADC_ADC1, 1);
}

static void gdi_adc_offset_calib(foc_adc_calib_t *ptCalib)
{
    uint64_t sumU = 0, sumV = 0, sumW = 0;
    uint32_t raw_u = 0, raw_v = 0, raw_w = 0;

    for (uint16_t i = 0; i < FOC_OFFSET_CALIB_TIMES; i++) {
        gdi_adc_get_raw(&raw_u, &raw_v, &raw_w);
        sumU += raw_u;
        sumV += raw_v;
        sumW += raw_w;
    }
    ptCalib->wOffsetU      = (uint32_t)(sumU / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetV      = (uint32_t)(sumV / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetW      = (uint32_t)(sumW / FOC_OFFSET_CALIB_TIMES);
    ptCalib->bIsCalibrated = true;
}

static void gdi_adc_reconstruct(phase_current_handle_t *ptHandle)
{
    uint32_t raw_u = 0, raw_v = 0, raw_w = 0;
    gdi_adc_get_raw(&raw_u, &raw_v, &raw_w);

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
            ptHandle->qIu = Q_ZERO;
            ptHandle->qIv = Q_ZERO;
            ptHandle->qIw = Q_ZERO;
            break;

        default:
            break;
    }
}
