/****************************************************************************
 * @file    foc_port.c
 * @brief   STM32G431 compile-time FOC ADC/PWM port
 * @author  Codex
 * @date    2026-08-29
 ************************************************************************** */

#include "foc_port.h"

#include <stdint.h>

#include "haladc.h"
#include "mdi_hw.h"
#include "mdi/mdi.h"
#include "port_mdi.h"

#define FOC_PORT_ADC_SAMPLES          512U
#define FOC_PORT_PWM_PERIOD           4250U
#define FOC_PORT_CURRENT_COUNTS_PU   1390U
#define FOC_PORT_OFFSET_MIN         20000U
#define FOC_PORT_OFFSET_MAX         60000U

static uint32_t s_wCalibrationCount;
static uint64_t s_ullSumU;
static uint64_t s_ullSumV;
static uint64_t s_ullSumW;
static bool s_bCalibrationComplete;
static bool s_bCalibrationFailed;

/** @brief Read the injected ADC results for all three phases. */
static void port_read_raw(uint32_t *pwRawU,
                          uint32_t *pwRawV,
                          uint32_t *pwRawW)
{
    *pwRawU = haladc_GetInjected(HALADC_ADC1, 0U);
    *pwRawV = haladc_GetInjected(HALADC_ADC2, 1U);
    *pwRawW = haladc_GetInjected(HALADC_ADC2, 0U);
}

/** @brief Convert one ADC offset delta into normalized current. */
static foc_scalar_t port_normalize_current(int32_t nDelta)
{
    int32_t nBase = (int32_t)FOC_PORT_CURRENT_COUNTS_PU;

    nDelta = nDelta > nBase ? nBase : nDelta;
    nDelta = nDelta < -nBase ? -nBase : nDelta;
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta * FOC_Q_SCALE) / nBase);
#else
    return (foc_scalar_t)nDelta / (foc_scalar_t)nBase;
#endif
}

/** @brief Convert a normalized duty value to the TIM1 compare range. */
static uint32_t port_duty_to_counts(foc_scalar_t qDuty)
{
    qDuty = foc_sat(qDuty, FOC_ZERO, FOC_ONE);
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * FOC_PORT_PWM_PERIOD) /
                      FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * (foc_scalar_t)FOC_PORT_PWM_PERIOD);
#endif
}

/** @brief Check the ADC offset range required by this power stage. */
static bool port_offsets_are_valid(const foc_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetU <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetV >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetV <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetW >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetW <= FOC_PORT_OFFSET_MAX;
}

/** @brief Finalize and validate the accumulated ADC offsets. */
static bool port_store_calibration(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->wOffsetU = (uint32_t)(s_ullSumU /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetV = (uint32_t)(s_ullSumV /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetW = (uint32_t)(s_ullSumW /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->bIsCalibrated = port_offsets_are_valid(ptCalibration);
    s_bCalibrationComplete = ptCalibration->bIsCalibrated;
    s_bCalibrationFailed = !s_bCalibrationComplete;
    return ptCalibration->bIsCalibrated;
}

void foc_port_Init(void)
{
    foc_port_CurrentCalibrationBegin();
}

void foc_port_CurrentCalibrationBegin(void)
{
    s_wCalibrationCount = 0U;
    s_ullSumU = 0U;
    s_ullSumV = 0U;
    s_ullSumW = 0U;
    s_bCalibrationComplete = false;
    s_bCalibrationFailed = false;
}

foc_calibration_state_e foc_port_CurrentCalibrationStep(
    foc_adc_calib_t *ptCalibration)
{
    uint32_t wRawU;
    uint32_t wRawV;
    uint32_t wRawW;

    if (ptCalibration == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    if (s_bCalibrationComplete) {
        return FOC_CALIBRATION_COMPLETE;
    }
    if (s_bCalibrationFailed) {
        return FOC_CALIBRATION_FAILED;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    s_ullSumU += wRawU;
    s_ullSumV += wRawV;
    s_ullSumW += wRawW;
    s_wCalibrationCount++;
    if (s_wCalibrationCount < FOC_PORT_ADC_SAMPLES) {
        return FOC_CALIBRATION_BUSY;
    }
    return port_store_calibration(ptCalibration)
        ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_FAILED;
}

foc_result_t foc_port_CurrentSample(const foc_adc_calib_t *ptCalibration,
                                    foc_core_input_t *ptInput)
{
    uint32_t wRawU;
    uint32_t wRawV;
    uint32_t wRawW;

    if (ptCalibration == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptCalibration->bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptInput->qIu = port_normalize_current(
        (int32_t)ptCalibration->wOffsetU - (int32_t)wRawU);
    ptInput->qIv = port_normalize_current(
        (int32_t)ptCalibration->wOffsetV - (int32_t)wRawV);
    ptInput->qIw = port_normalize_current(
        (int32_t)ptCalibration->wOffsetW - (int32_t)wRawW);
    return FOC_RESULT_OK;
}

foc_result_t foc_port_DutyCommit(const foc_duty_abc_t *ptDuty)
{
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    return port_mdi_MotorPwmSetDuty3(
        port_duty_to_counts(ptDuty->qU),
        port_duty_to_counts(ptDuty->qV),
        port_duty_to_counts(ptDuty->qW)) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_port_PwmEnable(bool bEnable)
{
    if (HW.ptMotorU == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_Enable(HW.ptMotorU, bEnable) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

void foc_port_EmergencyStop(void)
{
    if (HW.ptMotorU != NULL) {
        (void)MDI_Enable(HW.ptMotorU, false);
    }
}
