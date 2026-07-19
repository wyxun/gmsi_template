/*******************************************************************************
 * @file    foc_hal_mdi_adapter.c
 * @brief   STM32G431 MDI to instance-scoped FOC HAL adapter
 ******************************************************************************/

#include "global_define.h"

#if defined(FOC_SUPPORT) && FOC_SUPPORT

#include "foc_hal_mdi_adapter.h"

#include <stddef.h>
#include <stdint.h>

#include "foc_config.h"
#include "haladc.h"
#include "mdi/mdi.h"

static foc_result_t mdi_pwm_set_duty(void *pContext,
                                     q_type qDutyU,
                                     q_type qDutyV,
                                     q_type qDutyW);
static foc_result_t mdi_pwm_enable(void *pContext, bool bEnable);
static void mdi_pwm_emergency_stop(void *pContext);
static foc_result_t mdi_adc_start_conversion(void *pContext);
static foc_result_t mdi_adc_get_raw(void *pContext,
                                    uint32_t *pwRawU,
                                    uint32_t *pwRawV,
                                    uint32_t *pwRawW);
static foc_result_t mdi_adc_offset_calib(void *pContext,
                                         foc_adc_calib_t *ptCalib);
static foc_result_t mdi_adc_reconstruct(void *pContext,
                                        phase_current_handle_t *ptHandle);
static q_type mdi_adc_normalize(int32_t nDelta, uint32_t wBase);

static foc_mdi_motor_context_t s_tDefaultContext = {
    .ptHardware = &HW,
    .wPwmPeriod = 4250U,
};

foc_result_t foc_hal_mdi_Bind(foc_hal_t *ptHal,
                              foc_mdi_motor_context_t *ptContext)
{
    if (ptHal == NULL || ptContext == NULL ||
        ptContext->ptHardware == NULL || ptContext->wPwmPeriod == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptHal->tPwm.pContext = ptContext;
    ptHal->tPwm.fnSetDuty = mdi_pwm_set_duty;
    ptHal->tPwm.fnEnable = mdi_pwm_enable;
    ptHal->tPwm.fnEmergencyStop = mdi_pwm_emergency_stop;
    ptHal->tAdc.pContext = ptContext;
    ptHal->tAdc.fnStartConversion = mdi_adc_start_conversion;
    ptHal->tAdc.fnOffsetCalib = mdi_adc_offset_calib;
    ptHal->tAdc.fnGetRaw = mdi_adc_get_raw;
    ptHal->tAdc.fnReconstruct = mdi_adc_reconstruct;
    return foc_hal_Validate(ptHal);
}

foc_result_t foc_hal_mdi_BindDefault(foc_hal_t *ptHal)
{
    return foc_hal_mdi_Bind(ptHal, &s_tDefaultContext);
}

static foc_result_t mdi_pwm_set_duty(void *pContext,
                                     q_type qDutyU,
                                     q_type qDutyV,
                                     q_type qDutyW)
{
    foc_mdi_motor_context_t *ptContext =
        (foc_mdi_motor_context_t *)pContext;
    int32_t nResult;

    if (ptContext == NULL || ptContext->ptHardware->ptMotorU == NULL ||
        ptContext->ptHardware->ptMotorV == NULL ||
        ptContext->ptHardware->ptMotorW == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if FOC_USE_FPU_HARDWARE
    nResult = MDI_Write(ptContext->ptHardware->ptMotorU,
                        (uint32_t)(qDutyU * ptContext->wPwmPeriod));
    nResult |= MDI_Write(ptContext->ptHardware->ptMotorV,
                         (uint32_t)(qDutyV * ptContext->wPwmPeriod));
    nResult |= MDI_Write(ptContext->ptHardware->ptMotorW,
                         (uint32_t)(qDutyW * ptContext->wPwmPeriod));
#else
    nResult = MDI_Write(
        ptContext->ptHardware->ptMotorU,
        (uint32_t)((qDutyU * ptContext->wPwmPeriod) >> Q_SHIFT));
    nResult |= MDI_Write(
        ptContext->ptHardware->ptMotorV,
        (uint32_t)((qDutyV * ptContext->wPwmPeriod) >> Q_SHIFT));
    nResult |= MDI_Write(
        ptContext->ptHardware->ptMotorW,
        (uint32_t)((qDutyW * ptContext->wPwmPeriod) >> Q_SHIFT));
#endif
    return nResult == 0 ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t mdi_pwm_enable(void *pContext, bool bEnable)
{
    foc_mdi_motor_context_t *ptContext =
        (foc_mdi_motor_context_t *)pContext;

    if (ptContext == NULL || ptContext->ptHardware->ptMotorU == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_Enable(ptContext->ptHardware->ptMotorU, bEnable) == 0 ?
           FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static void mdi_pwm_emergency_stop(void *pContext)
{
    foc_mdi_motor_context_t *ptContext =
        (foc_mdi_motor_context_t *)pContext;

    if (ptContext != NULL && ptContext->ptHardware->ptMotorU != NULL) {
        (void)MDI_Enable(ptContext->ptHardware->ptMotorU, false);
    }
}

static foc_result_t mdi_adc_start_conversion(void *pContext)
{
    (void)pContext;
    return FOC_RESULT_OK;
}

static foc_result_t mdi_adc_get_raw(void *pContext,
                                    uint32_t *pwRawU,
                                    uint32_t *pwRawV,
                                    uint32_t *pwRawW)
{
    (void)pContext;
    if (pwRawU != NULL) *pwRawU = haladc_GetInjected(HALADC_ADC1, 0U);
    if (pwRawV != NULL) *pwRawV = haladc_GetInjected(HALADC_ADC2, 1U);
    if (pwRawW != NULL) *pwRawW = haladc_GetInjected(HALADC_ADC1, 1U);
    return FOC_RESULT_OK;
}

static foc_result_t mdi_adc_offset_calib(void *pContext,
                                         foc_adc_calib_t *ptCalib)
{
    uint64_t ullSumU = 0U;
    uint64_t ullSumV = 0U;
    uint64_t ullSumW = 0U;
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;
    uint16_t hwIndex;

    if (ptCalib == NULL) {
        return FOC_RESULT_NULL;
    }
    for (hwIndex = 0U; hwIndex < FOC_OFFSET_CALIB_TIMES; hwIndex++) {
        (void)mdi_adc_get_raw(pContext, &wRawU, &wRawV, &wRawW);
        ullSumU += wRawU;
        ullSumV += wRawV;
        ullSumW += wRawW;
    }
    ptCalib->wOffsetU = (uint32_t)(ullSumU / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetV = (uint32_t)(ullSumV / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetW = (uint32_t)(ullSumW / FOC_OFFSET_CALIB_TIMES);
    ptCalib->bIsCalibrated = true;
    return FOC_RESULT_OK;
}

static q_type mdi_adc_normalize(int32_t nDelta, uint32_t wBase)
{
    if (wBase == 0U) {
        return FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    return (q_type)nDelta / (q_type)wBase;
#else
    int32_t nBase = (int32_t)wBase;
    nDelta = nDelta > nBase ? nBase : nDelta;
    nDelta = nDelta < -nBase ? -nBase : nDelta;
    return (q_type)((nDelta * FOC_Q_SCALE) / nBase);
#endif
}

static foc_result_t mdi_adc_reconstruct(void *pContext,
                                        phase_current_handle_t *ptHandle)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;
    uint32_t wBaseU;
    uint32_t wBaseV;
    uint32_t wBaseW;
    int32_t nDeltaU;
    int32_t nDeltaV;
    int32_t nDeltaW;
    foc_adc_calib_t *ptCalib;

    if (ptHandle == NULL) {
        return FOC_RESULT_NULL;
    }
    (void)mdi_adc_get_raw(pContext, &wRawU, &wRawV, &wRawW);
    ptCalib = &ptHandle->tCalib;
    wBaseU = ptCalib->bIsCalibrated ? ptCalib->wOffsetU : 2048U;
    wBaseV = ptCalib->bIsCalibrated ? ptCalib->wOffsetV : 2048U;
    wBaseW = ptCalib->bIsCalibrated ? ptCalib->wOffsetW : 2048U;
    nDeltaU = (int32_t)wRawU - (int32_t)wBaseU;
    nDeltaV = (int32_t)wRawV - (int32_t)wBaseV;
    nDeltaW = (int32_t)wRawW - (int32_t)wBaseW;

    switch (ptHandle->eTopology) {
        case SENSING_TOPOLOGY_3P:
            ptHandle->qIu = mdi_adc_normalize(nDeltaU, wBaseU);
            ptHandle->qIv = mdi_adc_normalize(nDeltaV, wBaseV);
            ptHandle->qIw = mdi_adc_normalize(nDeltaW, wBaseW);
            break;
        case SENSING_TOPOLOGY_2P:
            ptHandle->qIu = mdi_adc_normalize(nDeltaU, wBaseU);
            ptHandle->qIv = mdi_adc_normalize(nDeltaV, wBaseV);
            ptHandle->qIw = foc_sub_sat(
                FOC_ZERO, foc_add_sat(ptHandle->qIu, ptHandle->qIv));
            break;
        case SENSING_TOPOLOGY_1P:
        default:
            ptHandle->qIu = FOC_ZERO;
            ptHandle->qIv = FOC_ZERO;
            ptHandle->qIw = FOC_ZERO;
            break;
    }
    return FOC_RESULT_OK;
}

#endif /* FOC_SUPPORT */
