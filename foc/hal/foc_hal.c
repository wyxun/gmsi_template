/*******************************************************************************
 * @file    foc_hal.c
 * @brief   Instance-scoped, architecture-independent FOC hardware wrappers
 ******************************************************************************/

#include "foc_hal.h"

#include <stddef.h>

foc_result_t foc_hal_Validate(const foc_hal_t *ptHal)
{
    if (ptHal == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptHal->tPwm.fnSetDuty == NULL || ptHal->tPwm.fnEnable == NULL ||
        ptHal->tPwm.fnEmergencyStop == NULL ||
        ptHal->tAdc.fnReconstruct == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

foc_result_t foc_hal_SetDuty(const foc_pwm_if_t *ptPwm,
                             q_type qDutyU,
                             q_type qDutyV,
                             q_type qDutyW)
{
    if (ptPwm == NULL || ptPwm->fnSetDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    if (qDutyU < FOC_ZERO || qDutyU > FOC_ONE ||
        qDutyV < FOC_ZERO || qDutyV > FOC_ONE ||
        qDutyW < FOC_ZERO || qDutyW > FOC_ONE) {
        return FOC_RESULT_OUT_OF_RANGE;
    }
    return ptPwm->fnSetDuty(ptPwm->pContext, qDutyU, qDutyV, qDutyW);
}

foc_result_t foc_hal_Enable(const foc_pwm_if_t *ptPwm, bool bEnable)
{
    if (ptPwm == NULL || ptPwm->fnEnable == NULL) {
        return FOC_RESULT_NULL;
    }
    return ptPwm->fnEnable(ptPwm->pContext, bEnable);
}

void foc_hal_EmergencyStop(const foc_pwm_if_t *ptPwm)
{
    if (ptPwm != NULL && ptPwm->fnEmergencyStop != NULL) {
        ptPwm->fnEmergencyStop(ptPwm->pContext);
    }
}

foc_result_t foc_hal_CurrentCalibrate(const foc_adc_if_t *ptAdc,
                                      foc_adc_calib_t *ptCalib)
{
    if (ptAdc == NULL || ptCalib == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptAdc->fnOffsetCalib == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return ptAdc->fnOffsetCalib(ptAdc->pContext, ptCalib);
}

foc_result_t foc_hal_CurrentGetRaw(const foc_adc_if_t *ptAdc,
                                   uint32_t *pwRawU,
                                   uint32_t *pwRawV,
                                   uint32_t *pwRawW)
{
    if (ptAdc == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptAdc->fnGetRaw == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return ptAdc->fnGetRaw(ptAdc->pContext, pwRawU, pwRawV, pwRawW);
}

foc_result_t foc_hal_CurrentReconstruct(
    const foc_adc_if_t *ptAdc,
    phase_current_handle_t *ptHandle)
{
    if (ptAdc == NULL || ptHandle == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptAdc->fnReconstruct == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return ptAdc->fnReconstruct(ptAdc->pContext, ptHandle);
}
