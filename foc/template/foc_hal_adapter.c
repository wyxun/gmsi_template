/*******************************************************************************
 * @file    foc_hal_adapter.c
 * @brief   Modus FOC — Platform HAL adapter template
 *
 * NOTE: This is a TEMPLATE. Search for "TODO" and replace with your platform's
 * actual register access. The file compiles as-is but produces no real output.
 *
 * Callback contexts (from README §5):
 *   fnSetDuty, fnEmergencyStop  —  called from high-frequency ISR
 *   fnEnable                    —  called from FSM inside critical section
 *   fnOffsetCalib               —  called during startup, PWM already off
 *   fnReconstruct               —  called from high-frequency ISR
 ******************************************************************************/

#include "foc_hal_adapter.h"

#include <stddef.h>
#include <stdint.h>

#include "foc_config.h"

/* ====== static callback prototypes ====== */

static foc_result_t hal_pwm_set_duty(void *pContext,
                                     q_type qDutyU,
                                     q_type qDutyV,
                                     q_type qDutyW);
static foc_result_t hal_pwm_enable(void *pContext, bool bEnable);
static void         hal_pwm_emergency_stop(void *pContext);

static foc_result_t hal_adc_start_conversion(void *pContext);
static foc_result_t hal_adc_get_raw(void *pContext,
                                    uint32_t *pwRawU,
                                    uint32_t *pwRawV,
                                    uint32_t *pwRawW);
static foc_result_t hal_adc_offset_calib(void *pContext,
                                         foc_adc_calib_t *ptCalib);
static foc_result_t hal_adc_reconstruct(void *pContext,
                                        phase_current_handle_t *ptHandle);

/* ====== ADC raw-to-pu normalisation ====== */

static q_type hal_adc_normalize(int32_t nDelta, uint32_t wBase);

/* ====== static default context ====== */

/*
 * Default single-motor context. Adjust wPwmPeriod to your timer's ARR value.
 * For multi-motor boards, create a separate instance per motor and call
 * foc_hal_Bind() instead of foc_hal_BindDefault().
 */
static foc_motor_context_t s_tDefaultContext = {
    .wPwmPeriod = 0U,   /* TODO: set to timer ARR value (e.g. 4250 for G431) */
};

/* ====== public bind API ====== */

foc_result_t foc_hal_Bind(foc_hal_t *ptHal, foc_motor_context_t *ptContext)
{
    if (ptHal == NULL || ptContext == NULL || ptContext->wPwmPeriod == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    ptHal->tPwm.pContext         = ptContext;
    ptHal->tPwm.fnSetDuty        = hal_pwm_set_duty;
    ptHal->tPwm.fnEnable         = hal_pwm_enable;
    ptHal->tPwm.fnEmergencyStop  = hal_pwm_emergency_stop;

    ptHal->tAdc.pContext            = ptContext;
    ptHal->tAdc.fnStartConversion  = hal_adc_start_conversion;
    ptHal->tAdc.fnOffsetCalib      = hal_adc_offset_calib;
    ptHal->tAdc.fnGetRaw           = hal_adc_get_raw;
    ptHal->tAdc.fnReconstruct      = hal_adc_reconstruct;

    return foc_hal_Validate(ptHal);
}

foc_result_t foc_hal_BindDefault(foc_hal_t *ptHal)
{
    return foc_hal_Bind(ptHal, &s_tDefaultContext);
}

/* ====== PWM callbacks ====== */

/*
 * hal_pwm_set_duty — write three-phase duty cycles to timer compare registers.
 *
 * Called from motor_HighFrequencyStep() in ISR context, every PWM cycle.
 * Requirements:
 *   - Three-phase update must be synchronous (shadow registers / BUFFER load).
 *   - Duty range is [0, 1]; scale by wPwmPeriod to get compare value.
 *   - Must be non-blocking and bounded.
 *   - Return non-FOC_RESULT_OK on error to trigger EmergencyStop.
 */
static foc_result_t hal_pwm_set_duty(void *pContext,
                                     q_type qDutyU,
                                     q_type qDutyV,
                                     q_type qDutyW)
{
    foc_motor_context_t *ptCtx = (foc_motor_context_t *)pContext;

    if (ptCtx == NULL) {
        return FOC_RESULT_NULL;
    }

    /* TODO: write duty cycles to timer compare registers
     *
     * Float backend:
     *   uint32_t wCmpU = (uint32_t)(qDutyU * ptCtx->wPwmPeriod);
     * Fixed backend:
     *   uint32_t wCmpU = (uint32_t)((qDutyU * ptCtx->wPwmPeriod) >> Q_SHIFT);
     *
     * Example (STM32):
     *   TIM1->CCR1 = wCmpU;
     *   TIM1->CCR2 = wCmpV;
     *   TIM1->CCR3 = wCmpW;
     */
    (void)qDutyU;
    (void)qDutyV;
    (void)qDutyW;

    return FOC_RESULT_OK;
}

/*
 * hal_pwm_enable — enable or disable PWM output.
 *
 * Called from motor FSM in main-loop context, inside the motor_sync_if_t
 * critical section. bEnable == false means normal stop (graceful), not fault.
 * bEnable == true enables PWM after successful calibration.
 *
 * Implementation MUST:
 *   - Be a bounded register write
 *   - NOT call back into any Modus FOC API
 */
static foc_result_t hal_pwm_enable(void *pContext, bool bEnable)
{
    foc_motor_context_t *ptCtx = (foc_motor_context_t *)pContext;

    if (ptCtx == NULL) {
        return FOC_RESULT_NULL;
    }

    /* TODO: enable/disable timer output
     *
     * Example (STM32, main output enable):
     *   if (bEnable) {
     *       TIM1->BDTR |= TIM_BDTR_MOE;
     *   } else {
     *       TIM1->BDTR &= ~TIM_BDTR_MOE;
     *   }
     */
    (void)bEnable;

    return FOC_RESULT_OK;
}

/*
 * hal_pwm_emergency_stop — immediately disable PWM output.
 *
 * Called from ANY context (ISR, fault path, main loop). Must:
 *   - Immediately force all PWM outputs to safe (low/inactive) state
 *   - Disable timer output / brake
 *   - NOT rely on software flags or delayed actions
 */
static void hal_pwm_emergency_stop(void *pContext)
{
    foc_motor_context_t *ptCtx = (foc_motor_context_t *)pContext;

    if (ptCtx == NULL) {
        return;
    }

    /* TODO: force PWM outputs off immediately
     *
     * Example (STM32):
     *   TIM1->BDTR &= ~TIM_BDTR_MOE;    // main output disable
     *   TIM1->EGR  |= TIM_EGR_UG;        // force update
     */
}

/* ====== ADC callbacks ====== */

/*
 * hal_adc_start_conversion — optionally start an ADC conversion.
 *
 * Only needed if the platform does NOT use timer-triggered ADC. Most
 * centre-aligned PWM designs trigger ADC automatically — return OK and do
 * nothing.
 */
static foc_result_t hal_adc_start_conversion(void *pContext)
{
    (void)pContext;

    /* TODO (optional): start a software-triggered ADC conversion.
     * Most designs leave this empty (timer-triggered ADC). */
    return FOC_RESULT_OK;
}

/*
 * hal_adc_get_raw — read latest ADC results without processing.
 *
 * Used for diagnostic logging (motor_GetRawCurrent()) and by the default
 * offset calibration implementation.
 *
 * Implementations that combine fnOffsetCalib + fnReconstruct into one
 * state machine may leave this as a no-op.
 */
static foc_result_t hal_adc_get_raw(void *pContext,
                                    uint32_t *pwRawU,
                                    uint32_t *pwRawV,
                                    uint32_t *pwRawW)
{
    (void)pContext;

    /* TODO: read ADC conversion results
     *
     * Example (STM32):
     *   uint32_t wU = LL_ADC_REG_ReadConversionData32(ADC1);
     *   uint32_t wV = LL_ADC_REG_ReadConversionData32(ADC2);
     *   uint32_t wW = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1);
     *
     *   if (pwRawU) *pwRawU = wU;
     *   if (pwRawV) *pwRawV = wV;
     *   if (pwRawW) *pwRawW = wW;
     */

    if (pwRawU != NULL) *pwRawU = 0U;
    if (pwRawV != NULL) *pwRawV = 0U;
    if (pwRawW != NULL) *pwRawW = 0U;

    return FOC_RESULT_OK;
}

/*
 * hal_adc_offset_calib — measure ADC offset at zero phase current.
 *
 * Called during motor startup when PWM is guaranteed OFF. Sum FOC_OFFSET_CALIB_TIMES
 * (default 200) samples per phase and store the averages.
 *
 * Two strategies:
 *
 *  Strategy A — blocking (simpler, AT32F413 style):
 *      Loop in-place, call hal_adc_get_raw() each iteration, compute average
 *      in this callback.
 *
 *  Strategy B — non-blocking / high-frequency (STM32G431 style):
 *      Set a "calibration in progress" flag and return with bIsCalibrated=false.
 *      Each subsequent fnReconstruct call accumulates a sample; after the
 *      required count it writes the results and clears the flag.
 *      This adapter implements Strategy A by default.
 */
static foc_result_t hal_adc_offset_calib(void *pContext,
                                         foc_adc_calib_t *ptCalib)
{
    uint64_t ullSumU = 0U;
    uint64_t ullSumV = 0U;
    uint64_t ullSumW = 0U;
    uint32_t wRawU, wRawV, wRawW;
    uint16_t hwIndex;
    foc_result_t eResult;

    if (ptCalib == NULL) {
        return FOC_RESULT_NULL;
    }

    /* Strategy A: blocking accumulate */
    for (hwIndex = 0U; hwIndex < FOC_OFFSET_CALIB_TIMES; hwIndex++) {
        eResult = hal_adc_get_raw(pContext, &wRawU, &wRawV, &wRawW);
        if (eResult != FOC_RESULT_OK) {
            return eResult;
        }
        ullSumU += wRawU;
        ullSumV += wRawV;
        ullSumW += wRawW;
    }

    ptCalib->wOffsetU = (uint32_t)(ullSumU / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetV = (uint32_t)(ullSumV / FOC_OFFSET_CALIB_TIMES);
    ptCalib->wOffsetW = (uint32_t)(ullSumW / FOC_OFFSET_CALIB_TIMES);
    ptCalib->bIsCalibrated = true;

    (void)pContext;
    return FOC_RESULT_OK;
}

/*
 * Normalise a signed ADC delta to pu (per-unit) q_type.
 *
 *   nDelta: ADC reading minus offset (signed, could be negative)
 *   wBase:  offset value used as the normalisation denominator
 *
 * Returns the per-unit value as foc_scalar_t.
 */
static q_type hal_adc_normalize(int32_t nDelta, uint32_t wBase)
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

/*
 * hal_adc_reconstruct — reconstruct three-phase currents from raw ADC readings.
 *
 * Called from motor_HighFrequencyStep() every PWM cycle. This is the most
 * platform-specific callback:
 *
 *   1. Read ADC results (via hal_adc_get_raw or direct register access).
 *   2. Subtract calibration offset (from ptHandle->tCalib).
 *   3. Scale / normalise to per-unit (pu) via hal_adc_normalize.
 *   4. Handle reconstruction topology: 3-resistor (3P), 2-resistor (2P), 1-resistor (1P).
 *   5. Validate results — return error to trigger EmergencyStop on invalid samples.
 *
 * Sampling must avoid switching noise and non-reconstructable zones.
 */
static foc_result_t hal_adc_reconstruct(void *pContext,
                                        phase_current_handle_t *ptHandle)
{
    uint32_t wRawU = 0U, wRawV = 0U, wRawW = 0U;
    uint32_t wBaseU, wBaseV, wBaseW;
    int32_t nDeltaU, nDeltaV, nDeltaW;
    foc_adc_calib_t *ptCalib;
    foc_result_t eResult;

    if (ptHandle == NULL) {
        return FOC_RESULT_NULL;
    }

    /* Step 1: read raw ADC data */
    eResult = hal_adc_get_raw(pContext, &wRawU, &wRawV, &wRawW);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }

    /* Step 2: apply calibration offset */
    ptCalib = &ptHandle->tCalib;

    /* TODO: adjust default offset base if needed (e.g. 2048 for 12-bit, 32768 for 16-bit) */
    wBaseU = ptCalib->bIsCalibrated ? ptCalib->wOffsetU : 2048U;
    wBaseV = ptCalib->bIsCalibrated ? ptCalib->wOffsetV : 2048U;
    wBaseW = ptCalib->bIsCalibrated ? ptCalib->wOffsetW : 2048U;

    /* TODO: verify subtraction direction — does a positive ADC delta correspond
     * to a positive phase current? Swap the subtraction if polarity is inverted. */
    nDeltaU = (int32_t)wRawU - (int32_t)wBaseU;
    nDeltaV = (int32_t)wRawV - (int32_t)wBaseV;
    nDeltaW = (int32_t)wRawW - (int32_t)wBaseW;

    /* Step 3: reconstruct by topology */
    switch (ptHandle->eTopology) {
        case SENSING_TOPOLOGY_3P:
            ptHandle->qIu = hal_adc_normalize(nDeltaU, wBaseU);
            ptHandle->qIv = hal_adc_normalize(nDeltaV, wBaseV);
            ptHandle->qIw = hal_adc_normalize(nDeltaW, wBaseW);
            break;

        case SENSING_TOPOLOGY_2P:
            ptHandle->qIu = hal_adc_normalize(nDeltaU, wBaseU);
            ptHandle->qIv = hal_adc_normalize(nDeltaV, wBaseV);
            ptHandle->qIw = foc_sub_sat(
                FOC_ZERO, foc_add_sat(ptHandle->qIu, ptHandle->qIv));
            break;

        case SENSING_TOPOLOGY_1P:
        default:
            /* Single-shunt not supported by default — return zeros */
            ptHandle->qIu = FOC_ZERO;
            ptHandle->qIv = FOC_ZERO;
            ptHandle->qIw = FOC_ZERO;
            break;
    }

    /* TODO: add validity checks — return FOC_RESULT_OUT_OF_RANGE if any
     * reconstructed current exceeds expected physical bounds. */

    return FOC_RESULT_OK;
}
