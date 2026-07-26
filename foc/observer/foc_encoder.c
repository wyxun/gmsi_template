/*******************************************************************************
 * @file    foc_encoder.c
 * @brief   Absolute magnetic-encoder (e.g. AS5600) angle/speed source
 *
 * 新样本到达：12 位码差分（int16 自然回卷）/ tick 数得原始速度，低通滤波；
 * 无新样本：按速度外插。输出机械角度/机械速度有效标志，电角度折算交给
 * foc_position_ApplyMechanicalConfig()。
 ******************************************************************************/

#include "foc_encoder.h"

#include <stddef.h>
#include <string.h>

#define FOC_ENCODER_RESOLUTION      4096U   /* 12-bit */

/* 12 位原始码 → BAM32 机械角度（4096 → 2^32 精确映射，与数值后端无关） */
static foc_angle_t encoder_raw_to_angle(uint16_t hwRawAngle)
{
    foc_angle_t tAngle = { ((uint32_t)hwRawAngle << 20) };
    return tAngle;
}

/* 差分原始码（±2048 回卷）与 tick 数 → turn/tick 速度 */
static foc_scalar_t encoder_sample_speed(int16_t nDelta, uint16_t hwTicks)
{
    if (hwTicks == 0U) {
        return FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    return (foc_scalar_t)nDelta /
           ((foc_scalar_t)FOC_ENCODER_RESOLUTION * (foc_scalar_t)hwTicks);
#else
    return (foc_scalar_t)(((int32_t)nDelta * FOC_Q_SCALE) /
                          ((int32_t)FOC_ENCODER_RESOLUTION * (int32_t)hwTicks));
#endif
}

void foc_encoder_DefaultParams(foc_encoder_params_t *ptParams)
{
    if (ptParams != NULL) {
        ptParams->qSpeedFilterAlpha = FOC_SCALAR(0.25f);
        ptParams->hwInvalidTimeout  = 4U;
    }
}

foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_params_t *ptParams)
{
    if (ptEncoder == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qSpeedFilterAlpha < FOC_ZERO ||
        ptParams->qSpeedFilterAlpha > FOC_ONE ||
        ptParams->hwInvalidTimeout == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptEncoder, 0, sizeof(*ptEncoder));
    ptEncoder->tParams = *ptParams;
    return FOC_RESULT_OK;
}

void foc_encoder_Reset(foc_encoder_t *ptEncoder)
{
    foc_encoder_params_t tParams;

    if (ptEncoder != NULL) {
        tParams = ptEncoder->tParams;
        memset(ptEncoder, 0, sizeof(*ptEncoder));
        ptEncoder->tParams = tParams;
    }
}

/* 无效样本处理：计数、超时置无效、上报故障 */
static foc_result_t encoder_invalid_sample(foc_encoder_t *ptEncoder,
                                           foc_position_output_t *ptOutput)
{
    if (ptEncoder->hwInvalidSamples < UINT16_MAX) {
        ptEncoder->hwInvalidSamples++;
    }
    if (ptEncoder->hwInvalidSamples >=
        ptEncoder->tParams.hwInvalidTimeout) {
        ptEncoder->bValid = false;
        ptEncoder->qConfidence = FOC_ZERO;
    }
    ptOutput->wFaults = FOC_POSITION_FAULT_INVALID_DATA;
    return FOC_RESULT_INVALID_ARGUMENT;
}

foc_result_t foc_encoder_Step(foc_encoder_t *ptEncoder,
                              uint16_t hwRawAngle,
                              uint32_t wSequence,
                              bool bMagnetOk,
                              foc_position_output_t *ptOutput)
{
    if (ptEncoder == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptOutput = (foc_position_output_t){0};

    hwRawAngle &= 0x0FFFU;
    if (!bMagnetOk) {
        return encoder_invalid_sample(ptEncoder, ptOutput);
    }

    if (ptEncoder->hwTicksSinceSample < UINT16_MAX) {
        ptEncoder->hwTicksSinceSample++;
    }

    if (wSequence != ptEncoder->wLastSequence) {
        /* 新样本 */
        ptEncoder->hwInvalidSamples = 0U;
        if (!ptEncoder->bInitialized) {
            ptEncoder->bInitialized = true;
            ptEncoder->tAngle = encoder_raw_to_angle(hwRawAngle);
            ptEncoder->qSpeed = FOC_ZERO;
        } else {
            int16_t nDelta = (int16_t)(hwRawAngle - ptEncoder->hwRawAngle);
            foc_scalar_t qRawSpeed =
                encoder_sample_speed(nDelta, ptEncoder->hwTicksSinceSample);
            foc_scalar_t qSpeedDelta =
                foc_sub_sat(qRawSpeed, ptEncoder->qSpeed);

            ptEncoder->qSpeed = foc_add_sat(
                ptEncoder->qSpeed,
                foc_mul_pu(qSpeedDelta,
                           ptEncoder->tParams.qSpeedFilterAlpha));
            ptEncoder->tAngle = encoder_raw_to_angle(hwRawAngle);
        }
        ptEncoder->hwRawAngle = hwRawAngle;
        ptEncoder->wLastSequence = wSequence;
        ptEncoder->hwTicksSinceSample = 0U;
        ptEncoder->bValid = true;
        ptEncoder->qConfidence = FOC_ONE;
    } else if (ptEncoder->bValid) {
        /* 无新样本：按速度外插 */
        ptEncoder->tAngle =
            foc_angle_add_scalar(ptEncoder->tAngle, ptEncoder->qSpeed);
    }

    *ptOutput = (foc_position_output_t){
        .tMechanicalAngle = ptEncoder->tAngle,
        .qMechanicalSpeed = ptEncoder->qSpeed,
        .qConfidence = ptEncoder->qConfidence,
        .eValidFlags = ptEncoder->bValid
            ? (FOC_POSITION_VALID_MECHANICAL_ANGLE |
               FOC_POSITION_VALID_MECHANICAL_SPEED)
            : FOC_POSITION_VALID_NONE,
    };
    return FOC_RESULT_OK;
}

static void encoder_interface_reset(void *pContext)
{
    foc_encoder_source_adapter_t *ptAdapter =
        (foc_encoder_source_adapter_t *)pContext;
    if (ptAdapter != NULL) {
        foc_encoder_Reset(ptAdapter->ptEncoder);
    }
}

static foc_result_t encoder_interface_step(
    void *pContext,
    const foc_position_input_t *ptInput,
    foc_position_output_t *ptOutput)
{
    foc_encoder_source_adapter_t *ptAdapter =
        (foc_encoder_source_adapter_t *)pContext;
    uint16_t hwRawAngle = 0U;
    uint32_t wSequence  = 0U;
    bool     bMagnetOk  = false;
    foc_result_t eResult;

    if (ptAdapter == NULL || ptInput == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptAdapter->fnReadSample(ptAdapter->pHardwareContext,
                                 &hwRawAngle, &wSequence, &bMagnetOk)) {
        *ptOutput = (foc_position_output_t){0};
        ptOutput->wFaults = FOC_POSITION_FAULT_INVALID_DATA;
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_encoder_Step(ptAdapter->ptEncoder,
                               hwRawAngle, wSequence, bMagnetOk,
                               ptOutput);
    ptOutput->wTimestamp = ptInput->wTimestamp;
    return eResult;
}

foc_result_t foc_encoder_source_Init(
    foc_encoder_source_adapter_t *ptAdapter,
    foc_encoder_t *ptEncoder,
    void *pHardwareContext,
    foc_encoder_read_sample_fn_t fnReadSample)
{
    if (ptAdapter == NULL || ptEncoder == NULL || fnReadSample == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptAdapter = (foc_encoder_source_adapter_t){
        .ptEncoder = ptEncoder,
        .pHardwareContext = pHardwareContext,
        .fnReadSample = fnReadSample,
    };
    return FOC_RESULT_OK;
}

foc_position_source_if_t foc_encoder_PositionSourceInterface(
    foc_encoder_source_adapter_t *ptAdapter)
{
    foc_position_source_if_t tInterface = {
        .pContext = ptAdapter,
        .fnReset = encoder_interface_reset,
        .fnStep = encoder_interface_step,
    };
    return tInterface;
}
