/**
 * @file  as5600.c
 * @brief AS5600 磁旋转编码器驱动（芯片无关，经 mdi_iic_t 访问总线）
 */

#include "as5600.h"

#include <string.h>

int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic)
{
    if (ptThis == NULL || ptIic == NULL ||
        ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    memset(ptThis, 0, sizeof(*ptThis));
    ptThis->ptIic = ptIic;
    return 0;
}

bool as5600_IsMagnetOk(uint8_t chStatus)
{
    return (chStatus & AS5600_STATUS_MD) != 0U &&
           (chStatus & (AS5600_STATUS_MH | AS5600_STATUS_ML)) == 0U;
}

int32_t as5600_ReadRawAngle(as5600_t *ptThis, uint16_t *phwAngle)
{
    uint8_t chReg = AS5600_REG_RAW_ANGLE_H;
    uint8_t achBuf[2] = {0U, 0U};
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || phwAngle == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, achBuf, 2U) < 0) {
        return -1;
    }
    *phwAngle = (uint16_t)(((uint16_t)achBuf[0] << 8 | achBuf[1]) & 0x0FFFU);
    return 0;
}

int32_t as5600_ReadStatus(as5600_t *ptThis, uint8_t *pchStatus)
{
    uint8_t chReg = AS5600_REG_STATUS;
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || pchStatus == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, pchStatus, 1U) < 0) {
        return -1;
    }
    return 0;
}

int32_t as5600_Update(as5600_t *ptThis)
{
    uint8_t chReg = AS5600_REG_STATUS;
    uint8_t achBuf[3] = {0U, 0U, 0U};
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    /* 一次传输连续读取 STATUS(0x0B) 与 RAW_ANGLE(0x0C, 0x0D) */
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, achBuf, 3U) < 0) {
        ptThis->tSample.bValid = false;
        return -1;
    }
    ptThis->tSample.chStatus = achBuf[0];
    ptThis->tSample.hwRawAngle = (uint16_t)(((uint16_t)achBuf[1] << 8 |
                                             achBuf[2]) & 0x0FFFU);
    ptThis->tSample.bMagnetOk = as5600_IsMagnetOk(achBuf[0]);
    ptThis->tSample.bValid = true;
    ptThis->tSample.wSequence++;
    return 0;
}

void as5600_GetSample(const as5600_t *ptThis, as5600_sample_t *ptSample)
{
    uint32_t wSeqBefore = 0U;
    uint32_t wSeqAfter = 0U;

    if (ptThis == NULL || ptSample == NULL) {
        return;
    }
    /* 单写者（低频任务）/单读者（ISR）：序号双读校验，读到一致快照为止 */
    do {
        wSeqBefore = ptThis->tSample.wSequence;
        *ptSample  = ptThis->tSample;
        wSeqAfter  = ptThis->tSample.wSequence;
    } while (wSeqBefore != wSeqAfter);
}

/* =========================================================================
 * 标准位置传感器接口实现（foc_sensor_ops_t）
 * ========================================================================= */

static int32_t as5600_sensor_Update(void *pPriv)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pPriv;

    if (ptDev == NULL) {
        return -1;
    }
    return as5600_Update(&ptDev->tDriver);
}

static foc_result_t as5600_sensor_Read(void *pPriv,
                                       foc_angle_t *ptMechanicalAngle,
                                       foc_scalar_t *pqMechanicalSpeed,
                                       bool *pbValid)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pPriv;
    as5600_sample_t tSample = {0};
    foc_encoder_sample_t tEncSample = {0};
    foc_encoder_output_t tEncOutput = {0};

    if (ptDev == NULL || ptMechanicalAngle == NULL ||
        pqMechanicalSpeed == NULL || pbValid == NULL) {
        return FOC_RESULT_NULL;
    }
    as5600_GetSample(&ptDev->tDriver, &tSample);
    if (!tSample.bValid) {
        *pbValid = false;
        return FOC_RESULT_OK;
    }
    tEncSample.hwRawAngle = tSample.hwRawAngle;
    tEncSample.wSequence = tSample.wSequence;
    tEncSample.bMagnetOk = tSample.bMagnetOk;
    if (foc_encoder_Step(&ptDev->tObserver, &tEncSample,
                         &tEncOutput) != FOC_RESULT_OK) {
        *pbValid = false;
        return FOC_RESULT_OK;
    }
    *ptMechanicalAngle = tEncOutput.tMechanicalAngle;
    *pqMechanicalSpeed = tEncOutput.qMechanicalSpeed;
    *pbValid = ptDev->tObserver.bValid;
    return FOC_RESULT_OK;
}

/* 电气零位标定由 app 的非阻塞流程统一承担（Id 电流对齐 → fnRead 取机械角
 * → 换算 tElectricalZero）；sensor 层不提供 fnCalibrate（置 NULL），
 * 避免两条 offset 计算路径口径漂移。 */
const foc_sensor_ops_t g_tAs5600SensorOps = {
    .fnUpdate    = as5600_sensor_Update,
    .fnRead      = as5600_sensor_Read,
    .fnCalibrate = NULL,
};

int32_t as5600_sensor_Init(as5600_sensor_t *ptSensor,
                           mdi_iic_t *ptIic,
                           const foc_encoder_params_t *ptParams)
{
    if (ptSensor == NULL || ptIic == NULL || ptParams == NULL) {
        return -1;
    }
    if (as5600_Init(&ptSensor->tDriver, ptIic) != 0) {
        return -1;
    }
    if (foc_encoder_Init(&ptSensor->tObserver, ptParams) != FOC_RESULT_OK) {
        return -1;
    }
    return 0;
}
