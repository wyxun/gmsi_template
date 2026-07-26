/**
 * @file  as5600.c
 * @brief AS5600 磁旋转编码器驱动（芯片无关，经 mdi_iic_t 访问总线）
 *
 * 寄存器读时序：fnWrite(寄存器地址) + fnRead(数据)，两次独立事务
 * （中间带 STOP），AS5600 支持该时序。若目标平台要求 repeated-start，
 * 在 mdi_iic_t 适配层合并事务，本驱动无需修改。
 */

#include "as5600.h"

#include <string.h>

/* 读 len 字节寄存器数据：先写寄存器地址，再读数据 */
static int32_t as5600_ReadRegs(as5600_t *ptThis, uint8_t chReg,
                               uint8_t *pchBuf, uint16_t hwLen)
{
    mdi_iic_t *ptIic = ptThis->ptIic;

    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0) {
        return -1;
    }
    if (ptIic->fnRead(ptIic->pPriv, pchBuf, hwLen) < 0) {
        return -1;
    }
    return 0;
}

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

int32_t as5600_ReadRawAngle(as5600_t *ptThis, uint16_t *phwAngle)
{
    uint8_t achBuf[2];

    if (ptThis == NULL || phwAngle == NULL) {
        return -1;
    }
    if (as5600_ReadRegs(ptThis, AS5600_REG_RAW_ANGLE_H, achBuf, 2U) < 0) {
        return -1;
    }
    *phwAngle = (uint16_t)(((uint16_t)achBuf[0] << 8 | achBuf[1]) & 0x0FFFU);
    return 0;
}

int32_t as5600_ReadStatus(as5600_t *ptThis, uint8_t *pchStatus)
{
    if (ptThis == NULL || pchStatus == NULL) {
        return -1;
    }
    return as5600_ReadRegs(ptThis, AS5600_REG_STATUS, pchStatus, 1U);
}

bool as5600_IsMagnetOk(uint8_t chStatus)
{
    return (chStatus & AS5600_STATUS_MD) != 0U &&
           (chStatus & (AS5600_STATUS_MH | AS5600_STATUS_ML)) == 0U;
}

int32_t as5600_Update(as5600_t *ptThis)
{
    uint16_t hwRawAngle;
    uint8_t  chStatus = 0U;
    bool     bMagnetOk = false;

    if (ptThis == NULL) {
        return -1;
    }
    if (as5600_ReadRawAngle(ptThis, &hwRawAngle) < 0) {
        ptThis->tSample.bValid = false;
        return -1;
    }
    if (as5600_ReadStatus(ptThis, &chStatus) == 0) {
        bMagnetOk = as5600_IsMagnetOk(chStatus);
    }

    ptThis->tSample.hwRawAngle = hwRawAngle;
    ptThis->tSample.chStatus   = chStatus;
    ptThis->tSample.bMagnetOk  = bMagnetOk;
    ptThis->tSample.bValid     = true;
    ptThis->tSample.wSequence++;    /* 序号最后写，供无锁双读校验 */
    return 0;
}

void as5600_GetSample(const as5600_t *ptThis, as5600_sample_t *ptSample)
{
    uint32_t wSeqBefore, wSeqAfter;

    if (ptThis == NULL || ptSample == NULL) {
        return;
    }
    /* 单写者（低频任务）/单读者（ISR）：序号双读校验，读到一致快照为止。
       读者不会阻塞写者，循环必然终止。 */
    do {
        wSeqBefore = ptThis->tSample.wSequence;
        *ptSample  = ptThis->tSample;
        wSeqAfter  = ptThis->tSample.wSequence;
    } while (wSeqBefore != wSeqAfter);
}
