/**
 * @file  hali2c.c
 * @brief I2C1 master driver — PB7(SDA) / PB8(SCL), AF4, blocking
 *
 * 供 mdi_iic_t 适配层使用（port_mdi.c）。仅在低频任务上下文中阻塞调用，
 * 不允许在 20 kHz 高频控制 ISR 内使用。
 */

#include "hali2c.h"

static I2C_HandleTypeDef s_hi2c1;

void hali2c_Init(void)
{
    GPIO_InitTypeDef tGpio = {0};

    I2C1_CLK_EN();
    I2C1_GPIO_CLK_EN();

    tGpio.Pin       = I2C1_SDA_PIN | I2C1_SCL_PIN;
    tGpio.Mode      = GPIO_MODE_AF_OD;
    tGpio.Pull      = GPIO_PULLUP;
    tGpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    tGpio.Alternate = I2C1_GPIO_AF;
    HAL_GPIO_Init(I2C1_SDA_PORT, &tGpio);

    s_hi2c1.Instance              = I2C1;
    s_hi2c1.Init.Timing           = I2C1_TIMING_400K;
    s_hi2c1.Init.OwnAddress1      = 0;
    s_hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    s_hi2c1.Init.OwnAddress2      = 0;
    s_hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    s_hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    s_hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) {
        while (1);
    }

    HAL_I2CEx_ConfigAnalogFilter(&s_hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&s_hi2c1, 0);
}

int32_t hali2c_Write(uint8_t chDevAddr, const uint8_t *pchData, uint16_t hwLen)
{
    if (HAL_I2C_Master_Transmit(&s_hi2c1, chDevAddr,
                                (uint8_t *)pchData, hwLen,
                                HALI2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    return (int32_t)hwLen;
}

int32_t hali2c_Read(uint8_t chDevAddr, uint8_t *pchBuf, uint16_t hwLen)
{
    if (HAL_I2C_Master_Receive(&s_hi2c1, chDevAddr,
                               pchBuf, hwLen,
                               HALI2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    return (int32_t)hwLen;
}

bool hali2c_IsBusy(void)
{
    return HAL_I2C_GetState(&s_hi2c1) != HAL_I2C_STATE_READY;
}
