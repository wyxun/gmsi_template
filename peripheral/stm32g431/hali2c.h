/**
 * @file  hali2c.h
 * @brief I2C1 master driver — PB7(SDA) / PB8(SCL), AF4, blocking
 */

#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* I2C1: PB7(SDA) / PB8(SCL) — AF4 */
#define I2C1_SDA_PORT           GPIOB
#define I2C1_SDA_PIN            GPIO_PIN_7
#define I2C1_SCL_PORT           GPIOB
#define I2C1_SCL_PIN            GPIO_PIN_8
#define I2C1_GPIO_AF            GPIO_AF4_I2C1
#define I2C1_GPIO_CLK_EN()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C1_CLK_EN()           __HAL_RCC_I2C1_CLK_ENABLE()

/* I2C_TIMINGR for 400 kHz Fast-mode @ PCLK1 = 170 MHz:
 *   PRESC=3  -> t_presc = 4 / 170 MHz = 23.53 ns
 *   SCLL=0x38 -> t_LOW  = 57 * 23.53 ns = 1.34 us (>= 1.3 us, FM spec)
 *   SCLH=0x2B -> t_HIGH = 44 * 23.53 ns = 1.04 us (>= 0.6 us, FM spec)
 *   SCLDEL=0xA, SDADEL=0
 *   SCL period ~ 2.5 us (400 kHz), sync overhead included.
 *   NOTE: verify the real SCL frequency with a scope on first bring-up. */
#define I2C1_TIMING_400K        0x30A02B38UL

#define HALI2C_TIMEOUT_MS       10U

/**
 * @brief  初始化 I2C1（PB7 SDA / PB8 SCL，400 kHz，主机模式）
 */
extern void    hali2c_Init(void);
/**
 * @brief  阻塞写（HAL 8 位地址格式，即 7 位地址 << 1）
 * @return 成功返回写入字节数，失败返回 -1
 */
extern int32_t hali2c_Write(uint8_t chDevAddr,
                            const uint8_t *pchData, uint16_t hwLen);
/**
 * @brief  阻塞读（HAL 8 位地址格式，即 7 位地址 << 1）
 * @return 成功返回读取字节数，失败返回 -1
 */
extern int32_t hali2c_Read(uint8_t chDevAddr,
                           uint8_t *pchBuf, uint16_t hwLen);
/**
 * @brief  总线忙状态查询
 */
extern bool    hali2c_IsBusy(void);

#endif /* __HAL_I2C_H__ */
