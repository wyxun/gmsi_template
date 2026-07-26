/**
 * @file   mdi_hw.h
 * @brief  全局外设资源统一定义（MDI 硬件池头文件）— STM32G431
 *
 * 为应用层提供统一硬件结构体定义，避免应用层暴露芯片私有头文件。
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/mdi.h"

/* 本芯片 MDI 硬件池携带 I2C 编码器（AS5600, I2C1 PB7/PB8） */
#define MDI_HW_HAS_I2C_ENCODER   1

/*============================================================================
 * 项目硬件资源池定义
 *===========================================================================*/

typedef struct {
    /* ---------- LED / Status ---------- */
    mdi_gpio_t   *ptLedStatus;    /**< PC6, active-low status LED */

    /* ---------- Comparators (overcurrent) ---------- */
    mdi_gpio_t   *ptCompU;        /**< COMP1 — U-phase overcurrent */
    mdi_gpio_t   *ptCompV;        /**< COMP2 — V-phase overcurrent */
    mdi_gpio_t   *ptCompW;        /**< COMP4 — W-phase overcurrent */

    /* ---------- ADC ---------- */
    mdi_adc_t    *ptAdcBusV;      /**< DC bus voltage */
    mdi_adc_t    *ptAdcTemp;      /**< Temperature sensor */
    mdi_adc_t    *ptAdcPot;       /**< Potentiometer */

    /* ---------- PWM (motor) ---------- */
    mdi_pwm_t    *ptMotorU;       /**< TIM1_CH1 + CH1N — U phase */
    mdi_pwm_t    *ptMotorV;       /**< TIM1_CH2 + CH2N — V phase */
    mdi_pwm_t    *ptMotorW;       /**< TIM1_CH3 + CH3N — W phase */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< USART2 debug serial */

    /* ---------- IIC ---------- */
    mdi_iic_t    *ptI2c1;         /**< I2C1 — AS5600 encoder (0x36, PB7/PB8) */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
