/**
 * @file gdi_hw.h
 * @brief 全局外设资源统一定义 (GDI 硬件池头文件) — STM32G431
 *
 * 为应用层提供统一硬件结构定义，避免应用层暴露芯片私有头文件。
 */

#ifndef __GDI_HW_H__
#define __GDI_HW_H__

#include "gdi/gdi.h"

/*============================================================================
 * 项目硬件资源池定义
 *===========================================================================*/

typedef struct {
    /* ---------- LED / Status ---------- */
    gdi_gpio_t   *ptLedStatus;    /**< PC6, active-low status LED */

    /* ---------- Comparators (overcurrent) ---------- */
    gdi_gpio_t   *ptCompU;        /**< COMP1 — U-phase overcurrent */
    gdi_gpio_t   *ptCompV;        /**< COMP2 — V-phase overcurrent */
    gdi_gpio_t   *ptCompW;        /**< COMP4 — W-phase overcurrent */

    /* ---------- ADC ---------- */
    gdi_adc_t    *ptAdcBusV;      /**< DC bus voltage */
    gdi_adc_t    *ptAdcTemp;      /**< Temperature sensor */
    gdi_adc_t    *ptAdcPot;       /**< Potentiometer */

    /* ---------- PWM (motor) ---------- */
    gdi_pwm_t    *ptMotorU;       /**< TIM1_CH1 + CH1N — U phase */
    gdi_pwm_t    *ptMotorV;       /**< TIM1_CH2 + CH2N — V phase */
    gdi_pwm_t    *ptMotorW;       /**< TIM1_CH3 + CH3N — W phase */

    /* ---------- Stream ---------- */
    gdi_stream_t *ptSerial;       /**< USART2 debug serial */

} gdi_hardware_t;

extern const gdi_hardware_t HW;

#endif /* __GDI_HW_H__ */
