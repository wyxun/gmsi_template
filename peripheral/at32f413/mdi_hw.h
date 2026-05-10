/**
 * @file mdi_hw.h
 * @brief Global peripheral resource definition (MDI hardware pool) — AT32F413 Motor EVB V1
 *
 * Provides a unified hardware structure for the application layer,
 * avoiding exposure of chip-specific headers.
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/mdi.h"

/*============================================================================
 * Project hardware resource pool
 *===========================================================================*/

typedef struct {
    /* ---------- LEDs ---------- */
    mdi_gpio_t   *ptLedStatus;    /**< PC13, main status LED (heartbeat) */
    mdi_gpio_t   *ptLedError;     /**< PB9,  error LED */
    mdi_gpio_t   *ptLedStatus2;   /**< PC14, status LED 2 */
    mdi_gpio_t   *ptLedStatus3;   /**< PC15, status LED 3 */

    /* ---------- PWM (motor) ---------- */
    mdi_pwm_t    *ptMotorU;       /**< TMR1 CH1+CH1N  — PA8/PB13  U phase */
    mdi_pwm_t    *ptMotorV;       /**< TMR1 CH2+CH2N  — PA9/PB14  V phase */
    mdi_pwm_t    *ptMotorW;       /**< TMR1 CH3+CH3N  — PA10/PB15 W phase */

    /* ---------- ADC ---------- */
    mdi_adc_t    *ptAdcCurrU;     /**< ADC1 CH0  — PA0  Phase A current */
    mdi_adc_t    *ptAdcCurrV;     /**< ADC1 CH1  — PA1  Phase B current */
    mdi_adc_t    *ptAdcCurrW;     /**< ADC1 CH2  — PA2  Phase C current */
    mdi_adc_t    *ptAdcBusV;      /**< ADC1 CH7  — PA7  DC bus voltage */
    mdi_adc_t    *ptAdcTemp;      /**< ADC1 CH9  — PB1  MOS temperature */

    /* ---------- Comparator (overcurrent break) ---------- */
    mdi_gpio_t   *ptCompBrk;      /**< PB12 — TMR1 brake input */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< USART1 (PB6/7 remapped) */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
