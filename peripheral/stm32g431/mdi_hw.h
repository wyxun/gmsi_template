/**
 * @file MDI_hw.h
 * @brief 鍏ㄥ眬澶栬璧勬簮缁熶竴瀹氫箟 (GDI 纭欢姹犲ご鏂囦欢) 鈥?STM32G431
 *
 * 涓哄簲鐢ㄥ眰鎻愪緵缁熶竴纭欢缁撴瀯瀹氫箟锛岄伩鍏嶅簲鐢ㄥ眰鏆撮湶鑺墖绉佹湁澶存枃浠躲€?
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/mdi.h"

/*============================================================================
 * 椤圭洰纭欢璧勬簮姹犲畾涔?
 *===========================================================================*/

typedef struct {
    /* ---------- LED / Status ---------- */
    mdi_gpio_t   *ptLedStatus;    /**< PC6, active-low status LED */

    /* ---------- Comparators (overcurrent) ---------- */
    mdi_gpio_t   *ptCompU;        /**< COMP1 鈥?U-phase overcurrent */
    mdi_gpio_t   *ptCompV;        /**< COMP2 鈥?V-phase overcurrent */
    mdi_gpio_t   *ptCompW;        /**< COMP4 鈥?W-phase overcurrent */

    /* ---------- ADC ---------- */
    mdi_adc_t    *ptAdcBusV;      /**< DC bus voltage */
    mdi_adc_t    *ptAdcTemp;      /**< Temperature sensor */
    mdi_adc_t    *ptAdcPot;       /**< Potentiometer */

    /* ---------- PWM (motor) ---------- */
    mdi_pwm_t    *ptMotorU;       /**< TIM1_CH1 + CH1N 鈥?U phase */
    mdi_pwm_t    *ptMotorV;       /**< TIM1_CH2 + CH2N 鈥?V phase */
    mdi_pwm_t    *ptMotorW;       /**< TIM1_CH3 + CH3N 鈥?W phase */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< USART2 debug serial */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
