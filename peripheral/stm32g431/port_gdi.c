/**
 * @file   port_gdi.c
 * @brief  STM32G431 GDI 端口 — 硬件抽象实例化
 */

#include "gdi_hw.h"
#include "stm32g4xx_hal.h"
#include "port_gdi.h"
#include "halusart.h"
#include "haltim1.h"
#include "haladc.h"
#include "halcomp.h"
#include "halledgpio.h"
#include "gdi/gdi.h"

/* --------------------------------------------------------------------------
 *  GDI GPIO wrappers
 * -------------------------------------------------------------------------- */

static int32_t gpiog_set(void *pPriv, gdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    HAL_GPIO_WritePin((GPIO_TypeDef *)ap[0],
                      (uint16_t)(uintptr_t)ap[1],
                      (eLevel == GDI_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return 0;
}

static int32_t gpiog_get(void *pPriv)
{
    void **ap = (void **)pPriv;
    return (int32_t)HAL_GPIO_ReadPin((GPIO_TypeDef *)ap[0],
                                     (uint16_t)(uintptr_t)ap[1]);
}

/* LED — PC6 */
static void *s_apvLedPriv[] = { GPIOC, (void *)(uintptr_t)GPIO_PIN_6 };
static gdi_gpio_t s_tGpioLed = {
    .pPriv   = s_apvLedPriv,
    .fnSet   = gpiog_set,
    .fnGet   = NULL,
};

/* COMP1 — PA1 (input only) */
static void *s_apvComp1Priv[] = { GPIOA, (void *)(uintptr_t)GPIO_PIN_1 };
static gdi_gpio_t s_tGpioComp1 = {
    .pPriv   = s_apvComp1Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* COMP2 — PA7 (input only) */
static void *s_apvComp2Priv[] = { GPIOA, (void *)(uintptr_t)GPIO_PIN_7 };
static gdi_gpio_t s_tGpioComp2 = {
    .pPriv   = s_apvComp2Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* COMP4 — PB0 (input only) */
static void *s_apvComp4Priv[] = { GPIOB, (void *)(uintptr_t)GPIO_PIN_0 };
static gdi_gpio_t s_tGpioComp4 = {
    .pPriv   = s_apvComp4Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* --------------------------------------------------------------------------
 *  GDI ADC wrappers
 * -------------------------------------------------------------------------- */

static int32_t adc_read(void *pPriv)
{
    uint32_t wChannel = (uint32_t)(uintptr_t)pPriv;

    switch (wChannel) {
    case HALADC_REG_BUS_VOLTAGE:
    case HALADC_REG_TEMPERATURE:
    case HALADC_REG_POTENTIOMETER:
        haladc_StartRegular();
        return (int32_t)haladc_GetRegular(wChannel);
    default:
        return 0;
    }
}

static gdi_adc_t s_tAdcBusV = { .pPriv = (void *)HALADC_REG_BUS_VOLTAGE, .fnRead = adc_read };
static gdi_adc_t s_tAdcTemp = { .pPriv = (void *)HALADC_REG_TEMPERATURE, .fnRead = adc_read };
static gdi_adc_t s_tAdcPot  = { .pPriv = (void *)HALADC_REG_POTENTIOMETER, .fnRead = adc_read };

/* --------------------------------------------------------------------------
 *  GDI PWM wrappers — TIM1 motor phases via haltim1_SetDuty
 * -------------------------------------------------------------------------- */

static int32_t pwm_setduty(void *pPriv, uint32_t wDuty)
{
    (void)pPriv;
    (void)wDuty;
    /* Single-phase set not used; use haltim1_SetDuty for 3-phase */
    return (int32_t)wDuty;
}

static int32_t pwm_enable(void *pPriv, bool bEn)
{
    (void)pPriv;
    if (bEn) haltim1_Start();
    else     haltim1_Stop();
    return 0;
}

static gdi_pwm_t s_tPwmU = { .pPriv = NULL, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static gdi_pwm_t s_tPwmV = { .pPriv = NULL, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static gdi_pwm_t s_tPwmW = { .pPriv = NULL, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };

/* --------------------------------------------------------------------------
 *  GDI Stream — USART2 debug serial
 * -------------------------------------------------------------------------- */

static int32_t uart_write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    (void)pPriv;
    return (int32_t)halusart_SendData(1, (uint8_t *)pchData, (uint16_t)wLen);
}

static int32_t uart_read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    (void)pPriv;
    (void)wLen;
    uint16_t hwRead = halusart_receiveData(1, pchBuf);
    return (hwRead > 0) ? (int32_t)hwRead : -1;
}

static int32_t uart_isbusy(void *pPriv)
{
    (void)pPriv;
    return 0;
}

static gdi_stream_t s_tStreamSerial = {
    .pPriv    = NULL,
    .fnWrite  = uart_write,
    .fnRead   = uart_read,
    .fnIsBusy = uart_isbusy,
};

/* --------------------------------------------------------------------------
 *  全局硬件资源池
 * -------------------------------------------------------------------------- */

const gdi_hardware_t HW = {
    .ptLedStatus  = &s_tGpioLed,
    .ptCompU      = &s_tGpioComp1,
    .ptCompV      = &s_tGpioComp2,
    .ptCompW      = &s_tGpioComp4,
    .ptAdcBusV    = &s_tAdcBusV,
    .ptAdcTemp    = &s_tAdcTemp,
    .ptAdcPot     = &s_tAdcPot,
    .ptMotorU     = &s_tPwmU,
    .ptMotorV     = &s_tPwmV,
    .ptMotorW     = &s_tPwmW,
    .ptSerial       = &s_tStreamSerial,
};
