/**
 * @file   port_MDI.c
 * @brief  STM32G431 MDI 缁旑垰褰?閳?绾兛娆㈤幎鍊熻杽鐎圭偘绶ラ崠?
 */

#include "mdi_hw.h"
#include "stm32g4xx_hal.h"
#include "port_mdi.h"
#include "halusart.h"
#include "haltim1.h"
#include "haladc.h"
#include "halledgpio.h"
#include "stm32g4xx_ll_tim.h"
#include "mdi/mdi.h"

/* --------------------------------------------------------------------------
 *  MDI GPIO wrappers
 * -------------------------------------------------------------------------- */

static int32_t gpiog_set(void *pPriv, mdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    /* Active-low logic: HIGH -> ON -> RESET(LOW) */
    HAL_GPIO_WritePin((GPIO_TypeDef *)ap[0],
                      (uint16_t)(uintptr_t)ap[1],
                      (eLevel == MDI_GPIO_HIGH) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return 0;
}

static int32_t gpiog_get(void *pPriv)
{
    void **ap = (void **)pPriv;
    GPIO_PinState state = HAL_GPIO_ReadPin((GPIO_TypeDef *)ap[0], (uint16_t)(uintptr_t)ap[1]);
    /* Active-low logic: RESET(LOW) -> HIGH(ON) */
    return (state == GPIO_PIN_RESET) ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
}

static int32_t gpiog_toggle(void *pPriv)
{
    void **ap = (void **)pPriv;
    HAL_GPIO_TogglePin((GPIO_TypeDef *)ap[0], (uint16_t)(uintptr_t)ap[1]);
    return 0;
}

/* LED 閳?PC6 */
static void *s_apvLedPriv[] = { GPIOC, (void *)(uintptr_t)GPIO_PIN_6 };
static mdi_gpio_t s_tGpioLed = {
    .pPriv   = s_apvLedPriv,
    .fnSet   = gpiog_set,
    .fnGet   = gpiog_get,
    .fnToggle = gpiog_toggle,
};

/* COMP1 閳?PA1 (input only) */
static void *s_apvComp1Priv[] = { GPIOA, (void *)(uintptr_t)GPIO_PIN_1 };
static mdi_gpio_t s_tGpioComp1 = {
    .pPriv   = s_apvComp1Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* COMP2 閳?PA7 (input only) */
static void *s_apvComp2Priv[] = { GPIOA, (void *)(uintptr_t)GPIO_PIN_7 };
static mdi_gpio_t s_tGpioComp2 = {
    .pPriv   = s_apvComp2Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* COMP4 閳?PB0 (input only) */
static void *s_apvComp4Priv[] = { GPIOB, (void *)(uintptr_t)GPIO_PIN_0 };
static mdi_gpio_t s_tGpioComp4 = {
    .pPriv   = s_apvComp4Priv,
    .fnSet   = NULL,
    .fnGet   = gpiog_get,
};

/* --------------------------------------------------------------------------
 *  MDI ADC wrappers
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

static mdi_adc_t s_tAdcBusV = { .pPriv = (void *)HALADC_REG_BUS_VOLTAGE, .fnRead = adc_read };
static mdi_adc_t s_tAdcTemp = { .pPriv = (void *)HALADC_REG_TEMPERATURE, .fnRead = adc_read };
static mdi_adc_t s_tAdcPot  = { .pPriv = (void *)HALADC_REG_POTENTIOMETER, .fnRead = adc_read };

/* --------------------------------------------------------------------------
 *  MDI PWM wrappers 閳?TIM1 motor phases via haltim1_SetDuty
 * -------------------------------------------------------------------------- */

static int32_t pwm_setduty(void *pPriv, uint32_t wDuty)
{
    uint32_t wChannel = (uint32_t)(uintptr_t)pPriv;
    if (wChannel == 1U)      LL_TIM_OC_SetCompareCH1(TIM1, wDuty);
    else if (wChannel == 2U) LL_TIM_OC_SetCompareCH2(TIM1, wDuty);
    else if (wChannel == 3U) LL_TIM_OC_SetCompareCH3(TIM1, wDuty);
    return 0;
}

static int32_t pwm_enable(void *pPriv, bool bEn)
{
    (void)pPriv;
    if (bEn) haltim1_Start();
    else     haltim1_Stop();
    return 0;
}

static mdi_pwm_t s_tPwmU = { .pPriv = (void *)1U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static mdi_pwm_t s_tPwmV = { .pPriv = (void *)2U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };
static mdi_pwm_t s_tPwmW = { .pPriv = (void *)3U, .fnSetDuty = pwm_setduty, .fnEnable = pwm_enable };

/* --------------------------------------------------------------------------
 *  MDI Stream 閳?USART2 debug serial
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

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = NULL,
    .fnWrite  = uart_write,
    .fnRead   = uart_read,
    .fnIsBusy = uart_isbusy,
};

/* --------------------------------------------------------------------------
 *  閸忋劌鐪涵顑挎鐠у嫭绨Ч?
 * -------------------------------------------------------------------------- */

const mdi_hardware_t HW = {
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
