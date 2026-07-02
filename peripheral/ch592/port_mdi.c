#include "mdi_hw.h"
#include "CH59x_common.h"
#include <stdint.h>
#include <stdbool.h>

/* GPIO LED (PA8) 对象驱动 */
static int32_t ch592_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    if (eLevel == MDI_GPIO_HIGH) {
        GPIOA_SetBits(GPIO_Pin_8);   /* 灭 */
    } else {
        GPIOA_ResetBits(GPIO_Pin_8); /* 亮 */
    }
    return 0;
}

static int32_t ch592_gpio_Get(void *pPriv)
{
    return (R32_PA_PIN & (1 << 8)) ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
}

static int32_t ch592_gpio_Toggle(void *pPriv)
{
    GPIOA_InverseBits(GPIO_Pin_8);
    return 0;
}

static mdi_gpio_t s_tLedGpio = {
    .pPriv    = NULL,
    .fnSet    = ch592_gpio_Set,
    .fnGet    = ch592_gpio_Get,
    .fnToggle = ch592_gpio_Toggle,
};

/* UART0 流传输对象驱动 */
static int32_t ch592_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    if (!pchData) return 0;
    for (uint32_t i = 0; i < wLen; i++) {
        while (!(R8_UART0_LSR & RB_LSR_TX_FIFO_EMP));
        UART0_SendByte(pchData[i]);
    }
    return wLen;
}

static int32_t ch592_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    if (!pchBuf) return 0;
    uint32_t i = 0;
    while (i < wLen) {
        if (R8_UART0_LSR & RB_LSR_DATA_RDY) {
            pchBuf[i++] = UART0_RecvByte();
        } else {
            break;
        }
    }
    return i;
}

static int32_t ch592_stream_IsBusy(void *pPriv)
{
    return (R8_UART0_LSR & RB_LSR_TX_ALL_EMP) ? 0 : 1;
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = NULL,
    .fnWrite  = ch592_stream_Write,
    .fnRead   = ch592_stream_Read,
    .fnIsBusy = ch592_stream_IsBusy,
};

/* 静态注册 */
const mdi_hardware_t HW = {
    .ptLedStatus   = &s_tLedGpio,
    .ptSerial      = &s_tStreamSerial,
};
