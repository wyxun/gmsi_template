#include "gdi_hw.h"
#include "at32f421.h"

/*============================================================================
 * AT32F421 GPIO 适配 (目前作为示范)
 *===========================================================================*/

static int32_t at32_gpio_Set(void *pPriv, gdi_gpio_level_t eLevel)
{
    /* (void)pPriv;
     * TODO: gpio_bits_write((gpio_type *)pPriv->port, pPriv->pin, (confirm_state)eLevel);
     */
    return 0;
}

static int32_t at32_gpio_Get(void *pPriv)
{
    return GDI_GPIO_LOW;
}

static int32_t at32_gpio_Toggle(void *pPriv)
{
    return 0;
}

/*============================================================================
 * 外设实例（静态分配）
 * 这里将具体的引脚、寄存器通过 pPriv 与操作函数绑定
 *===========================================================================*/

static gdi_gpio_t s_tLedGpio = {
    .pPriv    = NULL, /* 未来可以填入如 GPIOB 及其 Pin 号等私有结构体指针 */
    .fnSet    = at32_gpio_Set,
    .fnGet    = at32_gpio_Get,
    .fnToggle = at32_gpio_Toggle,
};

/*============================================================================
 * 全局硬件资源池实例化
 *===========================================================================*/

const gdi_hardware_t HW = {
    .ptLedStatus   = &s_tLedGpio,
};
