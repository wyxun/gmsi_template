#include "peripheral.h"
#include "at32f421.h"

/**
 * @brief 内部调用：AT32F421 系统时钟配置
 *
 *  AT32F421 内置 HICK 支持两种工作频率：
 *    - 默认 8 MHz（复位后）
 *    - 48 MHz（通过 misc2.hick_to_sclk=1 + misc1.hickdiv=1 使能）
 */
static void SystemClock_Config(void)
{
    /* 1. 等待 HICK 稳定 */
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) == RESET);

    /* 2. Flash 等待周期：48 MHz 需 ≥1WS（使用宏 flash_psr_set） */
    flash_psr_set(FLASH_WAIT_CYCLE_1);

    /* 3. 使能 HICK 48 MHz 模式：
     *    misc2.hick_to_sclk = 1  → 允许 HICK 以 48 MHz 供给系统时钟
     *    misc1.hickdiv       = 1  → HICK 48 MHz 不分频（NODIV）
     */
    CRM->misc2_bit.hick_to_sclk = TRUE;
    CRM->misc1_bit.hickdiv       = TRUE;

    /* 4. AHB/APB 总线不分频 */
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);

    /* 5. 切换系统时钟 → HICK */
    crm_sysclk_switch(CRM_SCLK_HICK);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_HICK);

    /* 6. 更新全局时钟变量 */
    system_core_clock_update();
}

/**
 * @brief 系统级别的底层外设初始化总入口 (实现)
 *
 * 包装了所有针对当前平台（AT32F421）的初始化代码，将 main 函数与底层解耦。
 */
void peripheral_Init(void)
{
    /* 1. 初始化系统时钟 HICK 48 MHz */
    SystemClock_Config();

    /* 2. 在这里添加其他底层外设的时钟、GPIO、中断初始化等 */
    /* TODO: gpio_init(), usart_init() ... */
}

/**
 * @brief 抽象的系统时钟获取函数
 */
uint32_t get_system_core_clock_hz(void)
{
    return system_core_clock; /* 返回系统固件库全局变量 */
}
