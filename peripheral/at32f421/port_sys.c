#include "peripheral.h"
#include "at32f421.h"
#include "at32f421_usart.h"

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

static void led_gpio_init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);

    /* PF7 — LED 指示 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins      = GPIO_PINS_7;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOF, &gpio_init_struct);
}
/**
 * @brief USART1 初始化
 */
static void usart1_init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);

    /* USART1 TX (PA9) */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins      = GPIO_PINS_9;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* USART1 RX (PA10) */
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);

    nvic_irq_enable(USART1_IRQn, 1, 2);

    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

    /* The reference setup: */
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);

    usart_flag_clear(USART1, USART_TDBE_FLAG);
    usart_flag_clear(USART1, USART_TDC_FLAG);
    usart_flag_clear(USART1, USART_RDBF_FLAG);
    usart_interrupt_enable(USART1, USART_TDC_INT, TRUE);

    extern void at32_usart1_init(void);
    at32_usart1_init();
}

/**
 * @brief 系统级别的底层外设初始化总入口 (实现)
 *
 * 包装了所有针对当前平台（AT32F421）的初始化代码，将 main 函数与底层解耦。
 */
void peripheral_Init(void)
{
    /* 中断优先级分组：4位抢占优先级，0位响应优先级 */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    /* 1. 初始化系统时钟 HICK 48 MHz */
    SystemClock_Config();

    /* 2. debug led 初始化 */
    led_gpio_init();

    /* 3. USART1 调试串口初始化 */
    usart1_init();

    /* 3. 启动 SysTick 1ms 中断 (48MHz / 1000 = 48000) */
    SysTick_Config(SystemCoreClock / 1000U);
}

/**
 * @brief 抽象的系统时钟获取函数
 */
uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock; /* 返回系统时钟（AT32: #define 到 system_core_clock） */
}
