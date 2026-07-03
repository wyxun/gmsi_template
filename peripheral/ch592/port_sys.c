#include "peripheral.h"
#include <stdint.h>
#include <stdbool.h>
#include "CH59x_common.h"
#include "port_mdi.h"

/* 系统内核时钟强符号回传，完美覆盖 perf_counter 中的弱符号 */
uint32_t SystemCoreClock = 60000000UL;

uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}

/* 系统时钟配置 — 使用 WCH vendor SetSysClock + SysTick_Config */
static void SystemClock_Config(void)
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);
}

/* 官方中断向量表默认调用的时钟/SysTick初始化入口 */
void SystemInit(void)
{
    SystemClock_Config();

    /* SysTick_Config 内部完成: CMP 设置 + PFIC_EnableIRQ + CTLR 启动
     *   60MHz × 1ms = 60000 ticks
     *   依赖 SetSysClock 之后 GetSysClock() 返回正确的 60MHz */
    SysTick_Config(60000);
}

/* 外设初始化总入口 */
void peripheral_Init(void)
{
    SystemInit();

    /* 1. 开启 UART0 外设时钟 (清零表示使能时钟) */
    R8_SLP_CLK_OFF0 &= ~RB_SLP_CLK_UART0;

    /* 2. 板载 GPIO LED (PA8) 初始化 */
    R32_PA_DIR |= (1 << 8);
    R32_PA_OUT |= (1 << 8); /* 默认灭 (低电平亮) */

    /* 3. 一站式初始化串口 0 硬件、RingBuffer 与中断 */
    ch592_usart_init(115200);
}

void peripheral_Clock(void) {}

void peripheral_EnableIRQ(void)
{
    uint32_t mask = 8;
    __asm__ __volatile__("csrrs zero, mstatus, %0" :: "r"(mask));
}

void peripheral_DisableIRQ(void)
{
    uint32_t mask = 8;
    __asm__ __volatile__("csrrc zero, mstatus, %0" :: "r"(mask));
}
