/**
 * @file   ch592_it.c
 * @brief  中断服务程序 (CH592F)
 *         SysTick 驱动 MODUS 框架时钟 + perf_counter 64 位溢出计数
 */

#include <stdint.h>
#include <stdbool.h>
#include "CH592SFR.h"
#include "core_riscv.h"
#include "perf_counter.h"
#include "port_mdi.h"

/* 由 perfc_port.c (RISC-V 移植) 导出 */
extern volatile uint64_t g_perfc_systick_overflow;

/* 由 main.c 导出 */
extern void modus_Clock(void);

/* 由 port_mdi.c 导出的串口实例 */
extern ch592_usart_priv_t s_tUsart1Priv;

/**
 * @brief SysTick 中断: 每 1ms 触发
 *
 * 1. 递增 g_perfc_systick_overflow (perf_counter 时间基准)
 * 2. 驱动串口 1ms 定时器状态机，实现自动分帧
 * 3. modus_Init 完成后调用 modus_Clock()
 */
__attribute__((interrupt))
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    g_perfc_systick_overflow += 60000;  /* 60MHz * 1ms = 60000 ticks */

    ch592_usart_timer_1ms(&s_tUsart1Priv);

    modus_Clock();
}

/**
 * @brief UART1 硬件中断接收/发送服务程序
 */
__attribute__((interrupt))
void UART1_IRQHandler(void)
{
    ch592_usart_irq_handler(&s_tUsart1Priv);
}
