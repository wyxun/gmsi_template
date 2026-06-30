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

/* 由 perfc_port.c (RISC-V 移植) 导出 */
extern volatile uint64_t g_perfc_systick_overflow;

/* 由 main.c 导出 */
extern void modus_Clock(void);
extern volatile uint8_t s_bInitDone;

/**
 * @brief SysTick 中断: 每 1ms 触发
 *
 * 1. 递增 g_perfc_systick_overflow (perf_counter 时间基准)
 * 2. modus_Init 完成后调用 modus_Clock()
 */
__attribute__((interrupt))
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    g_perfc_systick_overflow += 60000;  /* 60MHz * 1ms = 60000 ticks */

    if (s_bInitDone) {
        modus_Clock();
    }
}
