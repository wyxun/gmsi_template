/**
 * @file   ch592_it.c
 * @brief  中断服务程序存根 (CH592F)
 *         提供基本的SysTick时钟驱动与外设中断绑定。
 */

#include <stdint.h>
#include <stdbool.h>
#include "CH592SFR.h"
#include "core_riscv.h"
#include "perf_counter.h"

/* 由 main.c 导出 */
extern void modus_Clock(void);

/**
 * @brief RISC-V QingKe V4C 内核 System Timer 中断
 *        驱动 MODUS 框架时钟，无需调用 perf_counter 溢出处理（已基于64位全局Cycle）
 */
__attribute__((interrupt))
void SysTick_Handler(void)
{
    /* 直接累加 1ms 比较值 (60MHz 对应 60000 counts) */
    SysTick->CMP += 60000UL;

    /* 清除中断标志 */
    SysTick->SR = 0; 
    modus_Clock();
}

/* 其它外设中断存根，采用 weak 弱符号定义在汇编启动文件中，此处可按需覆写 */
