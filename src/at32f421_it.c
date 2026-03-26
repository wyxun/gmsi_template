/**
 * @file   at32f421_it.c
 * @brief  中断服务程序存根 (AT32F421)
 *
 * SysTick_Handler 是核心定时中断，驱动 perf_counter、gmsi_Clock 等。
 * 其余中断在此存根实现，按需展开。
 */

#include "at32f421.h"
#include "perf_counter.h"

/* 由 main.c 导出 */
extern void gmsi_Clock(void);
extern void peripheral_Clock(void);

/* 初始化完成标志（在 main.c 中定义） */
extern volatile uint8_t s_bInitDone;

/* -------------------------------------------------------------------------- */
/*  Cortex-M4 Core Exceptions                                                 */
/* -------------------------------------------------------------------------- */

void NMI_Handler(void)              { while(1); }
void HardFault_Handler(void)        { while(1); }
void MemManage_Handler(void)        { while(1); }
void BusFault_Handler(void)         { while(1); }
void UsageFault_Handler(void)       { while(1); }
void SVC_Handler(void)              {}
void DebugMon_Handler(void)         {}
void PendSV_Handler(void)           {}

/**
 * @brief SysTick 1ms 定时中断
 *        驱动 perf_counter tick 及 GMSI 框架时钟
 */
void SysTick_Handler(void)
{
    /* perf_counter 内部溢出处理（必须第一个调用） */
    perfc_port_insert_to_system_timer_insert_ovf_handler();

    if (s_bInitDone) {
        gmsi_Clock();
        /* peripheral_Clock(); */   /* 如有外设时钟驱动需求，取消注释 */
    }
}

/* -------------------------------------------------------------------------- */
/*  AT32F421 Peripheral Interrupts (存根，按需实现)                           */
/* -------------------------------------------------------------------------- */

void WWDT_IRQHandler(void)                  {}
void PVM_IRQHandler(void)                   {}
void ERTC_IRQHandler(void)                  {}
void FLASH_IRQHandler(void)                 {}
void CRM_IRQHandler(void)                   {}
void EXINT1_0_IRQHandler(void)              {}
void EXINT3_2_IRQHandler(void)              {}
void EXINT15_4_IRQHandler(void)             {}
void DMA1_Channel1_IRQHandler(void)         {}
void DMA1_Channel3_2_IRQHandler(void)       {}
void DMA1_Channel5_4_IRQHandler(void)       {}
void ADC1_CMP_IRQHandler(void)              {}
void TMR1_BRK_OVF_TRG_HALL_IRQHandler(void){}
void TMR1_CH_IRQHandler(void)               {}
void TMR3_GLOBAL_IRQHandler(void)           {}
void TMR6_GLOBAL_IRQHandler(void)           {}
void TMR14_GLOBAL_IRQHandler(void)          {}
void TMR15_GLOBAL_IRQHandler(void)          {}
void TMR16_GLOBAL_IRQHandler(void)          {}
void TMR17_GLOBAL_IRQHandler(void)          {}
void I2C1_EVT_IRQHandler(void)              {}
void I2C2_EVT_IRQHandler(void)              {}
void SPI1_IRQHandler(void)                  {}
void SPI2_IRQHandler(void)                  {}
void USART1_IRQHandler(void)                {}
void USART2_IRQHandler(void)                {}
void I2C1_ERR_IRQHandler(void)              {}
void I2C2_ERR_IRQHandler(void)              {}
