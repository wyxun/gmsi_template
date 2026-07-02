/**
 * @file peripheral.h
 * @brief 硬件抽象层总入口
 *
 * 该文件提供应用层需要的底层硬件初始化接口和系统资源获取接口。
 */

#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * API 声明
 *===========================================================================*/

/**
 * @brief 平台底层外设初始化总入口
 */
void peripheral_Init(void);

/**
 * @brief 外设 1ms 时钟节拍（SysTick_Handler 中调用）
 */
void peripheral_Clock(void);

/**
 * @brief 获取当前的系统内核时钟频率 (Hz)
 */
uint32_t get_system_core_clock_hz(void);

/**
 * @brief 全局开启中断
 */
void peripheral_EnableIRQ(void);

/**
 * @brief 全局关闭中断
 */
void peripheral_DisableIRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_H__ */
