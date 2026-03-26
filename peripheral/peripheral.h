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
#include "gdi_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * API 声明
 *===========================================================================*/

/**
 * @brief 平台底层外设初始化总入口
 * 
 * 必须在 main() 的开头被调用。包括了系统时钟（如 HICK 48MHz 等）、
 * 各种硬件 IO、外设时钟使能、中断以及 GDI 硬件对象池实例化过程等所有底层操作。
 */
void peripheral_Init(void);

/**
 * @brief 获取当前的系统内核时钟频率 (Hz)
 * 
 * 抽象了对裸机 `system_core_clock` 或 `SystemCoreClock` 全局变量的底层直接访问。
 * 供 `perfc_init` 等计算时钟 Tick 的中间件使用。
 * 
 * @return uint32_t 当前的系统频率，单位 Hz (如 48000000)
 */
uint32_t get_system_core_clock_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_H__ */
