/**
 * @file gdi_hw.h
 * @brief 全局外设资源统一定义 (GDI 硬件池头文件)
 *
 * 为应用层提供统一硬件结构定义，避免应用层暴露芯片私有头文件。
 */

#ifndef __GDI_HW_H__
#define __GDI_HW_H__

#include "gdi/gdi.h"

/*============================================================================
 * 项目硬件资源池定义
 *
 * 结构体内声明项目所有的使用到的外设抽象对象
 *===========================================================================*/

typedef struct {
    /* ---------- GPIO ---------- */
    gdi_gpio_t   *ptLedStatus;      /**< 状态指示 LED */

    /* ---------- Stream ---------- */
    gdi_stream_t *ptSerial;       /**< RS232 串口 (PA9/10) */
    
} gdi_hardware_t;

/**
 * @brief 全局统一的硬件资源实例
 * 
 * 真正的实例化发生在外设适配层（如 peripheral/AT32/port_gdi.c 中）
 */
extern const gdi_hardware_t HW;

#endif /* __GDI_HW_H__ */
