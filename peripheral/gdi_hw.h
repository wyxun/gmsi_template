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
    gdi_gpio_t   *ptPump;         /**< 真空泵驱动控制 (PA2) */
    gdi_gpio_t   *ptFan1;         /**< 风扇 1 驱动控制 (PA3) */
    gdi_gpio_t   *ptValveB;       /**< 电磁阀 2-2/阀B 控制 (PA4) */
    gdi_gpio_t   *ptValveA;       /**< 电磁阀 2-1/阀A 控制 (PA5) */
    gdi_gpio_t   *ptFan2;         /**< 风扇 2 驱动控制 (PA6) */
    gdi_gpio_t   *ptValveD;       /**< 电磁阀 3-1/阀D 控制 (PA7) */
    gdi_gpio_t   *ptValveC;       /**< 电磁阀 3-2/阀C 控制 (PB0) */
    gdi_gpio_t   *ptBuzzer;       /**< 蜂鸣器控制 (PB1) */
    gdi_gpio_t   *ptLedA;         /**< 指示灯 A 控制 (PA11) */
    gdi_gpio_t   *ptLedB;         /**< 指示灯 B 控制 (PA12) */
    gdi_gpio_t   *ptHx711Sck;     /**< 压力芯片时钟线 (PB3) */
    gdi_gpio_t   *ptHx711Dout;    /**< 压力芯片数据线 (PB4) */
    gdi_gpio_t   *ptAmbLed;       /**< 氛围灯条控制 (PA8) */

    /* ---------- Stream ---------- */
    gdi_stream_t *ptSerial;       /**< RS232 串口 (PA9/10) */

    /* ---------- ADC ---------- */
    gdi_adc_t    *ptBatAdc;       /**< 电池电压采样 (PA0) */
} gdi_hardware_t;

/**
 * @brief 全局统一的硬件资源实例
 * 
 * 真正的实例化发生在外设适配层（如 peripheral/AT32/port_gdi.c 中）
 */
extern const gdi_hardware_t HW;

#endif /* __GDI_HW_H__ */
