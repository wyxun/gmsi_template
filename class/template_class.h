#ifndef __TEMPLATE_CLASS_H__
#define __TEMPLATE_CLASS_H__

#include "modus.h"

/* 配置结构体 —— 声明强类型数据共享依赖 */
typedef struct {
    uint8_t  *pchRingBuffer;
    uint16_t  hwRingSize;
    
    /* === 显性共享数据依赖申报 (DI 依赖注入) === */
    const uint32_t *pwSharedSystemTick; // 示范：强类型共享全局 tick
} template_class_cfg_t;

/* 对象物理实体 */
typedef struct {
    modus_base_t *ptBase;
    
    /* 模块显性依赖指针 */
    const uint32_t *pwSharedSystemTick;
    
    /* 模块私有状态机与定时器 */
    uint8_t        chState;      
    msoft_timer_t  tLedTimer;    
} template_class_t;

int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

#endif /* __TEMPLATE_CLASS_H__ */
