/*******************************************************************************
 * @file    observer_lib.h
 * @brief   传感器与观测器抽象接口
 ******************************************************************************/

#ifndef __OBSERVER_LIB_H__
#define __OBSERVER_LIB_H__

#include "foc_observer.h"

typedef struct {
    void *pContext;
    foc_angle_t (*fnGetAngle)(void *pContext);
    foc_scalar_t (*fnGetSpeed)(void *pContext);
    void (*fnUpdate)(void *pContext);
} sensor_interface_t;

typedef foc_observer_if_t observer_interface_t;

#endif /* __OBSERVER_LIB_H__ */
