/*******************************************************************************
 * @file    observer_lib.h
 * @brief   传感器与观测器抽象接口
 ******************************************************************************/

#ifndef __OBSERVER_LIB_H__
#define __OBSERVER_LIB_H__

#include "foc_math_types.h"

typedef struct {
    q_type (*fnGetAngle)(void);
    q_type (*fnGetSpeed)(void);
    void   (*fnUpdate  )(void);
} sensor_interface_t;

typedef struct {
    q_type (*fnGetAngle)(void);
    q_type (*fnGetSpeed)(void);
    void   (*fnUpdate  )(q_type qIalpha, q_type qIbeta, q_type qValpha, q_type qVbeta);
} observer_interface_t;

#endif /* __OBSERVER_LIB_H__ */
