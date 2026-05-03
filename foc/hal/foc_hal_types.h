/*******************************************************************************
 * @file    foc_hal_types.h
 * @brief   FOC HAL 公共类型定义（采样拓扑、电流句柄）
 ******************************************************************************/

#ifndef __FOC_HAL_TYPES_H__
#define __FOC_HAL_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "foc_config.h"
#include "foc_math_types.h"

typedef enum {
    SENSING_TOPOLOGY_1P = 1,
    SENSING_TOPOLOGY_2P = 2,
    SENSING_TOPOLOGY_3P = 3,
} current_sensing_type_t;

typedef struct {
    uint32_t wOffsetU;
    uint32_t wOffsetV;
    uint32_t wOffsetW;
    bool     bIsCalibrated;
} foc_adc_calib_t;

struct phase_current_handle_s;

typedef struct {
    void (*fnStartConversion)(void);
    void (*fnOffsetCalib)(foc_adc_calib_t *ptCalib);
    void (*fnGetRaw)(uint32_t *pwRawU, uint32_t *pwRawV, uint32_t *pwRawW);
    void (*fnReconstruct)(struct phase_current_handle_s *ptHandle);
} foc_adc_ops_t;

typedef struct phase_current_handle_s {
    q_type      qIu;
    q_type      qIv;
    q_type      qIw;
    current_sensing_type_t eTopology;
    foc_adc_ops_t   tOps;
    foc_adc_calib_t tCalib;
} phase_current_handle_t;

#endif /* __FOC_HAL_TYPES_H__ */
