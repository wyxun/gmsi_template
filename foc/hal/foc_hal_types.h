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
    SENSING_TOPOLOGY_1P = 1,    /**< 单电阻采样 */
    SENSING_TOPOLOGY_2P = 2,    /**< 双电阻采样 */
    SENSING_TOPOLOGY_3P = 3,    /**< 三电阻采样 */
} current_sensing_type_t;

typedef struct {
    uint32_t wOffsetU;      /**< U 相 ADC 偏移量 */
    uint32_t wOffsetV;      /**< V 相 ADC 偏移量 */
    uint32_t wOffsetW;      /**< W 相 ADC 偏移量 */
    bool     bIsCalibrated; /**< 是否已完成校准 */
} foc_adc_calib_t;

struct phase_current_handle_s;

typedef struct {
    void *pContext;                     /**< ADC 上下文 */
    foc_result_t (*fnStartConversion)(void *pContext);  /**< 启动转换 */
    foc_result_t (*fnOffsetCalib)(void *pContext,
                                  foc_adc_calib_t *ptCalib);  /**< 偏移校准 */
    foc_result_t (*fnGetRaw)(void *pContext,
                             uint32_t *pwRawU,
                             uint32_t *pwRawV,
                             uint32_t *pwRawW);  /**< 读取原始值 */
    foc_result_t (*fnReconstruct)(void *pContext,
                                  struct phase_current_handle_s *ptHandle);  /**< 电流重建 */
} foc_adc_if_t;

typedef struct phase_current_handle_s {
    q_type      qIu;            /**< U 相电流，pu */
    q_type      qIv;            /**< V 相电流，pu */
    q_type      qIw;            /**< W 相电流，pu */
    current_sensing_type_t eTopology;   /**< 采样拓扑 */
    foc_adc_calib_t tCalib;             /**< ADC 校准值 */
} phase_current_handle_t;

#endif /* __FOC_HAL_TYPES_H__ */
