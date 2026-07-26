/*******************************************************************************
 * @file    foc_open_loop_source.h
 * @brief   Open-loop angle and speed generator implementing foc_position_source_if_t
 ******************************************************************************/

#ifndef FOC_OPEN_LOOP_SOURCE_H
#define FOC_OPEN_LOOP_SOURCE_H

#include "foc_angle.h"
#include "foc_numeric.h"
#include "motor_position.h"

typedef struct {
    foc_angle_t  tAngle;              /**< 状态：当前角度 */
    foc_scalar_t qSpeed;              /**< 状态：当前速度（带加速斜坡） */
    foc_scalar_t qTargetSpeed;        /**< 目标速度 */
    foc_scalar_t qAcceleration;       /**< 加速度 */
} foc_open_loop_source_t;

/**
 * @brief  初始化开环位置源
 * @param  ptSource  源实例指针
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_open_loop_source_Init(foc_open_loop_source_t *ptSource);

/**
 * @brief  设置开环速度
 * @param  ptSource  源实例指针
 * @param  qSpeed    速度值
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_open_loop_source_SetSpeed(foc_open_loop_source_t *ptSource,
                                            foc_scalar_t qSpeed);

/**
 * @brief  设置开环目标速度（带加速斜坡）
 * @param  ptSource      源实例指针
 * @param  qTargetSpeed  目标速度
 * @return               FOC_RESULT_OK 或错误码
 */
foc_result_t foc_open_loop_source_SetTargetSpeed(foc_open_loop_source_t *ptSource,
                                                  foc_scalar_t qTargetSpeed);

/**
 * @brief  设置开环加速度
 * @param  ptSource        源实例指针
 * @param  qAcceleration   加速度值
 * @return                 FOC_RESULT_OK 或错误码
 */
foc_result_t foc_open_loop_source_SetAcceleration(foc_open_loop_source_t *ptSource,
                                                   foc_scalar_t qAcceleration);

/**
 * @brief  设置开环角度
 * @param  ptSource  源实例指针
 * @param  tAngle    角度值
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_open_loop_source_SetAngle(foc_open_loop_source_t *ptSource,
                                            foc_angle_t tAngle);

/**
 * @brief  获取开环源的位置源接口
 * @param  ptSource  源实例指针
 * @return           位置源接口
 */
foc_position_source_if_t foc_open_loop_source_GetInterface(foc_open_loop_source_t *ptSource);

#endif /* FOC_OPEN_LOOP_SOURCE_H */
