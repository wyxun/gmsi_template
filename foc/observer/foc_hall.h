/*******************************************************************************
 * @file    foc_hall.h
 * @brief   Safe six-step Hall angle and speed observer
 ******************************************************************************/

#ifndef FOC_HALL_H
#define FOC_HALL_H

#include "motor_position.h"

typedef struct {
    uint8_t achSectorByCode[8];     /**< 霍尔编码到扇区的映射表 */
    uint16_t hwInvalidTimeout;      /**< 无霍尔边沿超时，单位 tick */
    foc_scalar_t qSpeedFilterAlpha; /**< 速度低通滤波系数 */
} foc_hall_params_t;

typedef struct {
    foc_hall_params_t tParams;      /**< Hall 参数 */
    foc_angle_t tAngle;             /**< 状态：估计角度 */
    foc_scalar_t qSpeed;            /**< 状态：估计速度 */
    foc_scalar_t qConfidence;       /**< 状态：置信度 */
    uint16_t hwTicksSinceEdge;      /**< 状态：距上次边沿的 tick 数 */
    uint16_t hwInvalidSamples;      /**< 状态：无效样本计数 */
    uint8_t chPreviousSector;       /**< 状态：上次扇区 */
    bool bInitialized;              /**< 状态：是否已初始化 */
    bool bValid;                    /**< 状态：输出是否有效 */
} foc_hall_t;

typedef uint8_t (*foc_hall_read_code_fn_t)(void *pHardwareContext);

typedef struct {
    foc_hall_t *ptHall;
    void *pHardwareContext;
    foc_hall_read_code_fn_t fnReadCode;
} foc_hall_source_adapter_t;

/**
 * @brief  用霍尔传感器映射表初始化默认参数
 * @param  ptParams  输出参数（扇区编码表、超时阈值、速度滤波系数）
 */
void foc_hall_DefaultParams(foc_hall_params_t *ptParams);
/**
 * @brief  初始化霍尔角度观测器实例
 * @param  ptHall    观测器实例指针
 * @param  ptParams  参数指针
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hall_Init(foc_hall_t *ptHall,
                           const foc_hall_params_t *ptParams);
/**
 * @brief  复位霍尔观测器状态
 * @param  ptHall  观测器实例指针
 */
void foc_hall_Reset(foc_hall_t *ptHall);
/**
 * @brief  执行一步霍尔角度 / 速度估算
 * @param  ptHall     观测器实例指针
 * @param  chHallCode 当前霍尔传感器编码
 * @param  ptOutput   输出（电角度、电速度、置信度等）
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hall_Step(foc_hall_t *ptHall,
                           uint8_t chHallCode,
                           foc_position_output_t *ptOutput);
/**
 * @brief  将霍尔观测器适配为位置源接口
 * @param  ptAdapter  适配器实例指针
 * @param  ptHall     霍尔观测器实例
 * @param  pHardwareContext  硬件读取函数上下文
 * @param  fnReadCode        霍尔编码读取函数
 * @return                   FOC_RESULT_OK 或错误码
 */
foc_result_t foc_hall_source_Init(foc_hall_source_adapter_t *ptAdapter,
                                  foc_hall_t *ptHall,
                                  void *pHardwareContext,
                                  foc_hall_read_code_fn_t fnReadCode);
/**
 * @brief  获取霍尔适配器的位置源接口
 * @param  ptAdapter  适配器实例指针
 * @return            位置源接口
 */
foc_position_source_if_t foc_hall_PositionSourceInterface(
    foc_hall_source_adapter_t *ptAdapter);

#endif /* FOC_HALL_H */
