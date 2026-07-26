/*******************************************************************************
 * @file    foc_encoder.h
 * @brief   Absolute magnetic-encoder (e.g. AS5600) angle/speed source
 *
 * 仿 foc_hall 的适配模式：硬件读样本回调 + 观测器 + 位置源接口。
 * 观测器输出机械角度/机械速度，电角度由
 * foc_position_ApplyMechanicalConfig() 按极对数/方向/零位折算。
 *
 * 采样模型：低频任务更新样本缓存（回调返回新序号），20 kHz 步进消费样本；
 * 无新样本时按速度外插角度。
 ******************************************************************************/

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include "motor_position.h"

typedef struct {
    foc_scalar_t qSpeedFilterAlpha; /**< 速度低通滤波系数 [0, 1] */
    uint16_t hwInvalidTimeout;      /**< 无效样本超时，单位 tick */
} foc_encoder_params_t;

typedef struct {
    foc_encoder_params_t tParams;   /**< 编码器参数 */
    foc_angle_t tAngle;             /**< 状态：机械角度 */
    foc_scalar_t qSpeed;            /**< 状态：机械速度 (turn/tick) */
    foc_scalar_t qConfidence;       /**< 状态：置信度 */
    uint32_t wLastSequence;         /**< 状态：上次消费的样本序号 */
    uint16_t hwRawAngle;            /**< 状态：上次 12 位原始角度码 */
    uint16_t hwTicksSinceSample;    /**< 状态：距上次新样本的 tick 数 */
    uint16_t hwInvalidSamples;      /**< 状态：无效样本计数 */
    bool bInitialized;              /**< 状态：是否已获得首个样本 */
    bool bValid;                    /**< 状态：输出是否有效 */
} foc_encoder_t;

/**
 * @brief 硬件采样回调：取编码器缓存样本
 * @param  pHardwareContext  硬件上下文
 * @param  phwRawAngle       输出 12 位原始角度 [0, 4095]
 * @param  pwSequence        输出样本序号（更新即递增）
 * @param  pbMagnetOk        输出磁铁状态
 * @return true=样本可读, false=无有效数据
 */
typedef bool (*foc_encoder_read_sample_fn_t)(void *pHardwareContext,
                                             uint16_t *phwRawAngle,
                                             uint32_t *pwSequence,
                                             bool *pbMagnetOk);

typedef struct {
    foc_encoder_t *ptEncoder;
    void *pHardwareContext;
    foc_encoder_read_sample_fn_t fnReadSample;
} foc_encoder_source_adapter_t;

/**
 * @brief  用默认参数填充（速度滤波系数、无效超时）
 * @param  ptParams  输出参数
 */
void foc_encoder_DefaultParams(foc_encoder_params_t *ptParams);
/**
 * @brief  初始化编码器观测器实例
 * @param  ptEncoder  观测器实例指针
 * @param  ptParams   参数指针
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_params_t *ptParams);
/**
 * @brief  复位编码器观测器状态
 * @param  ptEncoder  观测器实例指针
 */
void foc_encoder_Reset(foc_encoder_t *ptEncoder);
/**
 * @brief  执行一步编码器角度 / 速度估算
 * @param  ptEncoder   观测器实例指针
 * @param  hwRawAngle  当前 12 位原始角度码
 * @param  wSequence   当前样本序号（与上次相同表示无新样本）
 * @param  bMagnetOk   磁铁状态
 * @param  ptOutput    输出（机械角度、机械速度、置信度等）
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_encoder_Step(foc_encoder_t *ptEncoder,
                              uint16_t hwRawAngle,
                              uint32_t wSequence,
                              bool bMagnetOk,
                              foc_position_output_t *ptOutput);
/**
 * @brief  将编码器观测器适配为位置源接口
 * @param  ptAdapter         适配器实例指针
 * @param  ptEncoder         观测器实例
 * @param  pHardwareContext  硬件读取函数上下文
 * @param  fnReadSample      样本读取函数
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_encoder_source_Init(
    foc_encoder_source_adapter_t *ptAdapter,
    foc_encoder_t *ptEncoder,
    void *pHardwareContext,
    foc_encoder_read_sample_fn_t fnReadSample);
/**
 * @brief  获取编码器适配器的位置源接口
 * @param  ptAdapter  适配器实例指针
 * @return 位置源接口
 */
foc_position_source_if_t foc_encoder_PositionSourceInterface(
    foc_encoder_source_adapter_t *ptAdapter);

#endif /* FOC_ENCODER_H */
