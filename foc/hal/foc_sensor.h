/****************************************************************************
 * @file    foc_sensor.h
 * @brief   Standard position/speed sensor interface for FOC
 * @author  Codex
 * @date    2026-09-03
 *
 * 无论底层是磁编码器、光电编码器、霍尔还是无感观测器，
 * 统一通过本接口向 FOC 提供机械角度、速度与有效性。
 ****************************************************************************/

#ifndef FOC_SENSOR_H
#define FOC_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"
#include "foc_angle.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /**
     * @brief 慢速周期刷新（如 1 kHz I2C/SPI 读芯片、ABZ 溢出处理）
     * @param pPriv 具体传感器私有实例指针
     * @return 0 成功, 负数 错误码
     */
    int32_t (*fnUpdate)(void *pPriv);

    /**
     * @brief 20 kHz 高频 ISR 实时读取（无锁、无阻塞读取角度与速度）
     * @param pPriv             具体传感器私有实例指针
     * @param ptMechanicalAngle 输出机械角度
     * @param pqMechanicalSpeed 输出机械速度
     * @param pbValid           输出数据有效性
     * @return FOC_RESULT_OK 或错误码
     */
    foc_result_t (*fnRead)(void *pPriv,
                           foc_angle_t *ptMechanicalAngle,
                           foc_scalar_t *pqMechanicalSpeed,
                           bool *pbValid);

    /**
     * @brief 电气零位标定（可选能力，不支持可置 NULL）
     * @param pPriv             具体传感器私有实例指针
     * @param ptElectricalZero  输出校准后的电气零位
     * @return FOC_RESULT_OK 或错误码
     */
    foc_result_t (*fnCalibrate)(void *pPriv,
                                foc_angle_t *ptElectricalZero);
} foc_sensor_ops_t;

/**
 * @brief 标准位置传感器对象（能力虚表 + 实例私有数据）
 */
typedef struct {
    const foc_sensor_ops_t *ptOps;  /**< 能力方法表 */
    void                   *pPriv;  /**< 传感器私有实例对象指针 */
} foc_sensor_t;

#ifdef __cplusplus
}
#endif

#endif /* FOC_SENSOR_H */
