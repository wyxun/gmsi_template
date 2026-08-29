/****************************************************************************
 * @file    foc_app.h
 * @brief   Minimal single-motor FOC application interface
 *
 * 面向 STM32G431 单电机的极简应用层：唯一 foc_runtime_t 由 foc_app.c
 * 持有，本头只暴露命令、引用和状态读取。高频 ISR 由
 * foc_app_HighFrequencyISR() 进入，1 kHz 速度环由 MODUS Clock 驱动。
 ****************************************************************************/

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdint.h>
#include <stdbool.h>

#include "foc_types.h"

typedef struct {
    foc_run_state_e eState;
    uint32_t        wFaults;
    foc_angle_t     tElectricalAngle;
    foc_scalar_t    qElectricalSpeed;
    foc_dq_t        tCurrent;
    foc_dq_t        tVoltage;
    foc_duty_abc_t  tDuty;
    foc_adc_calib_t tCalibration;
    bool            bPwmEnabled;
} foc_status_t;

/**
 * @brief  初始化 FOC 应用（MODUS 对象注册入口）
 * @param  wObjectAddr    应用对象地址（未使用，兼容 MODUS 签名）
 * @param  wObjectCfgAddr 应用配置地址（未使用，兼容 MODUS 签名）
 * @return MODUS_SUCCESS 或错误码
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/**
 * @brief  20 kHz 高频控制 ISR 入口（ADC 转换完成中断）
 */
void foc_app_HighFrequencyISR(void);

/**
 * @brief  投递 START 命令（仅在 IDLE 接受），复制完整 command
 * @param  ptCommand  控制命令（模式、电压/电流/速度参考）
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_Start(const foc_core_command_t *ptCommand);

/**
 * @brief  停止：任何状态先执行硬件急停，再投递 STOP
 */
void foc_app_Stop(void);

/**
 * @brief  清除故障（仅 FAULT 且 PWM 已关闭时有效）
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_ClearFault(void);

/**
 * @brief  设置电压参考（仅 VOLTAGE 模式接受）
 * @param  qD  D 轴电压参考 (pu)
 * @param  qQ  Q 轴电压参考 (pu)
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_SetVoltageReference(foc_scalar_t qD,
                                         foc_scalar_t qQ);

/**
 * @brief  设置电流参考（仅 CURRENT 模式接受）
 * @param  qD  D 轴电流参考 (pu)
 * @param  qQ  Q 轴电流参考 (pu)
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_SetCurrentReference(foc_scalar_t qD,
                                         foc_scalar_t qQ);

/**
 * @brief  设置速度参考（仅 SPEED 模式接受）
 * @param  qMechanicalTurnPerSecond  机械速度参考 (turn/s)
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_SetSpeedReference(
    foc_scalar_t qMechanicalTurnPerSecond);

/**
 * @brief  在临界区内复制当前运行状态
 * @param  ptStatus  输出状态
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_app_GetStatus(foc_status_t *ptStatus);

#endif /* FOC_APP_H */
