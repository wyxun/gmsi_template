/*******************************************************************************
 * @file    foc_app.h
 * @brief   FOC 应用层接口 — 挂载在 MODUS 调度框架下
 *
 * 应用层只通过 motor 公共 API（motor.h）与电机对象交互：
 * 命令用 motor_Start()/motor_Stop()/引用 setter，观测用
 * motor_GetSnapshot()/motor_DebugReadEvent()，实时算法由
 * motor_HighFrequencyStep()/motor_LowFrequencyStep() 按真实速率调度。
 ******************************************************************************/

#ifndef __FOC_APP_H__
#define __FOC_APP_H__

#include "modus.h"
#include "userconfig.h"
#include "perf_counter.h"
#include "motor_types.h"

typedef struct {
    motor_handle_t *ptMotor;    /**< 电机句柄指针 */
} foc_app_cfg_t;

typedef struct foc_app_s {
    modus_base_t    *ptBase;                /**< MODUS 基类 */
    uint32_t        wLastHeartbeatTick;     /**< 上次心跳 tick */
    uint32_t        wLastButtonTick;        /**< 上次按键 tick */
    bool            bLastButtonState;       /**< 上次按键状态 */
    motor_handle_t *ptMotor;                /**< 电机句柄 */
} foc_app_t;

/**
 * @brief  初始化 FOC 应用层实例
 * @param  wObjectAddr    应用对象地址（uintptr_t 转换）
 * @param  wObjectCfgAddr 应用配置地址
 * @return                FOC_RESULT_OK 或错误码
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);
/**
 * @brief  运行 FOC 应用状态机（主循环调度）
 * @param  ptThis  应用实例指针
 * @return         fsm_rt_t 状态机运行结果
 */
fsm_rt_t foc_app_RunFSM(foc_app_t *ptThis);
/**
 * @brief  启动 FOC 应用
 * @param  ptThis  应用实例指针
 */
void foc_app_Start(foc_app_t *ptThis);
/**
 * @brief  停止 FOC 应用
 * @param  ptThis  应用实例指针
 */
void foc_app_Stop(foc_app_t *ptThis);

/**
 * @brief  高频控制 ISR 入口（20 kHz，由 ADC 转换完成中断触发）
 */
void foc_app_HighFrequencyISR(void);

extern void phase_testA(void);
extern void phase_testB(motor_handle_t *ptMotor);
extern void phase_testC(struct foc_app_s *ptApp);
extern void phase_test_waveform_init(void);
extern void phase_test_waveform_step(void);
extern void phase_test_waveform_hf_step(motor_handle_t *ptMotor);

#endif /* __FOC_APP_H__ */
