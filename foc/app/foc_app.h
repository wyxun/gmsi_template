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
    motor_handle_t *ptMotor;
} foc_app_cfg_t;

typedef struct foc_app_s {
    modus_base_t    *ptBase;
    uint32_t        wLastHeartbeatTick;
    uint32_t        wLastButtonTick;
    bool            bLastButtonState;
    motor_handle_t *ptMotor;
} foc_app_t;

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);
fsm_rt_t foc_app_RunFSM(foc_app_t *ptThis);
void foc_app_Start(foc_app_t *ptThis);
void foc_app_Stop(foc_app_t *ptThis);

/* 高频控制入口：仅由目标侧 ADC 抢占转换完成中断（TMR1 CH4 触发，
 * 20 kHz）调用，不得从主循环或低频调度调用。 */
void foc_app_HighFrequencyISR(void);

extern void phase_testA(void);
extern void phase_testB(motor_handle_t *ptMotor);
extern void phase_testC(struct foc_app_s *ptApp);
extern void phase_test_waveform_init(void);
extern void phase_test_waveform_step(void);

#endif /* __FOC_APP_H__ */
