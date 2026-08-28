/*******************************************************************************
 * @file    motor.h
 * @brief   电机对象操作接口
 *
 * ===== 电机对象 =====
 * 电机对象通过 motor_handle_t 不透明句柄操作，分为两个执行层级：
 *
 * 高频层（High-Frequency, HF）：电流内环，~20 kHz，在 ADC 转换完成
 *   中断中由 motor_HighFrequencyStep() 驱动。此层完成：
 *   电流采样 → Clarke → Park → D/Q 电流 PI → 逆 Park → SVPWM → 占空比输出
 *
 * 低频层（Low-Frequency, LF）：速度/位置外环，~1 kHz，在主循环状态机
 *   中由 motor_LowFrequencyStep() 驱动。此层完成：
 *   速度 PID / 位置 PID → 更新电流参考值
 *
 * ===== 启动流程 =====
 * motor_Start() 触发状态机，经过以下阶段后进入正常运行：
 *
 *   1. MOTOR_STARTUP_CALIBRATE     — 电流采样偏移校准
 *   2. MOTOR_STARTUP_WAIT_DELAY    — 等待使能延时
 *   3. MOTOR_STARTUP_ENABLE        — 使能 PWM 输出
 *   4. MOTOR_STARTUP_QUALIFY_SOURCE — 目标角度源资格判定
 *   5. MOTOR_STARTUP_BLEND_ANGLE   — 从初始源混合过渡到目标源
 *   6. MOTOR_STARTUP_COMPLETE      — 启动完成，转入 RUNNING
 *
 * 启动过程由 motor_RunFSM() 驱动，必须在主循环中周期性调用。
 ******************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_types.h"

/**
 * @brief  初始化电机对象
 * @param  ptMotor   电机句柄
 * @param  ptConfig  配置（电机参数、HAL、控制配置等）
 * @return           FOC_RESULT_OK 或错误码
 */
foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig);
/**
 * @brief  复位电机对象到初始状态
 * @param  ptMotor  电机句柄
 */
void motor_Reset(motor_handle_t *ptMotor);
/**
 * @brief  读取三相 ADC 原始值
 * @param  ptMotor  电机句柄
 * @param  pwRawU   输出 U 相原始值
 * @param  pwRawV   输出 V 相原始值
 * @param  pwRawW   输出 W 相原始值
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetRawCurrent(motor_handle_t *ptMotor,
                                 uint32_t *pwRawU,
                                 uint32_t *pwRawV,
                                 uint32_t *pwRawW);
/**
 * @brief  紧急停止：立即关闭 PWM 并置故障标志
 * @param  ptMotor  电机句柄
 * @param  eFault   故障类型
 */
void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_e eFault);
/**
 * @brief  获取电机运行快照（线程安全，在同步保护下拷贝）
 * @param  ptMotor    电机句柄
 * @param  ptSnapshot 输出快照
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot);
/**
 * @brief  获取轻量运行状态
 * @param  ptMotor  电机句柄
 * @param  peState  输出运行状态
 * @param  pwFaults 输出故障标志
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetStatus(const motor_handle_t *ptMotor,
                             motor_state_e *peState,
                             uint32_t *pwFaults);
/**
 * @brief  获取轻量运行遥测
 * @param  ptMotor      电机句柄
 * @param  ptTelemetry  输出电流/角度/速度
 * @return              FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetTelemetry(const motor_handle_t *ptMotor,
                                motor_telemetry_t *ptTelemetry);
/**
 * @brief  获取电流采样校准值
 * @param  ptMotor 电机句柄
 * @param  ptCalib 输出校准值
 * @return         FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetCurrentCalibration(const motor_handle_t *ptMotor,
                                         foc_adc_calib_t *ptCalib);
/**
 * @brief  设置编码器电气零位偏移（运行时生效，下一高频拍起用）
 * @param  ptMotor           电机句柄
 * @param  tElectricalOffset 电角度补偿偏移（BAM32，单位：圈）
 * @return                   FOC_RESULT_OK 或错误码
 * @note    对齐标定后调用；运行中写入会造成角度跳变，建议停机时设置。
 */
foc_result_t motor_SetPositionOffset(motor_handle_t *ptMotor,
                                     foc_angle_t tElectricalOffset);
/**
 * @brief  读取电机事件日志（非阻塞）
 * @param  ptMotor  电机句柄
 * @param  ptEvent  输出事件
 * @return          true=成功读到事件, false=队列空
 */
bool motor_DebugReadEvent(motor_handle_t *ptMotor,
                          motor_event_t *ptEvent);
/**
 * @brief  启动电机
 * @param  ptMotor       电机句柄
 * @param  ptRunConfig   运行配置（控制模式、开环参数、初始角度源等）
 * @return               FOC_RESULT_OK 或错误码
 */
foc_result_t motor_Start(motor_handle_t *ptMotor,
                         const motor_run_config_t *ptRunConfig);
/**
 * @brief  停止电机
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_Stop(motor_handle_t *ptMotor);
/**
 * @brief  运行电机状态机（应在主循环或低频调度中周期性调用）
 * @param  ptMotor  电机句柄
 * @return          fsm_rt_t 状态机运行结果
 */
fsm_rt_t motor_RunFSM(motor_handle_t *ptMotor);
/**
 * @brief  清除电机故障，恢复到 IDLE 状态
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_ClearFault(motor_handle_t *ptMotor);
#if defined(MOTOR_ENABLE_TEST_HOOKS)
/**
 * @brief  获取电机实现结构体大小（测试用）
 * @return  sizeof(motor_impl_t)
 */
size_t motor_TestGetImplementationSize(void);
/**
 * @brief  测试：设置开环命令速度
 * @param  ptMotor  电机句柄
 * @param  qSpeed   速度值
 */
void motor_TestSetOpenLoopCommandSpeed(motor_handle_t *ptMotor,
                                      foc_scalar_t qSpeed);
/**
 * @brief  测试：破坏 FSM 状态
 * @param  ptMotor  电机句柄
 * @param  eState   强制状态
 * @param  ePhase   强制启动阶段
 */
void motor_test_CorruptFSM(motor_handle_t *ptMotor,
                           motor_state_e eState,
                           motor_startup_phase_e ePhase);
/**
 * @brief  测试：检查位置源绑定是否有效
 * @param  ptMotor  电机句柄
 * @return          true=绑定有效, false=绑定无效
 */
bool motor_test_PositionBindingsValid(const motor_handle_t *ptMotor);
/**
 * @brief  测试：触发源切换超时
 * @param  ptMotor  电机句柄
 * @return          true=成功触发
 */
bool motor_TestCommitTransitionTimeout(motor_handle_t *ptMotor);
#endif

/*
 * Hardware bring-up diagnostic output. Excluded from production builds
 * (FOC_ENABLE_DIAGNOSTIC=0). The implementation enforces IDLE, no-fault,
 * per-phase duty-limit and maximum-duration checks; callers outside the
 * motor implementation never touch motor private members.
 */
#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC
/**
 * @brief  设置固定占空比输出，用于硬件诊断
 * @param  ptMotor  电机句柄
 * @param  qDutyU   U 相占空比
 * @param  qDutyV   V 相占空比
 * @param  qDutyW   W 相占空比
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_DiagnosticSetOutput(motor_handle_t *ptMotor,
                                       foc_scalar_t qDutyU,
                                       foc_scalar_t qDutyV,
                                       foc_scalar_t qDutyW);
/**
 * @brief  停止诊断输出
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_DiagnosticStopOutput(motor_handle_t *ptMotor);
#endif

#endif /* __MOTOR_H__ */
