#ifndef __FOC_VERIFY_H__
#define __FOC_VERIFY_H__

#include "motor.h"
#include "foc_config.h"

#if FOC_ENABLE_MOTOR_VERIFY

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  静态电角度锁定测试（开环电压模式）
 * @param  ptMotor 电机句柄指针
 * @param  fTurns  固定锁定电角度 (0.0f ~ 1.0f)
 * @param  fVq     输入 Vq 电压 (pu, 建议限制在 0.05f 以内)
 * @return foc_result_t
 */
foc_result_t foc_verify_StaticLock(motor_handle_t *ptMotor, float fTurns, float fVq);

/**
 * @brief  开环旋转测试（开环电压模式）
 * @param  ptMotor   电机句柄指针
 * @param  fVoltageQ 输入 Vq 电压 (pu)
 * @param  fSpeedHz  开环旋转频率 (Hz)
 * @return foc_result_t
 */
foc_result_t foc_verify_OpenLoopRun(motor_handle_t *ptMotor, float fVoltageQ, float fSpeedHz);

/**
 * @brief  电流闭环测试（D/Q 电流闭环 + 开环电角度拖动）
 * @param  ptMotor   电机句柄指针
 * @param  fIqRef    目标 Iq 电流 (pu, 如 0.03f)
 * @param  fSpeedHz  开环拖动频率 (Hz, 如 2.0f)
 * @return foc_result_t
 */
foc_result_t foc_verify_CurrentLoopRun(motor_handle_t *ptMotor, float fIqRef, float fSpeedHz);

/**
 * @brief  停止测试并使电机回归 IDLE 状态
 * @param  ptMotor 电机句柄指针
 * @return foc_result_t
 */
foc_result_t foc_verify_Stop(motor_handle_t *ptMotor);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ENABLE_MOTOR_VERIFY */

#endif /* __FOC_VERIFY_H__ */
