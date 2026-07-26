/*******************************************************************************
 * @file    motor_profile.h
 * @brief   Public immutable high-frequency profiling snapshot and per-motor API
 ******************************************************************************/

#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H

#include "foc_numeric.h"
#include <stdint.h>

#define MOTOR_HF_PROFILE_VALID_TOTAL           (1U << 0)
#define MOTOR_HF_PROFILE_VALID_SAMPLE_CURRENT  (1U << 1)
#define MOTOR_HF_PROFILE_VALID_CLARKE          (1U << 2)
#define MOTOR_HF_PROFILE_VALID_PARK            (1U << 3)
#define MOTOR_HF_PROFILE_VALID_IPARK           (1U << 4)
#define MOTOR_HF_PROFILE_VALID_MODULATE        (1U << 5)
#define MOTOR_HF_PROFILE_VALID_COMMIT          (1U << 6)
#define MOTOR_HF_PROFILE_VALID_ENTRY           (1U << 7)
#define MOTOR_HF_PROFILE_VALID_POSITION        (1U << 8)
#define MOTOR_HF_PROFILE_VALID_PI              (1U << 9)
#define MOTOR_HF_PROFILE_VALID_SETDUTY         (1U << 10)

typedef struct {
    uint32_t wSampleSequence;           /**< 样本序号 */
    uint32_t wTotalCycles;              /**< 总周期数 */
    uint32_t wSampleCurrentCycles;      /**< 电流采样周期数 */
    uint32_t wClarkeCycles;             /**< Clarke 变换周期数 */
    uint32_t wParkCycles;               /**< Park 变换周期数 */
    uint32_t wIparkCycles;              /**< 逆 Park 变换周期数 */
    uint32_t wModulateCycles;           /**< 调制周期数 */
    uint32_t wCommitCycles;             /**< 提交（设置占空比）周期数 */
    uint32_t wEntryCycles;              /**< 入口快照临界区（含控制状态拷贝） */
    uint32_t wPositionCycles;           /**< 位置源采样与切换管理 */
    uint32_t wPiCycles;                 /**< D/Q 电流环 PI */
    uint32_t wSetDutyCycles;            /**< commit 内 SetDuty 子段 */
    uint32_t wValidFlags;               /**< 有效标志位 */
    foc_result_t eResult;               /**< 结果码 */
} motor_hf_profile_snapshot_t;

union motor_handle_u;
typedef union motor_handle_u motor_handle_t;

/**
 * @brief  获取高频性能分析快照
 * @param  ptMotor    电机句柄
 * @param  ptSnapshot 输出快照
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t motor_GetHighFrequencyProfileSnapshot(
    const motor_handle_t *ptMotor,
    motor_hf_profile_snapshot_t *ptSnapshot);

#endif /* MOTOR_PROFILE_H */
