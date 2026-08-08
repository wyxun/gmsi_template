/*******************************************************************************
 * @file    motor_hf_private.h
 * @brief   高频 Fast Path 私有布局：plan / command / state / frame
 *
 * 这些类型只在 motor 实现内部使用（motor_private.h 包含），不暴露到公共
 * motor.h，调用者无法依赖其布局。
 *
 *   - motor_hf_plan_t     执行计划：Init 期填入 tIo，启动期（motor_Start）
 *                         由 plan resolver 填充位置源、电流环与调制函数
 *   - motor_hf_command_t  命令邮箱：慢路径在同步锁内写入，ISR 每周期读取
 *                         （控制模式 + 各环参考值）
 *   - motor_hf_state_t    高频运行时量：ISR 每周期读写（相电流句柄、dq
 *                         电流/电压、占空比、电角度/速度、位置源输出）
 *   - motor_hf_frame_t    单次高频步的栈上 scratch 帧（内核收敛时使用）
 *
 * 当前只含核心层字段（采样/变换/电流环/调制/提交）。后续扩展层（观测器
 * /DOB/cogging）接入时才用 FOC_CONFIG_ENABLE_* 宏裁剪。
 ******************************************************************************/

#ifndef MOTOR_HF_PRIVATE_H
#define MOTOR_HF_PRIVATE_H

#include "motor_types.h"

typedef struct {
    motor_startup_phase_e ePhase;
    foc_angle_t tStartAngle;
    foc_scalar_t qStartSpeed;
    uint16_t hwSampleCount;
    bool bChanged;
} motor_transition_update_t;

typedef foc_result_t (*motor_hf_modulate_fn_t)(
    const foc_ab_t *ptVoltage, foc_duty_abc_t *ptDuty);

typedef struct {
    foc_hf_io_if_t tIo;             /**< 高频采样/提交/急停接口（Init 填充） */
    void *pSourceContext;           /**< 位置源上下文（plan resolver 填充） */
    foc_result_t (*fnSourceStep)(void *, const foc_position_input_t *,
                                 foc_position_output_t *); /**< 位置源步进 */
    foc_controller_if_t tId;        /**< D 轴电流环（plan resolver 填充） */
    foc_controller_if_t tIq;        /**< Q 轴电流环（plan resolver 填充） */
    motor_hf_modulate_fn_t fnModulate; /**< 调制函数（plan resolver 填充） */
    foc_scalar_t qPeriod;           /**< 高频步周期（plan resolver 填充） */
    foc_position_config_t tPositionConfig; /**< 位置配置（plan resolver 填充） */
} motor_hf_plan_t;

typedef struct {
    motor_control_mode_e eMode;     /**< 当前控制模式 */
    foc_dq_t tVoltageReference;     /**< 电压参考值（电压开环模式使用） */
    foc_dq_t tCurrentReference;     /**< 电流参考值 */
    foc_scalar_t qSpeedReference;   /**< 速度参考值 */
    foc_scalar_t qPositionReference; /**< 位置参考值 */
} motor_hf_command_t;

typedef struct {
    uint16_t hwPayload;
    uint8_t chType;
    uint8_t chRole;
} motor_hf_pending_event_t;

typedef struct {
    phase_current_handle_t tPhaseCurrent; /**< 电流采样句柄（Clarke 输入） */
    foc_dq_t tCurrent;              /**< dq 电流反馈 */
    foc_dq_t tVoltage;              /**< dq 电压输出 */
    foc_ab_t tVoltageAlphaBeta;     /**< αβ 电压 */
    foc_duty_abc_t tDuty;           /**< 三相占空比 */
    foc_angle_t tElectricalAngle;   /**< 电角度（BAM32） */
    foc_scalar_t qElectricalSpeed;  /**< 电速度（pu） */
    foc_position_output_t tPositionOutput; /**< 最近一次位置源输出 */
    motor_hf_pending_event_t aPendingEvents[4]; /**< 延迟投递事件槽 */
    uint8_t chPendingCount;         /**< 待出列事件数 */
} motor_hf_state_t;

typedef struct {
    foc_ab_t tCurrentAlphaBeta;     /**< αβ 电流（Clarke 输出） */
    foc_position_input_t tPositionInput;   /**< 位置源输入 */
    foc_position_output_t tPositionOutput; /**< 位置源输出 */
    foc_dq_t tCurrent;              /**< dq 电流（Park 输出） */
    foc_dq_t tVoltage;              /**< dq 电压（电流环输出） */
    foc_ab_t tVoltageAlphaBeta;     /**< αβ 电压（IPark 输出） */
    foc_duty_abc_t tDuty;           /**< 三相占空比（调制输出） */
    foc_angle_t tAngle;             /**< 本拍电角度 */
    foc_scalar_t qSpeed;            /**< 本拍电速度 */
} motor_hf_frame_t;

#endif /* MOTOR_HF_PRIVATE_H */
