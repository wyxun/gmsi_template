/*******************************************************************************
 * @file    motor_types.h
 * @brief   电机对象核心类型定义
 ******************************************************************************/

#ifndef __MOTOR_TYPES_H__
#define __MOTOR_TYPES_H__

#include <stddef.h>
#include <stdint.h>

#include "foc_math_types.h"
#include "foc_angle.h"
#include "foc_hal.h"
#include "motor_control_types.h"
#include "motor_position.h"
#include "motor_profile.h"
#include "perf_counter.h"

typedef enum {
    MOTOR_STATE_IDLE        = 0,    /**< 空闲 */
    MOTOR_STATE_STARTING,           /**< 启动中（校准→开环→切换） */
    MOTOR_STATE_STOPPING,           /**< 停止中 */
    MOTOR_STATE_RUNNING,            /**< 正常运行 */
    MOTOR_STATE_FAULT,              /**< 故障 */
} motor_state_e;

typedef enum {
    MOTOR_STARTUP_IDLE = 0,             /**< 启动空闲 */
    MOTOR_STARTUP_CALIBRATE,            /**< 电流采样校准 */
    MOTOR_STARTUP_WAIT_DELAY,           /**< 等待使能延时 */
    MOTOR_STARTUP_ENABLE,               /**< 使能 PWM 输出 */
    MOTOR_STARTUP_QUALIFY_SOURCE,       /**< 资格判定目标角度源 */
    MOTOR_STARTUP_BLEND_ANGLE,          /**< 混合过渡到目标角度源 */
    MOTOR_STARTUP_COMPLETE,             /**< 启动完成，转入运行 */
} motor_startup_phase_e;

typedef enum {
    MOTOR_COMMAND_NONE = 0,             /**< 无命令 */
    MOTOR_COMMAND_START,                /**< 启动命令 */
    MOTOR_COMMAND_STOP,                 /**< 停止命令 */
} motor_command_e;

typedef enum {
    MOTOR_EVENT_COMMAND_ACCEPTED = 0,   /**< 命令已接受 */
    MOTOR_EVENT_COMMAND_REJECTED,       /**< 命令被拒绝 */
    MOTOR_EVENT_STATE_CHANGED,          /**< 状态机状态变化 */
    MOTOR_EVENT_SOURCE_VALIDITY_CHANGED, /**< 位置源有效性变化 */
    MOTOR_EVENT_TRANSITION_STARTED,     /**< 源切换开始 */
    MOTOR_EVENT_TRANSITION_COMPLETED,   /**< 源切换完成 */
    MOTOR_EVENT_TRANSITION_TIMEOUT,     /**< 源切换超时 */
    MOTOR_EVENT_FAULT,                  /**< 故障事件 */
} motor_event_type_e;

typedef enum {
    MOTOR_POSITION_ROLE_NONE = 0,       /**< 无角色 */
    MOTOR_POSITION_ROLE_ACTIVE,         /**< 当前使用的位置源 */
    MOTOR_POSITION_ROLE_CANDIDATE,      /**< 候选待切换的位置源 */
} motor_position_role_e;

typedef enum {
    MOTOR_FAULT_NONE            = 0U,       /**< 无故障 */
    MOTOR_FAULT_HARDWARE        = 1U << 0,  /**< 硬件故障 */
    MOTOR_FAULT_CURRENT_SAMPLE  = 1U << 1,  /**< 电流采样故障 */
    MOTOR_FAULT_INVALID_COMMAND = 1U << 2,  /**< 无效命令 */
    MOTOR_FAULT_POSITION_SOURCE = 1U << 3,  /**< 位置源故障 */
    MOTOR_FAULT_TRANSITION_TIMEOUT = 1U << 4, /**< 切换超时故障 */
} motor_fault_e;

typedef struct {
    uint32_t wResistanceMilliOhm;           /**< 相电阻，单位 mΩ */
    uint32_t wLdMicroHenry;                 /**< D 轴电感，单位 μH */
    uint32_t wLqMicroHenry;                 /**< Q 轴电感，单位 μH */
    uint32_t wBackEmfMicroVoltPerRadSec;    /**< 反电动势常数 Ke，单位 μV/(rad/s) */
    uint32_t wInertiaNanoKgM2;              /**< 转动惯量 J，单位 nkg·m² */
    uint32_t wRatedVoltageMilliVolt;        /**< 额定电压，单位 mV */
    uint32_t wRatedCurrentMilliAmp;         /**< 额定电流，单位 mA */
    uint8_t  chPolePairs;                   /**< 极对数 Pp */
} motor_params_t;

typedef struct {
    foc_angle_t         tThetaE;            /**< 当前电角度（BAM32） */
    q_type              qOmegaE;            /**< 当前电角速度（pu） */
    q_type              qVbus;              /**< 当前母线电压（pu） */
    uint32_t            wFaults;            /**< 故障标志位，按 motor_fault_e 组合 */
    motor_state_e       eRunState;          /**< 当前运行状态 */

} motor_state_t;

typedef struct {
    void *pContext;                         /**< 时间接口上下文 */
    uint32_t (*fnGetMilliseconds)(void *pContext); /**< 获取毫秒时间戳 */
} motor_time_if_t;

typedef struct {
    void *pContext;                         /**< 同步接口上下文 */
    uintptr_t (*fnEnter)(void *pContext);   /**< 进入临界区 */
    void (*fnExit)(void *pContext, uintptr_t wState); /**< 退出临界区 */
} motor_sync_if_t;

typedef struct {
    motor_control_mode_e eControlMode;          /**< 控制模式（电压开环/电流/速度/位置） */
    const foc_position_source_if_t *ptInitialPositionSource;  /**< 启动阶段用的初始角度源 */
    const foc_position_source_if_t *ptTargetPositionSource;   /**< 运行后切换到的目标角度源 */
    foc_scalar_t qInitialAngle;                 /**< 开环启动初始角度，单位：电圈数 */
    foc_scalar_t qOpenLoopSpeed;                /**< 开环速度，单位：电圈数/s */
    foc_scalar_t qAcceleration;                 /**< 开环加速度，单位：电圈数/s² */
    foc_dq_t tVoltageReference;                 /**< 电压参考值（电压开环模式直接使用） */
    foc_dq_t tCurrentReference;                 /**< 电流参考值（电流/速度/位置模式使用） */
    foc_scalar_t qSpeedReference;               /**< 速度参考值，单位：机械圈数/s */
    foc_scalar_t qPositionReference;            /**< 位置参考值，单位：机械圈数 */
} motor_run_config_t;

typedef struct {
    motor_params_t          tParams;            /**< 电机电气参数 */
    foc_hal_t               tHal;               /**< HAL 接口（PWM + ADC） */
    motor_control_config_t  tControl;           /**< 控制环配置 */
    current_sensing_type_t  eTopology;          /**< 电流采样拓扑（1/2/3 电阻） */
    motor_time_if_t         tTime;              /**< 时间接口 */
    motor_sync_if_t         tSync;              /**< 同步接口 */
    foc_scalar_t            qHighFrequencyPeriod; /**< 高频控制步周期，q_type 归一化 */
    foc_scalar_t            qLowFrequencyPeriod;  /**< 低频控制步周期，q_type 归一化 */
    foc_position_config_t   tPosition;          /**< 位置配置（极对数、零位偏移、方向） */
    foc_scalar_t            qTransitionMinimumConfidence;  /**< 源切换最低置信度 */
    foc_scalar_t            qTransitionMinimumSpeed;       /**< 源切换最低速度 */
    foc_scalar_t            qTransitionMaximumAngleError;  /**< 源切换最大允许角度误差 */
    uint32_t                wTransitionTimeoutMs;          /**< 源切换超时，单位 ms */
    uint16_t                hwTransitionQualificationSamples; /**< 资格判定稳定样本数 */
    uint16_t                hwTransitionBlendSamples;        /**< 混合过渡样本数 */
    uint32_t                wStartupDelayMs;               /**< 启动使能延时，单位 ms */
} motor_config_t;

/* Per-instance RAM and public ABI capacity. motor_private.h enforces the limit. */
#if FOC_HF_PROFILE
#define MOTOR_HANDLE_STORAGE_SIZE 832U
#else
#define MOTOR_HANDLE_STORAGE_SIZE 768U
#endif

typedef union motor_handle_u {
    max_align_t tAlignment;                     /**< 确保对齐 */
    uint8_t achPrivate[MOTOR_HANDLE_STORAGE_SIZE]; /**< 私有实现存储 */
} motor_handle_t;

_Static_assert(sizeof(motor_handle_t) == MOTOR_HANDLE_STORAGE_SIZE,
               "motor_handle_t storage size includes unexpected padding");

typedef struct {
    foc_scalar_t qIu;   /**< U 相电流 */
    foc_scalar_t qIv;   /**< V 相电流 */
    foc_scalar_t qIw;   /**< W 相电流 */
} motor_phase_current_t;

typedef struct {
    uint32_t wSequence;         /**< 事件序号 */
    uint32_t wFaults;           /**< 故障标志位 */
    uint32_t wPreviousValue;    /**< 事件前值 */
    uint32_t wCurrentValue;     /**< 事件当前值 */
    motor_event_type_e eType;           /**< 事件类型 */
    motor_state_e eFromState;           /**< 源状态 */
    motor_state_e eToState;             /**< 目标状态 */
    motor_command_e eCommand;           /**< 关联命令 */
    motor_position_role_e ePositionRole; /**< 位置源角色 */
    foc_result_t eResult;               /**< 关联结果码 */
} motor_event_t;

/*
 * Coherent diagnostic snapshot copied under motor_sync_if_t.
 *
 * 字段语义约定：
 *   - 角度（t*Angle）：以 foc_angle_t BAM32 格式表示的电圈数
 *   - 速度（q*Speed）：foc_scalar_t 电圈数/s（机械速度在变量名中显式标注）
 *   - 参考值/反馈（q*Reference/q*Current/q*Voltage）：pu 归一化 q_type
 *
 * 位置源角色说明：
 *   - active：当前被控制环使用的角度/速度
 *   - candidate：资格判定/混合过渡期间的目标源
 *   - open-loop：独立于候选/激活源的开环启动角度发生器
 *
 * 有效性：active/candidate 的角度/速度字段仅在对应的
 * eActiveSourceValidFlags / eCandidateSourceValidFlags 位被置位时有效。
 *
 * 安全性：此快照包含的是值拷贝，不包含指向私有存储、控制器、
 * 校准数据或 HAL 的指针，可安全在非特权上下文中读取。
 */
typedef struct {
    motor_state_e eRunState;            /**< 运行状态 */
    uint32_t wFaults;                   /**< 故障标志 */
    uint32_t wEventSequence;            /**< 最新事件序号 */
    uint32_t wEventOverwriteCount;      /**< 事件覆盖次数 */
    motor_phase_current_t tPhaseCurrent;  /**< 三相电流 */
    foc_dq_t tCurrentReference;          /**< 电流参考值（dq） */
    foc_dq_t tCurrent;                   /**< 实际电流（dq） */
    foc_dq_t tVoltageReference;          /**< 电压参考值（dq） */
    foc_dq_t tVoltage;                   /**< 实际电压（dq） */
    foc_duty_abc_t tDuty;                /**< 三相占空比 */
    foc_scalar_t qSpeedReference;        /**< 速度参考值 */
    foc_scalar_t qPositionReference;     /**< 位置参考值 */
    foc_angle_t tOpenLoopAngle;          /**< 开环角度 */
    foc_angle_t tActiveAngle;            /**< 当前使用的位置源角度 */
    foc_angle_t tCandidateAngle;         /**< 候选位置源角度 */
    foc_scalar_t qActiveSpeed;           /**< 当前使用的位置源速度 */
    foc_scalar_t qCandidateSpeed;        /**< 候选位置源速度 */
    foc_scalar_t qAngleError;            /**< 角度误差 */
    foc_scalar_t qBlendFactor;           /**< 混合过渡进度 [0, 1] */
    foc_angle_t tElectricalAngle;        /**< 最终电角度 */
    foc_scalar_t qElectricalSpeed;       /**< 最终电速度 */
    foc_scalar_t qVbus;                  /**< 母线电压，pu */
    foc_adc_calib_t tCurrentCalibration; /**< 电流采样校准值 */
    motor_control_mode_e eControlMode;   /**< 控制模式 */
    foc_position_valid_flag_e eActiveSourceValidFlags;   /**< 当前源有效性标志 */
    foc_position_valid_flag_e eCandidateSourceValidFlags; /**< 候选源有效性标志 */
    bool bPwmEnabled;                    /**< PWM 是否已使能 */
    motor_startup_phase_e eStartupPhase; /**< 启动阶段 */
    motor_command_e ePendingCommand;     /**< 待处理命令 */
} motor_snapshot_t;

#endif /* __MOTOR_TYPES_H__ */
