/*******************************************************************************
 * @file    motor_private.h
 * @brief   Motor implementation layout, private to motor implementation files
 *
 * ===== 不透明句柄（Opaque Handle）模式 =====
 * 对外：motor_handle_t 是一个固定大小的联合体（约 800 字节），只暴露给
 *       公共 API（motor.h）。用户分配内存、传递指针，但无法访问内部成员。
 *
 * 对内：motor_impl_t 定义了实际的布局，通过 may_alias 指针转换访问。
 *       所有电机状态（PID、电流、位置源、状态机等）都存在这里。
 *
 * 好处：
 *   - 公共头文件不暴露实现细节，调用者无法依赖内部布局
 *   - C 语言级的封装（没有 C++ 虚拟表开销）
 *   - 固定大小允许静态分配，无需堆内存
 *
 * ===== 内存布局 =====
 * 按对齐大小降序排列（指针 → 4 字节 → 2 字节 → 1 字节 → 位域），
 * 以减少填充浪费。_Static_assert 验证总大小不超出公共句柄存储。
 *
 * ===== 线程安全 =====
 * 通过 motor_sync_if_t 提供的 enter/exit 回调实现。高频步和低频步
 * 之间可能存在竞争——高频 ISR 中采样电流和角度，低频步中读取快照。
 * 进入临界区禁用中断（或获取锁），退出时恢复。
 ******************************************************************************/

#ifndef MOTOR_PRIVATE_H
#define MOTOR_PRIVATE_H

#include "motor_types.h"
#include "foc_open_loop_source.h"

#define MOTOR_IMPL_MAGIC 0x4D4F544FU
#define MOTOR_EVENT_CAPACITY 4U

/*
 * Compact 8-byte event record. wSequence is an independent monotonic
 * event counter (never milliseconds or high-frequency sample indices).
 * chMeta packs from-state (3 bits), to-state (3 bits), and detail (2
 * bits, e.g. motor_position_role_e). hwPayload is one event-specific
 * 16-bit numeric value: fault bits for FAULT, command | result << 8
 * for command events, old | new << 8 flags or phases for validity and
 * transition events. Realtime code never formats strings for events.
 */
typedef struct {
    uint32_t wSequence;         /**< 单调递增事件序号 */
    uint16_t hwPayload;         /**< 事件负载（故障位/命令|结果/新旧标志） */
    uint8_t chType;             /**< 事件类型（motor_event_type_e） */
    uint8_t chMeta;             /**< 元数据打包（from|to|detail） */
} motor_event_record_t;

#define MOTOR_EVENT_META(eFrom, eTo, chDetail) \
    ((uint8_t)((uint8_t)(eFrom) | ((uint8_t)(eTo) << 3) | \
               ((uint8_t)(chDetail) << 6)))
#define MOTOR_EVENT_META_FROM(chMeta) ((uint8_t)((chMeta) & 0x7U))
#define MOTOR_EVENT_META_TO(chMeta)   ((uint8_t)(((chMeta) >> 3) & 0x7U))
#define MOTOR_EVENT_META_DETAIL(chMeta) ((uint8_t)((chMeta) >> 6))

#if defined(__GNUC__) || defined(__clang__)
/*
 * The public handle has declared byte-array storage. This local alias type
 * permits implementation lvalue access without disabling strict aliasing for
 * the rest of the firmware. GCC, Clang, and ARM GCC support may_alias.
 */
#define MOTOR_PRIVATE_MAY_ALIAS __attribute__((__may_alias__))
#else
#error "Opaque motor storage requires compiler may_alias support"
#endif

/*
 * Layout is ordered by alignment so the private implementation stays
 * inside the fixed public handle storage on both 32-bit targets and
 * 64-bit hosts: pointer-bearing blocks first, then 4-byte, 2-byte, and
 * 1-byte members. Enum state is stored narrowed to uint8_t; flags that
 * are never address-taken are bit-packed. Do not grow fields without
 * checking the static assert below.
 */
typedef struct MOTOR_PRIVATE_MAY_ALIAS {
    /* ===== 8-byte aligned: pointer-bearing blocks first ===== */
    foc_hal_t               tHal;              /**< HAL 接口副本（PWM + ADC 函数表 + 上下文） */
    motor_control_t         tControl;          /**< 控制环运行时状态（控制模式、dq 电流/电压、参考值、占空比） */
    /* Internal default PID instances if custom controllers are not provided. */
    foc_pid_t               tIdPid;            /**< 默认 D 轴 PID（电流环） */
    foc_pid_t               tIqPid;            /**< 默认 Q 轴 PID（电流环） */
    foc_pid_t               tSpeedPid;         /**< 默认速度环 PID */
    foc_pid_t               tPositionPid;      /**< 默认位置环 PID */
    /* One copy backs both bindings because different sources are rejected. */
    foc_position_source_if_t tPositionSource;  /**< 位置源接口表（函数指针 + 上下文，初始和目标共用） */
    motor_time_if_t         tTime;             /**< 时间接口（获取毫秒时间戳） */
    motor_sync_if_t         tSync;             /**< 同步接口（中断保护 enter/exit） */
    /* 4-byte block. */
    motor_state_t           tRt;               /**< 运行时状态（电角度、电速度、母线电压、故障、运行态） */
    phase_current_handle_t  tCurrent;          /**< 电流采样句柄（Clarke 变换的输入） */
    foc_position_config_t   tPositionConfig;   /**< 位置配置（极对数、零位偏移、方向） */
    foc_angle_t             tMechanicalAngle;  /**< 当前机械角度（电角度 / 极对数） */
    foc_scalar_t            qMechanicalSpeed;  /**< 当前机械速度（电速度 / 极对数） */
    foc_open_loop_source_t  tDefaultOpenLoopSource;  /**< 默认开环源实例（启动阶段用） */
    foc_angle_t             tCandidateAngle;   /**< 候选源角度（待切换的位置源输出） */
    foc_scalar_t            qCandidateSpeed;   /**< 候选源速度 */
    foc_scalar_t            qAngleError;       /**< 角度误差 = tCandidateAngle - tActiveAngle */
    foc_scalar_t            qBlendFactor;      /**< 混合进度 [0, 1]，0=完全当前源, 1=完全候选源 */
    foc_scalar_t            qOpenLoopCommandSpeed;     /**< 用户设置的开环命令速度 */
    foc_scalar_t            qHighFrequencyPeriod;      /**< 高频步周期（来自 config，q_type 归一化） */
    foc_scalar_t            qTransitionMinimumConfidence; /**< 切换最低置信度 */
    foc_scalar_t            qTransitionMinimumSpeed;      /**< 切换最低速度 */
    foc_scalar_t            qTransitionMaximumAngleError; /**< 切换最大角度误差 */
    foc_angle_t             tTransitionStartAngle;       /**< 切换起始角度 */
    foc_scalar_t            qTransitionStartSpeed;       /**< 切换起始速度 */
    uint32_t                wStartupDelayMs;    /**< 启动延时，单位 ms */
    uint32_t                wStartupStartMs;    /**< 启动开始时间戳 */
    uint32_t                wDiagnosticStartMs; /**< 诊断开始时间戳（仅诊断模式） */
    uint32_t                wTransitionTimeoutMs;      /**< 切换超时，单位 ms */
    uint32_t                wPositionSampleTimestamp;  /**< 位置采样时间戳 */
    uint32_t                wNextEventSequence;        /**< 下一事件序号 */
    uint32_t                wEventOverwriteCount;      /**< 事件覆盖计数 */
#if FOC_HF_PROFILE
    motor_hf_profile_snapshot_t tProfileSnapshot; /**< 高频性能分析快照 */
#endif
    uint32_t                wMagic;             /**< 魔数，验证实例已初始化 */
    motor_event_record_t    atEvents[MOTOR_EVENT_CAPACITY]; /**< 环形事件缓冲区 */
    /* 2-byte block. */
    uint16_t                hwTransitionQualificationSamples; /**< 资格判定稳定样本数 */
    uint16_t                hwTransitionBlendSamples;        /**< 混合过渡样本数 */
    uint16_t                hwTransitionSampleCount;         /**< 当前切换已过样本数 */
    /* 1-byte block: flag bytes and narrowed enum state. */
    uint8_t                 chActiveValidFlags;     /**< 当前源有效性标志 */
    uint8_t                 chCandidateValidFlags;  /**< 候选源有效性标志 */
    uint8_t                 chMechanicalValidFlags; /**< 机械位置有效性标志 */
    uint8_t                 chStartupPhase;         /**< 启动阶段 */
    uint8_t                 chPendingCommand;       /**< 待处理命令 */
    uint8_t                 chEventHead;            /**< 事件环形缓冲头部索引 */
    uint8_t                 chEventCount;           /**< 事件环形缓冲有效计数 */
    /* Address-taken re-entrancy flags stay plain bool. */
    bool                    bHighFrequencyStepInProgress; /**< 高频步正在执行 */
    bool                    bLowFrequencyStepInProgress;  /**< 低频步正在执行 */
    /* Bit-packed flags (never address-taken). */
    uint8_t                 bCommandPending : 1;            /**< 有命令待处理 */
    uint8_t                 bPwmEnabled : 1;               /**< PWM 输出已使能 */
    uint8_t                 bInitialPositionSourceBound : 1; /**< 初始角度源已绑定 */
    uint8_t                 bTargetPositionSourceBound : 1;  /**< 目标角度源已绑定 */
    uint8_t                 bOuterLoopActive : 1;           /**< 外环（速度/位置）已激活 */
    uint8_t                 bDiagnosticActive : 1;           /**< 诊断输出已激活 */
} motor_impl_t;

_Static_assert(sizeof(motor_impl_t) <= MOTOR_HANDLE_STORAGE_SIZE,
               "motor implementation exceeds public handle storage");
_Static_assert(_Alignof(motor_handle_t) >= _Alignof(motor_impl_t),
               "motor_handle_t private storage is insufficiently aligned");

static inline motor_impl_t *motor_private(motor_handle_t *ptMotor)
{
    return (motor_impl_t *)(void *)ptMotor;
}

static inline const motor_impl_t *motor_private_const(
    const motor_handle_t *ptMotor)
{
    return (const motor_impl_t *)(const void *)ptMotor;
}

/**
 * @brief  检查电机句柄是否已初始化（魔数校验）
 * @param  ptMotor  电机句柄
 * @return          true=已初始化, false=未初始化或 NULL
 */
static inline bool motor_private_is_initialized(
    const motor_handle_t *ptMotor)
{
    return ptMotor != NULL &&
           motor_private_const(ptMotor)->wMagic == MOTOR_IMPL_MAGIC;
}

/**
 * @brief  进入临界区
 * @param  ptImpl  私有实现指针
 * @return         保存的中断状态（用于退出时恢复）
 */
static inline uintptr_t motor_private_enter(const motor_impl_t *ptImpl)
{
    return ptImpl->tSync.fnEnter != NULL ?
        ptImpl->tSync.fnEnter(ptImpl->tSync.pContext) : 0U;
}

/**
 * @brief  退出临界区
 * @param  ptImpl  私有实现指针
 * @param  wState  motor_private_enter 返回的状态
 */
static inline void motor_private_exit(const motor_impl_t *ptImpl,
                                      uintptr_t wState)
{
    if (ptImpl->tSync.fnExit != NULL) {
        ptImpl->tSync.fnExit(ptImpl->tSync.pContext, wState);
    }
}

/**
 * @brief  设置三相占空比输出（内部实现）
 * @param  ptMotor  电机句柄
 * @param  qDutyU   U 相占空比
 * @param  qDutyV   V 相占空比
 * @param  qDutyW   W 相占空比
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_private_SetDuty(motor_handle_t *, q_type, q_type, q_type);
/**
 * @brief  使能或禁用 PWM 输出（内部实现）
 * @param  ptMotor  电机句柄
 * @param  bEnable  true=使能, false=禁用
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_private_Enable(motor_handle_t *, bool);
/**
 * @brief  执行电流采样校准（内部实现）
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_private_CalibrateCurrent(motor_handle_t *);
/**
 * @brief  执行一步电流采样（内部实现）
 * @param  ptMotor  电机句柄
 * @return          FOC_RESULT_OK 或错误码
 */
foc_result_t motor_private_SampleCurrent(motor_handle_t *);
/**
 * @brief  追加事件到环形缓冲区（内部实现）
 * @param  ptImpl    私有实现指针
 * @param  eType     事件类型
 * @param  eFrom     源状态
 * @param  eTo       目标状态
 * @param  chDetail  事件详情
 * @param  hwPayload 事件负载
 */
void motor_private_AppendEvent(motor_impl_t *, motor_event_type_e,
                               motor_state_e, motor_state_e,
                               uint8_t, uint16_t);
#endif /* MOTOR_PRIVATE_H */
