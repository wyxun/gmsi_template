/*******************************************************************************
 * @file    motor_hf.c
 * @brief   高频 Fast Path 控制内核：电机 20 kHz 主控制步进循环
 *
 * 【V4 Fast Path 架构设计核心思想】
 * 1. 慢路径预解析（Zero-overhead Dispatch）：
 *    在绑定期（motor_Init / motor_Start）对所有回调（I/O 采样/提交、位置源、Id/Iq
 *    控制器、调制算法）进行严格的合法性与 ABI 校验，并解析填充为只读的 motor_hf_plan_t。
 * 2. 20 kHz ISR 路径零冗余（Direct Kernel Loop）：
 *    motor_HighFrequencyStep 在中断中直接调度已解析的回调指针，去除了二次包装、
 *    服务层转发、动态分支判断与非必要的锁竞争。
 * 3. 延迟事件槽与单出口故障处理：
 *    - 硬件或算力异常统一经由 motor_hf_fault() 出口触发急停（fnEmergencyStop）与故障标记；
 *    - 观察性状态变更（有效性改变、启动相位过渡）暂存至 ISR 4 槽无锁延迟队列
 *      aPendingEvents[4]，由低频调度 motor_LowFrequencyStep() 统一消费入环。
 ******************************************************************************/

#include "motor.h"
#include "motor_private.h"
#include "motor_hf_private.h"
#include "foc_modulation.h"
#include "foc_hf_profile.h"

/**
 * @brief  高频内核唯一故障出口（Fault Handling Exit）
 * @param  impl    电机私有实现句柄
 * @param  fault   故障类型掩码
 * @param  result  期望返回的结果码
 * @return         传入的 result
 * @note   在锁保护下写故障状态，解开重入标志，并立即触发硬件急停回路，确保系统安全。
 */
static foc_result_t motor_hf_fault(motor_impl_t *impl,
                                   motor_fault_e fault,
                                   foc_result_t result)
{
    uintptr_t s = motor_private_enter(impl);
    /* 1. 设置硬件故障字并切入故障状态 */
    impl->tRuntime.wFaults |= (uint32_t)fault;
    impl->tRuntime.eRunState = MOTOR_STATE_FAULT;
    impl->bPwmEnabled = false;
    impl->bHighFrequencyStepInProgress = false;

    /* 2. 故障事件作为安全状态变更，立即同步写入事件环形缓冲区 */
    motor_private_AppendEvent(impl, MOTOR_EVENT_FAULT, MOTOR_STATE_FAULT,
                              MOTOR_STATE_FAULT, 0U, (uint16_t)fault);
    motor_private_exit(impl, s);

    /* 3. 立即调用底层硬件急停回路（切断 PWM 驱动输出） */
    impl->tHfPlan.tIo.fnEmergencyStop(impl->tHfPlan.tIo.pIoContext);
    return result;
}

/**
 * @brief  校验候选位置源是否满足切换条件
 * @param  output           候选位置源的输出数据
 * @param  qualification    资格判定阈值参数
 * @param  reference_angle  开环参考电角度
 * @param  reference_speed  开环参考电速度
 * @param  timestamp        当前采样时间戳
 * @return                  true=合格, false=不合格
 */
static bool motor_hf_candidate_qualified(
    const foc_position_output_t *output,
    const foc_position_qualification_t *qualification,
    foc_angle_t reference_angle,
    foc_scalar_t reference_speed,
    uint32_t timestamp)
{
    foc_position_qualification_t sample = *qualification;

    sample.tReferenceAngle = reference_angle;
    sample.qReferenceSpeed = reference_speed;
    sample.wNow = timestamp;
    return foc_position_IsQualified(output, &sample);
}

/**
 * @brief  计算角度过渡平滑混合因子 progress = hwSample / hwTotal
 * @param  hwSample 当前已完成的融合样本数
 * @param  hwTotal  目标混合总样本数
 * @return          [0, 1] 范围的浮点/定点标量
 */
static foc_scalar_t motor_hf_blend_progress(uint16_t hwSample,
                                            uint16_t hwTotal)
{
    return foc_from_float((float)hwSample / (float)hwTotal);
}

/**
 * @brief  ISR 延迟事件无锁入队辅助函数
 * @param  state    高频运行状态句柄
 * @param  type     事件类型
 * @param  role     位置源角色（ACTIVE / CANDIDATE）
 * @param  payload  事件载荷数据
 */
static inline void motor_hf_post_event(motor_hf_state_t *state,
                                      motor_event_type_e type,
                                      motor_position_role_e role,
                                      uint16_t payload)
{
    /* 4 槽无锁延迟队列，防止覆盖 */
    if (state->chPendingCount < 4U) {
        state->aPendingEvents[state->chPendingCount++] = (motor_hf_pending_event_t){
            .hwPayload = payload,
            .chType = (uint8_t)type,
            .chRole = (uint8_t)role,
        };
    }
}

/*******************************************************************************
 * @brief  电机高频 20 kHz 控制步进函数 (Fast Path 核心入口)
 * @param  ptMotor  电机公开句柄
 * @return          FOC_RESULT_OK 或错误码
 *
 * 【控制算法完整数据流】
 *  1. 状态与并发拦截  -> 检查初始化、重入标志、PWM使能与运行状态；
 *  2. 提取局部快照    -> 拷贝 command / plan 局部只读变量，快速解锁；
 *  3. 批量相电流采样  -> 调度 plan->tIo.fnSampleCurrent 直接读取 ADC；
 *  4. Clarke 变换     -> 三相电流 (Iu, Iv, Iw) 转换为静止坐标系 (Iα, Iβ)；
 *  5. 位置源步进      -> 调度位置源 fnStep 计算当前电角度与速度；
 *  6. 切源与角度融合  -> 处理开环拖动到闭环观察器的平滑过渡（ Qualification & Blend ）；
 *  7. Park 变换       -> 基于正余弦表将 (Iα, Iβ) 旋转变换为 (Id, Iq) 闭环反馈；
 *  8. D/Q 电流闭环    -> 调度计划中的 Id/Iq PI 控制器生成 (Vd, Vq) 输出；
 *  9. 逆 Park 变换    -> 将 (Vd, Vq) 解旋恢复为静止坐标系 (Vα, Vβ)；
 * 10. 已解析调制      -> 调度调制算法（SVPWM/SPWM）生成三相占空比；
 * 11. 占空比硬件提交  -> 调度 plan->tIo.fnCommitDuty 一次性写入三相寄存器；
 * 12. 状态发布与事件  -> 将本拍运算结果写回 state，可观察事件入延迟槽。
 ******************************************************************************/
foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *impl;
    motor_hf_plan_t *plan;
    motor_hf_command_t *command;
    motor_hf_state_t *state;
    motor_hf_frame_t frame = {0};   /* 栈上 scratch 临时帧，避免全局状态乱序 */
    foc_result_t result;
    uintptr_t s;
    motor_transition_update_t transition = {0};
    foc_scalar_t blend_factor = FOC_ZERO;

#if FOC_HF_PROFILE
    /* Profile 性能测量变量及整环计时开始 */
    uint32_t wTotalCycles = 0U, wEntryCycles = 0U, wSampleCurrentCycles = 0U;
    uint32_t wPositionCycles = 0U, wAlgoCycles = 0U, wCommitCycles = 0U;
    FOC_HF_PROFILE_TOTAL_BEGIN(tTotalStart);
#endif

    /* -------------------------------------------------------------------------
     * 阶段 0：前置参数与合法性校验
     * ------------------------------------------------------------------------- */
    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }

    impl = motor_private(ptMotor);
    plan = &impl->tHfPlan;
    command = &impl->tHfCommand;
    state = &impl->tHfState;

#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tEntryStart);
#endif

    /* 临界区：快速校验并提取当前高频控制所需的慢路径配置只读快照 */
    s = motor_private_enter(impl);
    
    /* 0.1 检查并发重入标志：防止 20 kHz ISR 被抢占或重入引发数据竞争 */
    if (impl->bHighFrequencyStepInProgress) {
        motor_private_exit(impl, s);
        return FOC_RESULT_BUSY;
    }
    
    /* 0.2 检查运行状态与使能位：非 STARTING/RUNNING、有故障或 PWM 禁用时拒绝执行 */
    if ((impl->tRuntime.eRunState != MOTOR_STATE_STARTING &&
         impl->tRuntime.eRunState != MOTOR_STATE_RUNNING) ||
        impl->tRuntime.wFaults != MOTOR_FAULT_NONE ||
        !impl->bPwmEnabled) {
        motor_private_exit(impl, s);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    impl->bHighFrequencyStepInProgress = true;

    /* 0.3 提取局部快照，减少后续在 ISR 计算期间对全局结构的锁持有与内存访问 */
    motor_control_mode_e mode = command->eMode;
    foc_dq_t voltage_ref = command->tVoltageReference;
    foc_dq_t current_ref = command->tCurrentReference;
    bool direct_source = impl->bInitialPositionSourceBound;
    bool candidate_source = impl->bTargetPositionSourceBound;
    bool transition_required = !direct_source && candidate_source;
    foc_angle_t angle = state->tElectricalAngle;
    foc_scalar_t speed = impl->qOpenLoopCommandSpeed;
    motor_startup_phase_e startup_phase = impl->chStartupPhase;
    uint16_t transition_samples = impl->hwTransitionSampleCount;
    foc_angle_t transition_start_angle = impl->tTransitionStartAngle;
    foc_scalar_t transition_start_speed = impl->qTransitionStartSpeed;
    foc_scalar_t high_frequency_period = impl->qHighFrequencyPeriod;
    foc_position_config_t position_config = impl->tPositionConfig;
    foc_position_qualification_t qualification = (foc_position_qualification_t){
        .eRequiredValid = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                          FOC_POSITION_VALID_ELECTRICAL_SPEED,
        .qMinimumConfidence = impl->qTransitionMinimumConfidence,
        .qMinimumSpeed = impl->qTransitionMinimumSpeed,
        .qMaximumAngleError = impl->qTransitionMaximumAngleError,
        .wMaximumAge = 0U,
    };
    uint16_t qualification_samples = impl->hwTransitionQualificationSamples;
    uint16_t blend_samples = impl->hwTransitionBlendSamples;
    
    /* 更新并生成递增的采样时间戳 */
    uint32_t position_timestamp = ++impl->wPositionSampleTimestamp;
    if (position_timestamp == 0U) {
        position_timestamp = ++impl->wPositionSampleTimestamp;
    }
    motor_private_exit(impl, s);

#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tEntryStart, wEntryCycles);
#endif

    /* -------------------------------------------------------------------------
     * 阶段 1：批量相电流采样（Direct Hardware Sample）
     * ------------------------------------------------------------------------- */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tSampleStart);
#endif
    /* 直接调用绑定期解析的 fnSampleCurrent 回调，读取三相 ADC 并完成零偏与归一化 */
    result = plan->tIo.fnSampleCurrent(plan->tIo.pIoContext,
                                       &state->tPhaseCurrent);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tSampleStart, wSampleCurrentCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_HARDWARE, result);
    }

    /* -------------------------------------------------------------------------
     * 阶段 2：Clarke 变换与位置源步进
     * ------------------------------------------------------------------------- */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tPosStart);
#endif
    /* 2.1 Clarke 变换：将三相电流 (Iu, Iv, Iw) 投影到 α-β 两相静止坐标系 */
    result = foc_clarke(state->tPhaseCurrent.qIu,
                        state->tPhaseCurrent.qIv,
                        state->tPhaseCurrent.qIw,
                        &frame.tCurrentAlphaBeta);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    /* 构造位置源步进所需的输入数据结构 */
    frame.tPositionInput = (foc_position_input_t){
        .tCurrent = frame.tCurrentAlphaBeta,
        .tVoltage = state->tVoltageAlphaBeta,
        .qSamplePeriod = high_frequency_period,
        .wTimestamp = position_timestamp,
    };

    /* 2.2 位置源步进计算：根据启动阶段选择开环发生器或闭环观察器/编码器 */
    if (transition_required) {
        /* 需要切换：同时步进内部开环源与目标闭环观察器 */
        foc_position_output_t open_loop_output = {0};
        foc_position_source_if_t open_loop_if =
            foc_open_loop_source_GetInterface(&impl->tDefaultOpenLoopSource);
        result = open_loop_if.fnStep(open_loop_if.pSourceContext,
                                     &frame.tPositionInput,
                                     &open_loop_output);
        if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
        }
        angle = open_loop_output.tElectricalAngle;
        speed = open_loop_output.qElectricalSpeed;

        /* 调度目标位置源 fnStep 回调 */
        result = plan->fnSourceStep(plan->pSourceContext,
                                    &frame.tPositionInput,
                                    &frame.tPositionOutput);
        if (result != FOC_RESULT_OK || frame.tPositionOutput.wFaults != 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  result != FOC_RESULT_OK ?
                                  result : FOC_RESULT_INVALID_ARGUMENT);
        }
        /* 若输出机械量，由统一框架应用极对数、方向与机械零位平移 */
        if ((frame.tPositionOutput.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            result = foc_position_ApplyMechanicalConfig(
                &position_config, &frame.tPositionOutput);
            if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
                FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
                return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
            }
        }
    } else {
        /* 无需切换：直接步进已绑定的主要位置源 */
        result = plan->fnSourceStep(plan->pSourceContext,
                                    &frame.tPositionInput,
                                    &frame.tPositionOutput);
        if (result != FOC_RESULT_OK || frame.tPositionOutput.wFaults != 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  result != FOC_RESULT_OK ?
                                  result : FOC_RESULT_INVALID_ARGUMENT);
        }
        if ((frame.tPositionOutput.eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED)) != 0U) {
            result = foc_position_ApplyMechanicalConfig(
                &position_config, &frame.tPositionOutput);
            if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
                FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
                return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE, result);
            }
        }
        if ((frame.tPositionOutput.eValidFlags &
             FOC_POSITION_VALID_ELECTRICAL_ANGLE) == 0U) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  FOC_RESULT_INVALID_ARGUMENT);
        }
        angle = frame.tPositionOutput.tElectricalAngle;
        if ((frame.tPositionOutput.eValidFlags &
             FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U) {
            speed = frame.tPositionOutput.qElectricalSpeed;
        }
    }

    /* 2.3 并行观测源：已在低频步（motor_LowFrequencyStep，1 kHz）驱动。
       观测器计算（含 atan2）不放在 20 kHz ISR 里 —— 周期性的观测
       时序扰动会破坏 PWM 控制节奏，导致电机"只震不转"。
       候选角度/速度由低频步写入 state->tObservationOutput，
       本 ISR 只读发布（见阶段 7）。 */

    /* -------------------------------------------------------------------------
     * 阶段 3：源切换管理（资格判定与最短路径混合过渡）
     * ------------------------------------------------------------------------- */
    if (transition_required &&
        startup_phase == MOTOR_STARTUP_QUALIFY_SOURCE) {
        /* 3.1 资格判定阶段：检查观察器置信度、转速与角度误差是否持续合格 */
        if (motor_hf_candidate_qualified(
                &frame.tPositionOutput, &qualification, angle, speed,
                position_timestamp)) {
            transition_samples++;
            if (transition_samples >= qualification_samples) {
                /* 连续合格，进入混合过渡阶段 */
                transition.ePhase = MOTOR_STARTUP_BLEND_ANGLE;
                transition.tStartAngle = angle;
                transition.qStartSpeed = speed;
                transition.hwSampleCount = 0U;
            } else {
                transition.ePhase = startup_phase;
                transition.tStartAngle = transition_start_angle;
                transition.qStartSpeed = transition_start_speed;
                transition.hwSampleCount = transition_samples;
            }
        } else {
            /* 判定中断，重置样本计数 */
            transition.ePhase = startup_phase;
            transition.tStartAngle = transition_start_angle;
            transition.qStartSpeed = transition_start_speed;
            transition.hwSampleCount = 0U;
        }
        transition.bChanged = true;
    } else if (transition_required &&
               startup_phase == MOTOR_STARTUP_BLEND_ANGLE) {
        /* 3.2 混合过渡阶段：在开环角与闭环角之间按比例加权平滑接管，避免电流冲击 */
        foc_position_output_t from = {
            .tElectricalAngle = transition_start_angle,
            .qElectricalSpeed = transition_start_speed,
            .qConfidence = FOC_ONE,
            .eValidFlags = FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                           FOC_POSITION_VALID_ELECTRICAL_SPEED,
            .wTimestamp = position_timestamp,
        };
        foc_position_output_t blended;

        if (!motor_hf_candidate_qualified(
                &frame.tPositionOutput, &qualification, transition_start_angle,
                transition_start_speed, position_timestamp)) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_POSITION_SOURCE,
                                  FOC_RESULT_INVALID_ARGUMENT);
        }
        transition_samples++;
        blend_factor = motor_hf_blend_progress(transition_samples,
                                              blend_samples);
        result = foc_position_Blend(&from, &frame.tPositionOutput,
                                    blend_factor, &blended);
        if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
            FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif
            return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
        }
        angle = blended.tElectricalAngle;
        speed = blended.qElectricalSpeed;
        transition.ePhase = transition_samples >= blend_samples ?
            MOTOR_STARTUP_COMPLETE : MOTOR_STARTUP_BLEND_ANGLE;
        transition.tStartAngle = transition_start_angle;
        transition.qStartSpeed = transition_start_speed;
        transition.hwSampleCount = transition_samples;
        transition.bChanged = true;
        
        /* 切换完成瞬间，反向预置速度/位置控制器，确保无缝接管 */
        if (transition.ePhase == MOTOR_STARTUP_COMPLETE &&
            mode >= MOTOR_CONTROL_SPEED) {
            foc_scalar_t speed_reference = command->qSpeedReference;
            if (mode >= MOTOR_CONTROL_POSITION) {
                speed_reference = frame.tPositionOutput.qMechanicalSpeed;
                foc_controller_Track(&impl->tControlConfig.tPosition,
                    speed_reference, command->qPositionReference,
                    foc_angle_to_turns(frame.tPositionOutput.tMechanicalAngle));
            }
            foc_controller_Track(&impl->tControlConfig.tSpeed,
                current_ref.qQ, speed_reference,
                frame.tPositionOutput.qMechanicalSpeed);
        }
    }
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tPosStart, wPositionCycles);
#endif

    /* -------------------------------------------------------------------------
     * 阶段 4：核心算法 (Park -> PI 电流环 -> IPark -> 调制)
     * ------------------------------------------------------------------------- */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tAlgoStart);
#endif
    /* 4.1 正余弦计算：基于 BAM32 角度计算成对 sin/cos（LUT 或 CORDIC 后端） */
    foc_scalar_t sin_theta, cos_theta;
    foc_angle_sincos(angle, &sin_theta, &cos_theta);
    
    /* 4.2 Park 变换：复用已计算的正余弦值，将 (Iα, Iβ) 转换为 d-q 旋转坐标系电流 */
    result = foc_park_cached(&frame.tCurrentAlphaBeta, sin_theta, cos_theta,
                             &frame.tCurrent);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    /* 4.3 电流闭环调节：开环模式直接赋值，闭环模式调度只读的 tId/tIq PI 回调 */
    if (mode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP) {
        frame.tVoltage = voltage_ref;
    } else {
        frame.tVoltage.qD = plan->tId.fnStep(plan->tId.pController,
                                             current_ref.qD,
                                             frame.tCurrent.qD);
        frame.tVoltage.qQ = plan->tIq.fnStep(plan->tIq.pController,
                                             current_ref.qQ,
                                             frame.tCurrent.qQ);
    }

    /* 4.4 逆 Park 变换：将调节生成的 (Vd, Vq) 解旋还原为 α-β 轴电压 */
    result = foc_ipark_cached(&frame.tVoltage, sin_theta, cos_theta,
                              &frame.tVoltageAlphaBeta);
    if (result != FOC_RESULT_OK) {
#if FOC_HF_PROFILE
        FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    /* 4.5 调制计算：调度绑定的调制回调（SVPWM / SPWM）生成三相占空比 */
    result = plan->fnModulate(&frame.tVoltageAlphaBeta, &frame.tDuty);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tAlgoStart, wAlgoCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_INVALID_COMMAND, result);
    }

    /* -------------------------------------------------------------------------
     * 阶段 5：软停机与并发拦截点
     * ------------------------------------------------------------------------- */
    s = motor_private_enter(impl);
    bool stopping = impl->bCommandPending &&
                    impl->chPendingCommand == MOTOR_COMMAND_STOP;
    if (stopping ||
        (impl->tRuntime.eRunState != MOTOR_STATE_STARTING &&
         impl->tRuntime.eRunState != MOTOR_STATE_RUNNING)) {
        impl->bHighFrequencyStepInProgress = false;
        motor_private_exit(impl, s);
        return FOC_RESULT_BUSY;
    }
    motor_private_exit(impl, s);

    /* -------------------------------------------------------------------------
     * 阶段 6：占空比硬件提交（Direct Duty Commit）
     * ------------------------------------------------------------------------- */
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_BEGIN(tCommitStart);
#endif
    /* 直接调度硬件直写回调 fnCommitDuty，一次性将三相占空比写入定时器预装载寄存器 */
    result = plan->tIo.fnCommitDuty(plan->tIo.pIoContext, &frame.tDuty);
#if FOC_HF_PROFILE
    FOC_HF_PROFILE_STAGE_END(tCommitStart, wCommitCycles);
#endif
    if (result != FOC_RESULT_OK) {
        return motor_hf_fault(impl, MOTOR_FAULT_HARDWARE, result);
    }

    /* -------------------------------------------------------------------------
     * 阶段 7：状态发布与 ISR 延迟事件处理
     * ------------------------------------------------------------------------- */
    s = motor_private_enter(impl);
    /* 7.1 更新高频核心状态量（电角度、速度、dq 电压电流、占空比等） */
    state->tElectricalAngle = angle;
    state->qElectricalSpeed = speed;
    impl->qOpenLoopCommandSpeed = speed;
    const foc_position_output_t *candidate =
        plan->fnObservationStep != NULL
        ? &state->tObservationOutput
        : &frame.tPositionOutput;
    impl->tCandidateAngle = candidate->tElectricalAngle;
    impl->qCandidateSpeed = candidate->qElectricalSpeed;
    impl->qAngleError = foc_angle_diff(
        candidate->tElectricalAngle,
        state->tElectricalAngle);
    impl->qBlendFactor = blend_factor;
    
    uint8_t active_valid = direct_source ?
        (uint8_t)frame.tPositionOutput.eValidFlags :
        (uint8_t)(FOC_POSITION_VALID_ELECTRICAL_ANGLE |
                  FOC_POSITION_VALID_ELECTRICAL_SPEED);
    uint8_t candidate_valid = plan->fnObservationStep != NULL
        ? (uint8_t)state->tObservationOutput.eValidFlags
        : (candidate_source ?
           (uint8_t)frame.tPositionOutput.eValidFlags : 0U);

    /* 7.2 观察性事件延迟投递：写入 aPendingEvents[4] 无锁槽位，不直接竞争事件环锁 */
    if (active_valid != impl->chActiveValidFlags) {
        motor_hf_post_event(state, MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                            MOTOR_POSITION_ROLE_ACTIVE,
                            (uint16_t)((uint16_t)impl->chActiveValidFlags |
                                       ((uint16_t)active_valid << 8)));
        impl->chActiveValidFlags = active_valid;
    }
    if (candidate_valid != impl->chCandidateValidFlags) {
        /* 绑定观测源时抑制 CANDIDATE 有效性事件：观测器在阈值边缘
           抖动（如 SMO 低速 BEMF≈阈值）会每拍翻转刷爆事件环/RTT；
           观测有效性只是遥测，不是切换候选，不值得发事件。 */
        if (plan->fnObservationStep == NULL) {
            motor_hf_post_event(state,
                                MOTOR_EVENT_SOURCE_VALIDITY_CHANGED,
                                MOTOR_POSITION_ROLE_CANDIDATE,
                                (uint16_t)((uint16_t)
                                           impl->chCandidateValidFlags |
                                           ((uint16_t)candidate_valid << 8)));
        }
        impl->chCandidateValidFlags = candidate_valid;
    }
    impl->tMechanicalAngle = frame.tPositionOutput.tMechanicalAngle;
    impl->qMechanicalSpeed = frame.tPositionOutput.qMechanicalSpeed;
    impl->chMechanicalValidFlags =
        (uint8_t)(frame.tPositionOutput.eValidFlags &
         (FOC_POSITION_VALID_MECHANICAL_ANGLE |
          FOC_POSITION_VALID_MECHANICAL_SPEED));
    
    if (transition.bChanged) {
        motor_startup_phase_e previous_phase = impl->chStartupPhase;
        impl->chStartupPhase = transition.ePhase;
        impl->tTransitionStartAngle = transition.tStartAngle;
        impl->qTransitionStartSpeed = transition.qStartSpeed;
        impl->hwTransitionSampleCount = transition.hwSampleCount;
        if (previous_phase != transition.ePhase &&
            transition.ePhase == MOTOR_STARTUP_BLEND_ANGLE) {
            motor_hf_post_event(state, MOTOR_EVENT_TRANSITION_STARTED, 0U,
                                (uint16_t)((uint16_t)previous_phase |
                                           ((uint16_t)transition.ePhase << 8)));
        } else if (previous_phase != transition.ePhase &&
                   transition.ePhase == MOTOR_STARTUP_COMPLETE) {
            motor_hf_post_event(state, MOTOR_EVENT_TRANSITION_COMPLETED, 0U,
                                (uint16_t)((uint16_t)previous_phase |
                                           ((uint16_t)transition.ePhase << 8)));
        }
    }
    
    /* 7.3 保存本拍运算波形与诊断数据 */
    state->tCurrent = frame.tCurrent;
    state->tVoltage = frame.tVoltage;
    state->tVoltageAlphaBeta = frame.tVoltageAlphaBeta;
    state->tDuty = frame.tDuty;
    state->tPositionOutput = frame.tPositionOutput;
    
    /* 清除并发重入标记 */
    impl->bHighFrequencyStepInProgress = false;

#if FOC_HF_PROFILE
    /* 7.4 Profile 数据分析快照写回 */
    FOC_HF_PROFILE_TOTAL_END(tTotalStart, wTotalCycles);
    impl->tProfileSnapshot = (motor_hf_profile_snapshot_t){
        .wSampleSequence = position_timestamp,
        .wTotalCycles = wTotalCycles,
        .wSampleCurrentCycles = wSampleCurrentCycles,
        .wPositionCycles = wPositionCycles,
        .wClarkeCycles = wAlgoCycles,
        .wCommitCycles = wCommitCycles,
        .wEntryCycles = wEntryCycles,
        .wValidFlags = MOTOR_HF_PROFILE_VALID_TOTAL |
                       MOTOR_HF_PROFILE_VALID_SAMPLE_CURRENT |
                       MOTOR_HF_PROFILE_VALID_POSITION |
                       MOTOR_HF_PROFILE_VALID_CLARKE |
                       MOTOR_HF_PROFILE_VALID_COMMIT |
                       MOTOR_HF_PROFILE_VALID_ENTRY,
        .eResult = FOC_RESULT_OK,
    };
#endif
    motor_private_exit(impl, s);

    return FOC_RESULT_OK;
}
