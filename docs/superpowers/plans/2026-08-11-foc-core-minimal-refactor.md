# FOC 核心最小化重构方案（motor_impl_t 根精简 + 调用嵌套 实时≤3/Shell≤4 层）

> **状态：设计文档，未动代码。** 确认后按 Task 拆分执行。
> 关联：`2026-08-10-encoder-light-load-sensorless-observer-validation.md`（验证计划，本方案为其清理前置）。

**Goal:**
1. `motor_impl_t` 从根源精简：58 个成员 → ~22 个，删掉双份拷贝、过渡框架、
   事件环、源接口表等死重（§3）。
2. 20 kHz 高频中断 `motor_HighFrequencyStep`（现 500+ 行）精简为 ~130 行、
   一眼看穿的 FOC 核心（§4）。
3. 取消快照/事件环/遥测；波形改为直接读裸变量（§5）。
4. 全项目业务调用链嵌套深度：实时路径 ≤ 3 层，Shell/初始化 ≤ 4 层
   （§1 硬约束）。

**Motivation:**
- 500 行 ISR 里真正的 FOC 数学只有 ~130 行；`motor_impl_t` 58 个成员里约
  一半服务于已不使用的"多位置源切换"框架（开环→观察器过渡、资格判定、
  事件环、快照、profile）。
- 当前验证模式（编码器直连，无感已取消）根本不走这些路径，纯死重。
- 启动无感算法后出现"溢出"（候选原因见 §6），精简后按构造消除。
- 冗余框架下调用逻辑不可读（用户明确反馈）。

---

## 1. 硬约束：调用嵌套 实时 ≤3 层，Shell/初始化 ≤4 层

**定义：**
- **实时热路径**（20 kHz ISR / 1 kHz Clock / 主循环）：业务调用嵌套
  **≤ 3 层**（入口计第 1 层）：入口(1) → 核心编排(2) → 叶子算子(3)。
- **Shell 命令 / 初始化路径**：≤ 4 层（mshell 分发与配置校验天然深，
  非实时，不强压）。

**计层规则：**
- `A → B → C` = 3 层（A 为入口）。
- 函数指针间接调用（`plan->fnXxx(...)`）**计 1 层**，且热路径禁止使用
  （不可读、难以静态审计）。
- `static inline` / 宏展开**不计层**（编译器内联，无实际调用）。
- **叶子算子内部允许直读直写 HAL 寄存器**（内联宏 / CMSIS 访问，不计层）；
  叶子禁止再调用非内联业务函数。
- 框架管道（`SysTick → modus_Clock → foc_app_Clock`、`mwaveform.Step`、
  mshell 分发）不占业务层名额；业务入口统一为：
  - 20 kHz：`foc_app_HighFrequencyISR`
  - 1 kHz：`foc_app_Clock`
  - 主循环：`foc_app_Run`
  - Shell：`cmd_motor` / `cmd_encoder` / `cmd_smo`
- 每个入口函数的 Doxygen `@note` 必须写明"全链路深度 = N（实时 ≤3 /
  Shell ≤4）"，实现时以 §8 审计表逐条核对。

---

## 2. 现状诊断

### 2.1 当前 3 层调用链

```
20 kHz  ADC1_2_IRQHandler (target/stm32g431/stm32g4xx_it.c:81)
  └─ foc_app_HighFrequencyISR                    (foc_app.c:307)
       ├─ motor_HighFrequencyStep                (motor_hf.c:130, 500 行)
       │    采样→Clarke→位置源(接口+机械换算+开环/闭环双分支)
       │    →过渡判定(QUALIFY/BLEND/COMPLETE)→Park→Id/Iq PI→IPark
       │    →SVPWM→写TIM1→状态发布/有效性事件/候选角度/快照/profile
       └─ phase_test_waveform_hf_step            (phase_test.c:189)

1 kHz  SysTick → modus_Clock → foc_app_Clock     (foc_app.c:446)
  ├─ motor_LowFrequencyStep                      (motor_control.c:62, 速度环)
  ├─ as5600_Update                               (I2C 读编码器，缓存给 20 kHz)
  └─ phase_test_waveform_step                    (phase_test.c:144, Push 遥测/快照)

主循环  foc_app_Run                              (foc_app.c:353)
  ├─ motor_RunFSM                                (状态机)
  └─ 心跳日志
```

### 2.2 20 kHz 路径深度审计（现状，实测代码）

| 链 | 深度 | 判定 |
|---|---|---|
| ISR→`motor_HighFrequencyStep`→`plan->fnSampleCurrent`(指针)→`mdi_adc_reconstruct`→`haladc_GetInjected` | 5 | ❌ 超 |
| ISR→`motor_HighFrequencyStep`→`plan->fnSourceStep`(指针)→`encoder_interface_step`→`foc_encoder_Step`→`encoder_raw_to_angle` | 6 | ❌ 严重超 |
| ISR→`motor_HighFrequencyStep`→`plan->fnCommitDuty`(指针)→`mdi_pwm_commit_duty`→TIM1 寄存器写 | 5 | ❌ 超 |
| ISR→`motor_HighFrequencyStep`→`plan->fnModulate`(指针)→`foc_svpwm` | 4 | ✅ 勉强 |
| 1 kHz：Clock→`motor_LowFrequencyStep`→`foc_controller_Step` | 3 | ✅ |
| 主循环：Run→`motor_ObservationStep`→`fnObservationStep`→`smo_step` | 4 | ✅ 边界 |

结论：热路径超深的主因是**函数指针间接层 + 位置源适配器嵌套**；过渡/事件/
快照是死代码量（~300 行），不是深度问题。

---

## 3. 根设计：motor_impl_t 精简（58 → ~22 成员）

### 3.1 现状成员全景（逐项判定）

| 成员 | 用途 | 判定 | 理由 |
|---|---|---|---|
| `tHal` (foc_hal_t) | PWM+ADC+HFIO 三张函数表 | ✅ 保留 | HAL 契约，ISR 直调 `tHal.tHfIo.fnSampleCurrent/fnCommitDuty` |
| `tControlConfig` | Id/Iq/Speed/Position 控制器绑定 + 调制 | 🔧 瘦身 | 删 `tPosition` 绑定（位置模式砍掉）；调制固定 SVPWM |
| `tHfPlan` | 高频执行计划（tIo+源指针+控制器+调制） | ❌ 删除 | `tIo` 是 `tHal.tHfIo` 的**第二份拷贝**；源指针/控制器/调制全部与 impl 其他字段重复；ISR 改直调叶子后无存在意义 |
| `tIdPid`/`tIqPid` | 电流环 PI 实例 | ✅ 保留 | 电流闭环必需 |
| `tSpeedPid` | 速度环 PI 实例 | ✅ 保留 | 速度闭环必需 |
| `tPositionPid` | 位置环 PI 实例 | ❌ 删除 | 位置模式随过渡框架一起砍（验证不用） |
| `tPositionSource` | 初始/目标源接口表 | ❌ 删除 | 角度来源移到 app 层裸变量（§3.5），motor 经 `foc_angle_read()` 叶子直读 |
| `tObservationSource` | 并行观测源接口表 | ❌ 删除 | 观测器（s_tSmo/s_tNlfo）本来就在 app 层静态实例，直接读其输出变量 |
| `tTime` | 毫秒时间接口 | ❌ 删除 | 仅过渡超时/启动延时用；两者删除后无使用者（编码器 cal 用 shell 的 delay_ms） |
| `tSync` | 临界区 enter/exit | ✅ 保留 | 命令邮箱 shell/ISR 并发必需 |
| `tHfCommand` | 命令邮箱（mode+参考值） | ✅ 保留 | ISR 每拍读 |
| `tHfState` | 高频运行时量 | 🔧 瘦身 | 删 `tPositionOutput`/`tObservationOutput`/`aPendingEvents[4]`/`chPendingCount`；留相电流/dq/αβ/duty/电角度/电速度 |
| `tRuntime` | eRunState+wFaults+qVbus | ✅ 保留 | 运行状态与故障 |
| `tPositionConfig` | 极对数/零位/方向 | ✅ 保留 | 编码器机械角→电角度换算必需 |
| `tMechanicalAngle`/`qMechanicalSpeed` | 机械角度/速度发布 | ✅ 保留 | 1 kHz 速度环反馈 |
| `tDefaultOpenLoopSource` | 开环角度源实例 | ❌ 删除 | 开环角度发生器内联 5 行（`angle += speed×period`）进核心 |
| `tCandidateAngle`/`qCandidateSpeed`/`qAngleError`/`qBlendFactor` | 候选源/混合发布 | ❌ 删除 | CandAngle/AngleErr 变 app 层裸变量（波形直读）；qBlendFactor 随过渡删除 |
| `qOpenLoopCommandSpeed` | 开环命令速度 | 🔧 并入 | 移入 `tHfCommand`（开环模式参考） |
| `qHighFrequencyPeriod` | 高频步周期 | ✅ 保留 | 速度换算必需 |
| `qTransitionMinimumConfidence/Speed/AngleError`、`tTransitionStartAngle`、`qTransitionStartSpeed`、`hwTransitionQualificationSamples/BlendSamples/SampleCount`、`wTransitionTimeoutMs` | 过渡家族 9 个 | ❌ 全删 | 无切换，无过渡 |
| `wStartupDelayMs`/`wStartupStartMs` | 启动延时 | ❌ 全删 | 启动直接进 RUNNING（当前验证即如此） |
| `wDiagnosticStartMs`/`bDiagnosticActive` | 诊断模式 | ❌ 全删 | phase_testA/B 保持 FOC_ENABLE_DIAGNOSTIC 门控，不占 impl |
| `wNextEventSequence`/`wEventOverwriteCount`/`atEvents[4]`/`chEventHead`/`chEventCount` | 事件环全家 | ❌ 全删 | 故障改 `tRuntime.wFaults` 裸标志 |
| `chActiveValidFlags`/`chCandidateValidFlags`/`chMechanicalValidFlags` | 有效性标志×3 | ❌ 全删 | 角度有效收敛为 1 个 bool（或直接信任源） |
| `chStartupPhase` | 启动阶段枚举 | 🔧 收敛 | 保留最小 STARTING 态：**只做三相零偏校准**（见 §3.6），完成后自动进 RUNNING；删 QUALIFY/BLEND 等过渡相位 |
| `chPendingCommand`/`bCommandPending` | 命令握手 | ✅ 保留 | start/stop 命令邮箱 |
| `bInitial/Target/ObservationPositionSourceBound` | 源绑定标志×3 | ❌ 删除 | 由 1 个 `chAngleSource`（ENC/OBS/OPENLOOP）替代 |
| `bOuterLoopActive` | 外环激活 | ❌ 删除 | 由 `eControlMode >= MOTOR_CONTROL_SPEED` 推导 |
| `bHighFrequencyStepInProgress`/`bLowFrequencyStepInProgress` | 重入保护 | 🔧 保留 1 个 | 保留高频重入标志；低频步无竞争源可删 |
| `bPwmEnabled` | PWM 使能 | ✅ 保留 | 急停/状态查询 |
| `tProfileSnapshot`（FOC_HF_PROFILE 门控） | 性能快照 | ❌ 删除 | profile 宏保持关闭，字段不再占用 |
| `wMagic` | 魔数校验 | ✅ 保留 | Init 校验 |

### 3.2 删除的根源概念

1. **`tHfPlan` 双份拷贝**：`tHfPlan.tIo` ≡ `tHal.tHfIo`，`tHfPlan.tId/tIq` ≡
   `tControlConfig.tId/tIq`，`tHfPlan.tPositionConfig` ≡ impl 同名字段。
   整个 plan 层是"绑定期预解析"架构的产物，直调叶子后无存在意义。
2. **位置源接口表 ×2 + 开环源实例 + 时间接口**：角度获取全部移出 motor，
   进 app 层 `foc_app_GetAngle()`（§3.5），motor 核心只接收**纯数据注入**，
   不引用任何 app 符号（TDD 隔离，见 §3.5 做法 A）。
3. **过渡家族**：资格阈值×3、起止角速度、样本数×3、超时、启动延时×2。
4. **事件环全家**：记录×4、序号、覆盖计数、头/计数、有效性标志×3。
5. **候选/混合发布**：candidate angle/speed/err/blend 四量 → CandAngle/
   AngleErr 变 app 裸变量，blend 删除。
6. **位置模式 / 诊断模式**：整条链路（tPositionPid、tPosition 绑定、
   MOTOR_CONTROL_POSITION、诊断字段）随验证需求一并收敛。

### 3.3 目标布局（草案，~22 成员）

```c
typedef struct {
    /* 绑定与接口 */
    foc_hal_t    tHal;            /* 采样/提交/急停函数表（ISR 直调叶子） */
    motor_control_runtime_config_t tControlConfig; /* Id/Iq/Speed 绑定 + 调制 */
    foc_pid_t    tIdPid;          /* D 轴电流 PI */
    foc_pid_t    tIqPid;          /* Q 轴电流 PI */
    foc_pid_t    tSpeedPid;       /* 速度环 PI */
    motor_sync_if_t tSync;        /* 命令邮箱临界区 */
    foc_position_config_t tPositionConfig; /* 极对数/零位/方向 */

    /* 命令邮箱（ISR 每拍读） */
    motor_hf_command_t tHfCommand; /* mode + Vd/Vq + Id/Iq + speed ref */

    /* 高频运行时量（ISR 每拍写） */
    motor_hf_state_t   tHfState;   /* 相电流/dq/αβ/duty/电角度/电速度 */

    /* 运行状态与机械量（1 kHz 速度环读） */
    motor_state_t      tRuntime;   /* eRunState + wFaults（+ qVbus 可选） */
    foc_angle_t        tMechanicalAngle;
    foc_scalar_t       qMechanicalSpeed;
    foc_scalar_t       qHighFrequencyPeriod;
    uint32_t           wPositionSampleTimestamp;

    /* 控制与状态 */
    uint8_t  chAngleSource;        /* ENC / OBS / OPENLOOP */
    uint8_t  chPendingCommand;
    bool     bCommandPending;
    bool     bPwmEnabled;
    bool     bHighFrequencyStepInProgress;
    uint32_t wMagic;
} motor_impl_t;
```

### 3.4 连锁类型精简

| 类型 | 精简动作 |
|---|---|
| `motor_run_config_t` | 删 `ptInitialPositionSource`/`ptTargetPositionSource`/`ptObservationPositionSource`/`qInitialAngle`/`qAcceleration`/`qPositionReference`；加 `chAngleSource`（ENC/OBS/OPENLOOP） |
| `motor_config_t` | 删 transition 家族 6 字段、`wStartupDelayMs`、`tTime`；保留 `tHal`/`tPosition`/`qHighFrequencyPeriod`/`qLowFrequencyPeriod` |
| `motor_control_config_t`/`runtime_config_t` | 删 `tPositionParams`/`tPosition` 绑定；`eModulation` 保留枚举但只实现 SVPWM（或宏裁剪） |
| `motor_event_type_e`/`motor_event_record_t`/`motor_hf_pending_event_t`/`motor_transition_update_t` | 全删 |
| `motor_hf_plan_t`/`motor_hf_command_t`/`motor_hf_state_t`/`motor_hf_frame_t` | plan 删；command 并入 `qOpenLoopCommandSpeed`；state 删输出/事件槽；frame 精简为纯数学 scratch |
| `motor_handle_t` 存储 | impl 缩小后 `MOTOR_HANDLE_STORAGE_SIZE`（896）可降到 ~640，同步改测试——**可选**，默认先保持原尺寸低风险 |

### 3.5 角度来源职责边界（纯数据注入，做法 A）

**核心原则（TDD 隔离）：** `foc/motor/` 层禁止引用 `foc_app.c` 的任何符号
（静态/全局变量、函数）。主机测试（tests/foc/）不链接 foc_app.c，若
motor_hf.c 直接读 s_tEncoder/s_tSmo 会破坏模块独立性导致无法链接。
采用用户建议的**做法 A（入参注入）**：

```c
/* foc/motor 层：纯数学闭环，零 app 依赖，输入是数据不是符号 */
typedef struct {
    foc_angle_t  tElectricalAngle;  /* 电角度（BAM32），已含极对数/零位换算 */
    foc_scalar_t qElectricalSpeed;  /* 电速度（pu），1 kHz 速度环反馈 */
    bool         bValid;            /* 角度有效标志 */
} foc_angle_input_t;

foc_result_t foc_core_step(motor_handle_t *ptMotor,
                           const foc_angle_input_t *ptAngle); /* TDD 可注入 */
```

```c
/* foc/app 层：入口编排，负责角度获取（直读+外推+换算） */
void foc_app_HighFrequencyISR(void)          /* 入口(1) */
{
    foc_angle_input_t tAngle = {0};
    foc_app_GetAngle(&tAngle);               /* (2) 直读/外推/换算 */
    (void)foc_core_step(&s_tMotor, &tAngle); /* (2) 纯数学闭环 */
    mwaveform.Step();                        /* (2) 框架，不计层 */
}
```

**app 层职责（foc_app_GetAngle，按 chAngleSource 选源）：**
- 编码器：读 `s_tEncoder` 缓存（1 kHz 刷新）→ **20 kHz 速度外推**（§3.7）
  → 机械→电角度换算（`θ_elec = θ_mech × Pp + offset`）。
- 观测器：读 `s_tSmo.tAngle`（主循环刷新，观测器本身带滤波，无需外推）。
- 开环：`θ += qOpenLoopCommandSpeed × period`（20 kHz 积分）。

**TDD 覆盖边界：** `foc_core_step`（注入合成角度输入）与
`foc_encoder_ExtrapolateAngle`（外推数学，纯模块函数）在主机测试覆盖；
`foc_app_GetAngle` 是 app 薄壳（选源+换算），不参与主机链接。

### 3.6 上电三相零偏校准归属（状态机收敛后）

- **保留最小 STARTING 态，唯一职责 = 三相零偏校准**：状态机为
  IDLE → STARTING(校准) → RUNNING → FAULT。STARTING 期间在 20 kHz
  采样路径累计 512 拍 ADC 原始值（复用现有 `g_wCalibStartTrigger` +
  `mdi_adc_reconstruct` 累加逻辑，~25.6 ms），算得 `wOffsetU/V/W` 后
  自动进 RUNNING 并使能 PWM。
- 不放在 `motor_Init()`/MDI 初始化：上电到首次启动间隔未知，零偏随
  温度/电源漂移，启动时校准最贴近实际工况。
- **保留 `motor_GetCurrentCalibration()`**；`motor_Start()` 增加零偏
  合法性校验：`!bIsCalibrated` 或偏移超出 [20000, 60000]（现有钳位区间）
  则拒绝启动（防零偏漂移炸机）。

### 3.7 编码器 20 kHz 直读：角度外推设计（重点）

**问题：** 编码器 1 kHz I2C 采样缓存，20 kHz ISR 若纯静态直读，则连续
20 拍角度不变，形成阶梯/滞后：50 电 turn/s 下 1 ms 转过 18° 电角度，
电流环承受最高 18° 相位滞后与阶梯转矩脉动。

**方案：** 基于 1 kHz 测得并经低通滤波的速度做**拍内线性外推**：

```text
θ_hf = θ_cached + ω_filtered × (ticks_since_sample × T_hf)
```

- 外推系数**只在 1 kHz 新样本到达时更新**（ω 用 foc_encoder 已滤波的
  `qSpeed`，α=0.25），20 kHz 只做乘法累加，不引入新噪声源。
- **低速保护阈值：** `|ω| < 1 e-turn/s` 时保持角度不外推（量化噪声
  主导，外推反而抖动）——即此前"取消外推"的教训，仅限低速区间。

**误差预算（50 e-turn/s，1 kHz 采样）：**

| 方案 | 最大角度误差（电角度） |
|---|---|
| 纯静态直读（阶梯） | 18°（拍间全程滞后） |
| 外推（速度量化 ±1 count → ±1.7 e-turn/s ≈ ±3.4%） | ≤ 0.61°（拍末最差） |

外推残余误差比阶梯小 ~30 倍；且 50 e-turn/s 是已验证的速度环稳定工作点，
滤波后速度平稳，此前外推放大噪声的问题（发生在极低速）由保护阈值规避。

**实现落点：** `foc/observer/foc_encoder.c` 新增纯函数
`foc_encoder_ExtrapolateAngle(enc, qDt, &qAngle)`（模块内可单测）；
app 层维护 20 kHz 拍计数器计算 `qDt = Δticks × T_hf`。

---

## 4. 目标调用链与最小核心

### 4.1 目标调用链（深度标注：实时 ≤3）

```
20 kHz  ADC1_2_IRQHandler
  └─ foc_app_HighFrequencyISR        (1)  ← 入口，角度获取+核心+波形
       ├─ foc_app_GetAngle()         (2)  → foc_encoder_ExtrapolateAngle (3)
       │                                 直读缓存+速度外推+机械→电换算
       ├─ foc_core_step(m,&angle)    (2)  ← 唯一核心编排，~130 行
       │    ├─ foc_sample_current()  (3)  内含 ADC 直读(内联)+归一化
       │    ├─ foc_clarke()          (3)
       │    ├─ foc_park() / foc_pi() / foc_ipark() / foc_svpwm()  (3)
       │    └─ foc_commit_duty()     (3)  内含 TIM1 寄存器写(内联)
       └─ mwaveform.Step()           (2)  框架，不计层

1 kHz  foc_app_Clock                 (1)
  └─ foc_speed_loop_step()           (2) → foc_controller_Step (3)
  └─ as5600_Update()                 (2) → I2C 读 (3)

主循环  foc_app_Run                  (1)
  ├─ motor_RunFSM()                  (2)
  └─ motor_ObservationStep()         (2) → smo_step / nlfo_step (3)

Shell  cmd_motor                     (1)
  └─ foc_app_Start()                 (2) → motor_Start() (3) → validate/save (4)  ← Shell 例外
```

**关键决策：**
- ISR 内不再经过 `motor_HighFrequencyStep` 长链；核心步骤作为
  `foc_app_HighFrequencyISR` 的直属叶子直调（去掉函数指针间接层）。
- 角度以**纯数据注入**（`foc_angle_input_t`）传给 `foc_core_step`，motor
  层零 app 符号依赖（TDD 隔离，§3.5 做法 A）。
- 删掉 QUALIFY/BLEND/COMPLETE 过渡状态机（当前模式 initial==target，
  transition_required 恒假，是死代码）。
- 速度环（1 kHz）、观测器（主循环）、编码器 I2C（1 kHz）保持原层级。

### 4.2 最小 20 kHz 核心（伪代码，目标 ~130 行）

```c
/* @note 全链路深度：ISR(1) → GetAngle/core_step(2) → 叶子(3)，≤3。
 * 叶子内含内联 HAL 直读直写（不计层），禁止再调非内联业务函数。 */
void foc_app_HighFrequencyISR(void)          /* 入口(1) */
{
    foc_angle_input_t tAngle = {0};
    foc_app_GetAngle(&tAngle);               /* (2) 直读+外推+换算（§3.7）*/
    (void)foc_core_step(&s_tMotor, &tAngle); /* (2) 纯数学闭环 */
    mwaveform.Step();                        /* 框架，不计层 */
}

foc_result_t foc_core_step(motor_handle_t *ptMotor,
                           const foc_angle_input_t *ptAngle)  /* 核心(2) */
{
    /* 1. 三相电流采样（叶子 3: foc_sample_current，内联 ADC 直读+归一化）*/
    /* 2. Clarke → Iα Iβ（叶子 3）*/
    /* 3. 使用注入角度 ptAngle（已外推/换算，核心不再读任何源）*/
    /* 4. sin/cos + Park → Id Iq（叶子 3）*/
    /* 5. Id/Iq PI → Vd Vq（电压开环=直接取参考）（叶子 3）*/
    /* 6. IPark → Vα Vβ（叶子 3）*/
    /* 7. SVPWM → 三相占空比（叶子 3）*/
    /* 8. 提交占空比（叶子 3: foc_commit_duty，内联 TIM1 寄存器写）*/
    /* 9. 裸变量写回：Iu/Iv/Iw/Id/Iq/Angle/Speed/Vd/Vq/CandAngle/AngleErr */
}
```

### 4.3 数据流：裸变量谁写谁读

```
写（20 kHz ISR 每拍）          读
─────────────────────────────  ─────────────────────────
s_hwIu/Iv/Iw (int16)      ──→  mwaveform 直读 / shell
s_fId/s_fIq (float)       ──→  mwaveform 直读
s_fAngle/s_fSpeed (float) ──→  mwaveform 直读 / 1 kHz 速度环
s_fVd/s_fVq (float)       ──→  mwaveform 直读
s_fCandAngle/s_fAngleErr  ──→  mwaveform 直读（观测器输出，主循环写）
```

1 kHz 速度环只**读** `s_fSpeed`、**写** Iq 参考；主循环观测器只**读**
αβ 电流/电压裸变量、**写**候选角度。无锁：写侧单生产者（ISR/Clock/Run
各自独占自己的变量），读侧 volatile 采样。

---

## 5. 波形直读变量（AddVariable）

**机制已原生支持，无需新代码：** `mwaveform.AddVariable(name, scale, &var,
type)` 注册**变量地址**；`mwaveform_Step()` 每次调用直接 `volatile` 读该
内存（`modus/src/mdebug/mwaveform.c:260-276`），不需要 `Push`、不需要锁、
不需要快照拷贝。20 kHz 满速采样。

**通道表（src/userconfig.h 现有 MAX_CHANNELS=9，够用）：**

| 通道 | 绑定变量 | 类型 | Scale |
|---|---|---|---|
| Iu / Iv / Iw | `s_hwIu/Iv/Iw` | RAW int16 | 1.0 |
| Id / Iq | `s_fId` / `s_fIq` | FLOAT | 1000 |
| Angle | `s_fAngle` | FLOAT | 1000 |
| Speed | `s_fSpeed` | FLOAT | 1000 |
| Vd / Vq | `s_fVd` / `s_fVq` | FLOAT | 1000 |
| CandAngle | `s_fCandAngle` | FLOAT | 1000 |
| AngleErr | `s_fAngleErr` | FLOAT | 1000 |

**删除：** `phase_test_waveform_step` 的 `motor_GetSnapshot` /
`motor_GetTelemetry` + `mwaveform.Push`（phase_test.c:144-185）；
`SnapshotFeed` / `SnapshotStart` 调用；`MWAVEFORM_SNAPSHOT_ENABLE=0`
（src/userconfig.h:41）。

---

## 6. 文件改动清单

### 类型层（根设计，Task A）
- `foc/motor/motor_private.h` — `motor_impl_t` 58→~22 成员（§3.3）；
  删事件环/过渡/源接口/plan 相关内联与声明
- `foc/motor/motor_types.h` — 删事件枚举/记录、run_config 瘦身、
  config 删 transition 字段；`MOTOR_HANDLE_STORAGE_SIZE` 可选降
- `foc/motor/motor_control_types.h` — 删 Position 绑定、位置模式枚举值
- `foc/motor/motor_fsm.c` — 删 AppendEvent/DrainPendingEvents/启动过渡；
  状态收敛 IDLE/RUNNING/FAULT；validate/save 按新 run_config
- `foc/motor/motor.c` — 删事件环/快照/遥测结构；公共 API 改读裸变量薄壳
- `foc/motor/motor_control.c` — 删位置环分支与 bOuterLoopActive；
  速度环、`motor_ObservationStep` 保留
- `foc/motor/motor_hf_private.h` — plan 删；command/state/frame 瘦身

### 核心 ISR（Task B）
- `foc/motor/motor_hf.c` — 重写核心 500→~130 行（§4.2）；删过渡/事件/
  快照/profile；`foc_core_step` 接收注入角度（§3.5 做法 A）
- `foc/observer/foc_encoder.c` — 新增 `foc_encoder_ExtrapolateAngle()`
  纯函数（§3.7 外推 + 低速保护阈值），可单测
- `foc/app/foc_app.c` — `foc_app_HighFrequencyISR` 直调 GetAngle+核心；
  `foc_app_GetAngle()` 直读/外推/换算；心跳/状态读裸变量；删看门狗
  死代码；`motor enc/obs/start` 按新 run_config 填 `chAngleSource`

### 波形（Task C）
- `foc/app/phase_test.c` — 波形全改 `AddVariable`；删 Push/快照/遥测
- `src/userconfig.h` — `MWAVEFORM_SNAPSHOT_ENABLE=0`

### 不动
- `foc/math/`（Clarke/Park/IPark/PI/SVPWM 数学叶子）
- `foc/observer/`（SMO/NLFO 本体、foc_encoder 本体、s_tEncoder 缓存）
- `peripheral/`（haladc/halopamp/haltim1，作为叶子保留）
- `foc_hal_mdi_adapter.c` 的归一化/零偏逻辑（挪到叶子层，逻辑不变）

---

## 7. 溢出修复映射

| 精简动作 | 消除的溢出源 |
|---|---|
| 删每拍有效性事件 + 事件环 | 事件环洪泛（seq 爆涨） |
| `MWAVEFORM_SNAPSHOT_ENABLE=0` + 删 SnapshotFeed | 快照帧缓冲 + RTT 拥塞 |
| 核心栈帧减半（删过渡局部量、frame 瘦身） | ISR 栈溢出风险 |
| 删事件环/快照/遥测/plan/源接口结构体 | RAM 回收（21072 → 预计 <19 KB） |

---

## 8. 目标态深度审计表（实现后逐条核对：实时 ≤3，Shell ≤4）

| 入口 | 目标链 | 深度 |
|---|---|---|
| 20 kHz | ISR→core_step→foc_sample_current（内联 ADC 直读，不计层） | 3 ✅ |
| 20 kHz | ISR→core_step→foc_commit_duty（内联 TIM1 写，不计层） | 3 ✅ |
| 20 kHz | ISR→core_step→foc_clarke/park/pi/ipark/svpwm | 3 ✅ |
| 20 kHz | ISR→foc_app_GetAngle→foc_encoder_ExtrapolateAngle（外推） | 3 ✅ |
| 1 kHz | Clock→foc_speed_loop_step→foc_controller_Step | 3 ✅ |
| 1 kHz | Clock→as5600_Update→I2C 读 | 3 ✅ |
| 主循环 | Run→motor_ObservationStep→smo_step | 3 ✅ |
| Shell | cmd_motor→foc_app_Start→motor_Start→validate_run | 4 ✅（例外） |

---

## 9. 验证步骤（每 Task 完成后）

1. 主机侧测试：`mingw32-make -C tests/foc`（观察/变换/外推测试随 API 调整
   同步改，红→绿）。
2. 构建：`mingw32-make BUILD=debug-rel`，`mingw32-make size` 对比 RAM/Flash。
3. 深度审计：按 §8 表格逐链核对源码，入口函数 Doxygen `@note` 标注深度。
4. MISRA/风格自查：显式强转、无符号混合比较、78 字符行宽、
   switch-default/break（按 .agents/skills/embedded-coding/SKILL.md）。
5. 上板（RTT 恢复流程：`.\make.bat auto` 或手动 openocd + rtt_server_manual.cfg）：
   - `motor status`（状态/故障/角度/电流正常）
   - `encoder cal` → `motor enc 0.1 50`（速度闭环稳定，50 e-turn/s）
   - 波形验证角度平滑：Angle 通道无 18° 阶梯（外推生效，§3.7）
   - `motor obs smo` + `motor enc 0.1 50`（并行观测不溢出不震动）
   - `smo` / `encoder`（角度对比，波形直读变量无丢帧）

---

## 10. 任务拆分（checkbox）

### Task A：根设计精简（motor_impl_t + 类型层）
- [ ] **A1** `motor_impl_t` 重排为 §3.3 目标布局（58→~22 成员）
- [ ] **A2** 删 `motor_hf_plan_t`、源接口表×2、开环源实例、`tTime`、过渡家族、
      事件环全家、候选发布、位置/诊断模式字段
- [ ] **A3** `motor_run_config_t`/`motor_config_t`/`motor_control_types.h`
      连锁精简（§3.4），加 `chAngleSource`；新增 `foc_angle_input_t`
- [ ] **A4** `motor_fsm.c`/`motor.c` 删事件环/快照/遥测，公共 API 改裸变量薄壳
- [ ] **A5** 状态收敛 IDLE/STARTING(仅零偏校准)/RUNNING/FAULT（§3.6）；
      `motor_Start()` 增加零偏合法性校验；`motor status` 输出保持可用
- [ ] **A6** `make size` 对比 RAM/Flash 回收量；`_Static_assert` 通过
- [ ] **A7** 工程节奏：**`MOTOR_HANDLE_STORAGE_SIZE` 保持 896U 不变**，
      待 Task A~C 单测+真机全绿后作为收尾统一调整（§3.4 备注）

### Task B：核心 ISR 瘦身（motor_hf.c / motor_hf_private.h / foc_app.c）
- [ ] **B1** `foc_app_HighFrequencyISR` = GetAngle + `foc_core_step(&m,&angle)`
      + mwaveform.Step，删 `plan->fnXxx` 间接层
- [ ] **B2** 删阶段 2.2 开环并行分支 + 阶段 3 过渡/资格/混合状态机
- [ ] **B3** 删阶段 7 有效性事件/候选发布/快照/profile；保留故障急停
- [ ] **B4** `foc_app_GetAngle()`：按 `chAngleSource` 选源 + 机械→电换算；
      编码器路径调用 `foc_encoder_ExtrapolateAngle`（§3.7）
- [ ] **B5** `foc_encoder_ExtrapolateAngle()` 纯函数 + 低速保护阈值
      （|ω|<1 e-turn/s 保持），配主机单测（误差预算 ≤0.61° @50 e-turn/s）
- [ ] **B6** `motor_hf_private.h` 的 command/state/frame 瘦身
- [ ] **B7** 深度审计：20 kHz 路径全部 ≤3（§8 表）

### Task C：波形直读变量（phase_test.c / foc_app.c / userconfig.h）
- [ ] **C1** 裸变量集定义：Iu/Iv/Iw、Id/Iq、Angle、Speed、Vd/Vq、
      CandAngle、AngleErr（§5 通道表）
- [ ] **C2** ISR 写回裸变量；删 `motor_GetSnapshot`/`GetTelemetry` Push
- [ ] **C3** `mwaveform.AddVariable` 全量绑定；删 SnapshotFeed/Start
- [ ] **C4** `MWAVEFORM_SNAPSHOT_ENABLE=0`；心跳改读裸变量

### Task D：回归验证（无感并行观测）
- [ ] **D1** 主机测试更新并 PASS（float + fixed，含外推单测）
- [ ] **D2** 上板：`motor enc 0.1 50` 速度闭环稳定（50 e-turn/s），
      波形 Angle 无阶梯（外推生效）
- [ ] **D3** `motor obs smo` 并行观测：无溢出、无震动、角度对比合理
- [ ] **D4** 全入口深度审计表（§8）逐条核对通过
- [ ] **D5** 收尾：单测+真机全绿后统一调整 `MOTOR_HANDLE_STORAGE_SIZE`
      （896→~640）与 `_Static_assert`，重跑全部测试
- [ ] **D6** 更新本文档与验证计划文档的 Task 状态
