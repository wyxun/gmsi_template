# FOC 极简核心重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将现有通用、多实例、运行时可配置的 FOC 框架重构为面向当前
STM32G431 单电机产品的极简实现，只保留编码器角度、电流环、速度环、SVPWM、
启动校准和必要故障保护，同时保留 float/fixed 编译期数值后端切换。

**Architecture:** 高频 ISR 直接执行“端口采样 → 角度更新 → 纯数学 FOC 核心
→ 端口提交”，硬件通过编译期链接的 `foc_port_*()` 函数接入，不再使用 HAL
函数表、motor opaque handle、执行 plan、位置源接口和运行时控制器接口。
全系统只保留一份 `foc_runtime_t`，Shell、1 kHz 速度环和波形均读取该状态。

**Tech Stack:** STM32G431、C11、MODUS、AS5600、float/fixed FOC、BAM32、PI、
SVPWM、MStudio Waveform、host clang tests、OpenOCD/RTT。

---

## 0. 范围锁定

本计划是产品级极简重构，不保持旧 FOC 框架的源码兼容、ABI 兼容或功能兼容。

### 0.1 本次必须保留

- 单个 STM32G431 电机实例。
- `FOC_NUMERIC_FLOAT` / `FOC_NUMERIC_FIXED` 编译期数值后端切换。
- 核心、编码器、PI、SVPWM 和端口归一化代码必须同时支持两个后端。
- AS5600 编码器机械角度、机械速度和 20 kHz 拍内角度外推。
- 机械角度到电角度的极对数、方向和零位换算。
- 电压开环，用于静态锁角、方向检查和硬件初试。
- Id/Iq 双 PI 电流环。
- 1 kHz 速度 PI，输出 Iq 参考。
- SVPWM。
- 三相电流零偏校准和三相电流采样。
- IDLE、CALIBRATING、RUNNING、FAULT 四态生命周期。
- START、STOP、CLEAR_FAULT 最小命令处理。
- PWM 立即急停，以及采样、角度、计算和提交失败时进入 FAULT。
- 最多 9 路实时波形。
- host 纯数学测试和 STM32G431 真机验证。

### 0.2 本次明确删除或停止构建

- `motor_handle_t`、`motor_impl_t` 和固定 896/960/1280 字节私有存储。
- 多电机实例能力。
- `foc_hal_t`、`foc_hf_io_if_t` 及所有运行时硬件函数表。
- `motor_hf_plan_t` 和绑定期 plan resolver。
- `foc_controller_if_t` 运行时控制器接口；Id/Iq/Speed 固定使用 PI。
- Position 模式和位置 PID。
- Initial/Target/Observation 位置源接口。
- 开环到观测器的 QUALIFY/BLEND/COMPLETE 切换流程。
- SMO、NLFO、HFI、Hall 在当前固件中的构建和运行入口。
- 事件环、pending event、事件序号和事件覆盖计数。
- motor snapshot、telemetry 和 profile snapshot。
- SPWM、三次谐波 SPWM 和运行时调制枚举。
- MTPA、弱磁、DOB、前馈、齿槽补偿、实验辨识和诊断 FSM。
- 运行时切换数值后端；后端只允许在编译期选择。

源文件可以在核心切换成功后删除；在切换前先从 `foc/foc.mk` 排除，保证每个
阶段均可构建和回退。

### 0.3 数值后端约束

- float 版用于 STM32G431 和波形调试。
- fixed 版必须能在没有 FPU 的 Cortex-M 芯片上编译；高频路径不得调用
  `foc_to_float()`、`printf("%f")`、libm 或其他软浮点辅助函数。
- 所有核心常量、PI 参数和端口缩放都使用 `FOC_SCALAR()`、`foc_gain_t`
  和现有数值 API；禁止在核心代码中写 float-only 临时变量。
- fixed 版关闭依赖 `float *` 的 AddVariable 波形通道，不能把 Q16.15
  地址伪装成 FLOAT。fixed 版仍必须通过核心、编码器和生命周期测试。
- float/fixed 的算法边界、角度 wrap、饱和和故障语义必须一致。

### 0.4 不属于本计划

- 真正无感运行和编码器到无感观察器的在线切换。
- 多电机、多芯片 FOC 通用库。
- 运行中切换控制模式或控制器类型。
- 在线修改 PI、极对数、采样拓扑等结构性参数。
- AT32F413 FOC 适配。AT32 其他业务不受影响，但不再构建极简 FOC。

如果上述能力以后重新成为产品需求，应以独立模块重新设计，不能恢复为
`motor_impl_t` 内的可选字段。

---

## 1. 目标架构

### 1.1 文件边界

```text
foc/
├─ foc_types.h                    公共组合、校准和核心传输类型
├─ math/                         数值、BAM32、三角函数；保留
├─ middleware/
│  ├─ foc_core.h                 极简核心输入、命令、状态和 API
│  └─ foc_core.c                 两后端共用的纯数学闭环
├─ control/
│  ├─ foc_pid.h                  具体 PI 实例；保留
│  └─ foc_pid.c
├─ modulation/
│  ├─ foc_modulation.h           只导出 foc_svpwm
│  └─ foc_modulation.c           只构建 SVPWM
├─ observer/
│  ├─ foc_encoder.h              无位置源适配器的编码器状态
│  └─ foc_encoder.c              新样本更新、速度滤波、角度外推
├─ hal/
│  └─ foc_port.h                 编译期硬件端口 API，无函数指针
└─ app/
   ├─ foc_app.h                  Start/Stop/ISR/Clock/状态查询
   └─ foc_app.c                  唯一runtime、四态生命周期和9路波形

peripheral/stm32g431/
└─ foc_port.c                    ADC、校准、PWM 提交、使能、急停
```

### 1.2 依赖方向

```text
foc_app ──────→ foc_encoder
   │
   ├─────────→ foc_core ─→ math + foc_pid + foc_svpwm
   │
   └─────────→ foc_port ─→ STM32G431 MDI/芯片驱动
```

约束：

- `foc_core.c` 不包含 `foc_app.h`、`foc_port.h`、MDI 或厂商头文件。
- `foc_core_step()` 不读 ADC、不写 PWM、不读全局变量。
- 只有 `peripheral/stm32g431/foc_port.c` 可以访问芯片和 MDI 实现。
- `foc_app.c` 是唯一运行状态所有者。
- 不通过把 motor 字段搬成另一套 app 镜像变量来“缩小结构体”。
- 波形直接绑定 `foc_runtime_t.tCore` 中的运行量。

### 1.3 高频调用链

```text
ADC1_2_IRQHandler
└─ foc_app_HighFrequencyISR                    业务入口(1)
   ├─ foc_app_ConsumeCommand                   static inline
   ├─ CALIBRATING:
   │  └─ foc_port_CurrentCalibrationStep       直接链接函数(2)
   └─ RUNNING:
      ├─ foc_port_CurrentSample                直接链接函数(2)
      ├─ foc_app_AngleStep                     static inline
      │  └─ foc_encoder_Step                   直接链接函数(2)
      ├─ foc_core_step                         核心编排(2)
      │  ├─ foc_clarke                         叶子(3)
      │  ├─ foc_angle_sincos                   叶子(3)
      │  ├─ foc_park_cached                    叶子(3)
      │  ├─ foc_pid_Step ×2                    叶子(3)
      │  ├─ foc_ipark_cached                   叶子(3)
      │  └─ foc_svpwm                          叶子(3)
      ├─ foc_port_DutyCommit                   直接链接函数(2)
      └─ mwaveform.Step                        调试框架
```

“业务深度不超过 3”只用于保持代码可读，不作为实时性能证明。是否内联、实际
栈深和周期占用以编译器 map、`.su`、反汇编和 DWT 测量为准。

---

## 2. 极简数据模型

### 2.1 运行状态和控制模式

在 `foc/foc_types.h` 定义运行状态、控制模式、`foc_ab_t`、
`foc_dq_t`、`foc_duty_abc_t`、`foc_adc_calib_t`、core input 和
command。`foc_modulation.h` 与 `foc_core.h` 都包含该文件，禁止两者
互相包含。`foc_core_state_t` 因包含具体 PI 实例，定义在
`foc/middleware/foc_core.h`：

```c
typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_CALIBRATING,
    FOC_STATE_RUNNING,
    FOC_STATE_FAULT
} foc_run_state_e;

typedef enum {
    FOC_MODE_VOLTAGE = 0,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED
} foc_control_mode_e;

typedef enum {
    FOC_COMMAND_NONE = 0,
    FOC_COMMAND_START,
    FOC_COMMAND_STOP,
    FOC_COMMAND_CLEAR_FAULT
} foc_command_e;
```

### 2.2 核心输入、命令和状态

```c
typedef struct {
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
    foc_angle_t  tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool         bAngleValid;
} foc_core_input_t;

typedef struct {
    foc_control_mode_e eMode;
    foc_dq_t           tVoltageReference;
    foc_dq_t           tCurrentReference;
    foc_scalar_t       qSpeedReference;
} foc_core_command_t;

typedef struct {
    foc_pid_t      tIdPi;
    foc_pid_t      tIqPi;
    foc_ab_t       tCurrentAlphaBeta;
    foc_dq_t       tCurrent;
    foc_dq_t       tVoltage;
    foc_ab_t       tVoltageAlphaBeta;
    foc_duty_abc_t tDuty;
    foc_angle_t    tElectricalAngle;
    foc_scalar_t   qElectricalSpeed;
    foc_scalar_t   qIu;
    foc_scalar_t   qIv;
    foc_scalar_t   qIw;
} foc_core_state_t;
```

`foc_core_step()` 使用调用者传入的 command，不拥有命令邮箱：

```c
foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput);
```

规则：

- VOLTAGE 直接使用 `tVoltageReference`。
- CURRENT 和 SPEED 都执行 Id/Iq PI；SPEED 的 Iq 参考由 1 kHz 速度环更新。
- `bAngleValid == false` 立即返回 `FOC_RESULT_SAFETY`。
- 任一数学步骤失败立即返回，不产生新的 duty。
- `foc_core_Reset()` 清零 Id/Iq PI、运行量并设置 duty 为 0.5/0.5/0.5。

### 2.3 唯一运行上下文

在 `foc/app/foc_app.c` 文件内定义，不对外公开布局：

```c
typedef struct {
    foc_core_state_t   tCore;
    foc_core_command_t tCommand;
    foc_pid_t           tSpeedPi;
    foc_encoder_t      tEncoder;
    foc_adc_calib_t    tCalibration;
    foc_angle_t        tMechanicalZero;
    foc_scalar_t       qMechanicalSpeed;
    foc_scalar_t       qSpeedReference;
#if defined(FOC_NUMERIC_FLOAT)
    float              fElectricalAngleTurns;
#endif
    uint32_t           wFaults;
    uint16_t           hwCalibrationTicks;
    uint8_t            chPolePairs;
    uint8_t            chPendingCommand;
    foc_run_state_e    eState;
    bool               bDirectionInverted;
    bool               bPwmEnabled;
} foc_runtime_t;

static foc_runtime_t s_tFoc;
```

不得再定义 `s_fId`、`s_fIq`、`s_fAngle` 等重复镜像。float 版的 Angle 因
BAM32 无法按 FLOAT 注册，只保留 runtime 内一个展示字段；fixed 版不创建该
float 字段。速度 PI 属于 1 kHz app 编排，不放入 20 kHz 纯数学核心。

---

## 3. 编译期硬件端口

### 3.1 端口 API

新建 `foc/hal/foc_port.h`：

```c
typedef enum {
    FOC_CALIBRATION_BUSY = 0,
    FOC_CALIBRATION_COMPLETE,
    FOC_CALIBRATION_FAILED
} foc_calibration_state_e;

void foc_port_Init(void);
void foc_port_CurrentCalibrationBegin(void);
foc_calibration_state_e foc_port_CurrentCalibrationStep(
    foc_adc_calib_t *ptCalibration);
foc_result_t foc_port_CurrentSample(
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput);
foc_result_t foc_port_DutyCommit(const foc_duty_abc_t *ptDuty);
foc_result_t foc_port_PwmEnable(bool bEnable);
void foc_port_EmergencyStop(void);
```

这些是普通直接链接函数，不包含上下文指针、ABI 版本或回调表。当前产品只有
一个功率级，因此端口内部允许使用 STM32G431 的静态硬件资源。

### 3.2 校准端口行为

- `CalibrationBegin()` 清空 U/V/W 64 位累加值和样本计数。
- `CalibrationStep()` 每次 ISR 读取一组三相 ADC 原始值。
- 累计 512 组后求平均。
- STM32G431 左对齐 ADC 的三相偏移均必须位于 `[20000, 60000]`。
- 任一偏移越界返回 FAILED，不得自动替换为 32768 后报告成功。
- 校准期间 U/V/W PWM 输出必须关闭，但 PWM 定时器和 ADC 触发继续运行。
- `CurrentSample()` 只做原始值读取、减零偏、归一化和三相重构。
- `DutyCommit()` 只转换并一次性写入三相预装载寄存器。
- `EmergencyStop()` 必须在任意上下文立即关闭功率输出。

---

## 4. 生命周期和并发

### 4.1 状态迁移

```text
IDLE
 └─ START → CALIBRATING
               ├─ 512 样本完成且偏移有效
               │    → 清 PI → 提交中性 duty → 使能 PWM → RUNNING
               ├─ 超过 100 ms 未完成 → FAULT
               └─ 偏移非法 → FAULT

RUNNING
 ├─ STOP → 立即关闭 PWM → IDLE
 └─ 采样/角度/计算/提交错误 → 立即关闭 PWM → FAULT

FAULT
 └─ CLEAR_FAULT（确认 PWM 已关闭）→ IDLE
```

`foc_app_Start()` 不得以 `!bIsCalibrated` 拒绝启动。每次 START 都重新校准，
只有 CALIBRATING 成功后才能进入 RUNNING。

### 4.2 并发规则

- 高频 ISR 是 state、fault、核心反馈和 duty 的唯一写者。
- Shell/按键只写一个 `chPendingCommand`；写入时使用项目现有全局中断守卫。
- 1 kHz Clock 读取速度并计算 Speed PI，更新 `tCurrentReference.qQ` 时使用同一
  中断守卫，保证 ISR 不读取半更新命令。
- STOP 不等待 ISR：`foc_app_Stop()` 先调用 `foc_port_EmergencyStop()`，再投递
  STOP，使软件状态在下一个 ISR 边界收敛。
- CLEAR_FAULT 只允许在 PWM 已关闭且状态为 FAULT 时执行。
- 32 位单变量读写虽然在 Cortex-M4 上原子，但不把它当作多字段一致性保证。

校准超时不调用毫秒时钟：`hwCalibrationTicks` 在每个20 kHz ISR递增，
达到2000拍即100ms。完成、失败、STOP或重新START时清零。

### 4.3 对外最小API

`foc/app/foc_app.h` 只公开命令、引用和状态读取：

```c
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);
void foc_app_HighFrequencyISR(void);

foc_result_t foc_app_Start(const foc_core_command_t *ptCommand);
void foc_app_Stop(void);
foc_result_t foc_app_ClearFault(void);
foc_result_t foc_app_SetVoltageReference(foc_scalar_t qD,
                                         foc_scalar_t qQ);
foc_result_t foc_app_SetCurrentReference(foc_scalar_t qD,
                                         foc_scalar_t qQ);
foc_result_t foc_app_SetSpeedReference(foc_scalar_t qMechanicalTurnPerSecond);
foc_result_t foc_app_GetStatus(foc_status_t *ptStatus);
```

- Start只在IDLE接受，复制完整command并投递START。
- 控制模式在一次RUNNING期间固定，不能在线切换。
- Set接口只在对应模式接受，并在临界区内更新引用。
- Speed参考单位固定为mechanical turn/s；1 kHz速度PI输出Iq参考。
- Stop是安全接口，任何状态调用都先执行硬件急停。

---

## 5. 编码器角度路径

### 5.1 数据流

```text
1 kHz Clock
└─ as5600_Update()
   └─ 更新 raw angle / magnet / sequence，sequence 最后写

20 kHz ISR
└─ as5600_GetSample()              无锁双读 sequence，得到一致快照
   └─ foc_encoder_Step()
      ├─ sequence 改变：更新机械角和滤波机械速度
      ├─ sequence 未变：累计距样本时间
      └─ 根据速度执行拍内外推
         └─ 机械角 × 极对数 + offset → 电角度
```

删除 `foc_encoder_source_adapter_t`、`foc_encoder_read_sample_fn_t` 和
`foc_encoder_PositionSourceInterface()`。

### 5.2 编码器极简 API

```c
typedef struct {
    foc_scalar_t qSpeedFilterAlpha;     /* 速度低通滤波系数 [0, 1] */
    uint16_t     hwInvalidTimeout;      /* 无新样本超时，单位 tick */
    uint8_t      chPolePairs;           /* 极对数（外推门限） */
    foc_scalar_t qHighFrequencyPeriod;  /* 高频周期 (s) */
} foc_encoder_params_t;

typedef struct {
    uint16_t hwRawAngle;    /* 当前 12 位 AS5600 角度码 */
    uint32_t wSequence;     /* 样本序号；不变表示无新样本 */
    bool     bMagnetOk;     /* 磁铁位置在正常范围 */
} foc_encoder_sample_t;

typedef struct {
    foc_angle_t  tMechanicalAngle;      /* 输出机械角度 */
    foc_scalar_t qMechanicalSpeed;      /* 输出机械速度 (turn/s) */
} foc_encoder_output_t;

typedef struct {
    foc_encoder_params_t tParams;       /* 参数（Init 后不变） */
    foc_angle_t  tMechanicalAngle;      /* 状态：机械角度 */
    foc_scalar_t qMechanicalSpeed;      /* 状态：机械速度 (turn/s) */
    uint32_t     wLastSequence;         /* 状态：上次消费的样本序号 */
    uint16_t     hwLastRawAngle;        /* 状态：上次 12 位原始角度码 */
    uint16_t     hwTicksSinceSample;    /* 状态：距上次新样本的 tick 数 */
    bool         bInitialized;          /* 状态：是否已获得首个样本 */
    bool         bValid;                /* 状态：输出是否有效 */
} foc_encoder_t;

foc_result_t foc_encoder_Step(foc_encoder_t *ptEncoder,
                              const foc_encoder_sample_t *ptSample,
                              foc_encoder_output_t *ptOutput);
```

按 L1.1 函数参数 ≤4 约束，极对数与高频周期作为生命周期内不变的配置
下沉到 `foc_encoder_params_t`（Init 时固定），样本输入与输出分别打包
为 `foc_encoder_sample_t` / `foc_encoder_output_t`，Step 保持三参数。

实现约束：

- 速度输出单位固定为 mechanical turn/s，不能保留 turn/tick 中间单位给调用者。
- 新样本速度先用 `alpha=0.25` 一阶低通。
- `|mechanical speed × pole pairs| < 1 electrical turn/s` 时不外推。
- 其他速度下，按 `ticks_since_sample × T_hf` 外推机械角。
- 磁铁失效或样本超时立即 `bValid=false`；RUNNING 时无有效角度进入 FAULT。
- 50 electrical turn/s、1 kHz 新样本时，稳态拍末外推量约 18°；测试验证
  外推方向、wrap 和固定速度下误差，不把 0.61° 当作加减速过程的保证。

---

## 6. 波形与状态读取

`MWAVEFORM_MAX_CHANNELS` 保持 9，固定注册以下 9 路：

| ID | 名称 | 地址 | 类型 | Scale |
|---:|---|---|---|---:|
| 0 | Iu | `s_tFoc.tCore.qIu` | FLOAT | 1000 |
| 1 | Iv | `s_tFoc.tCore.qIv` | FLOAT | 1000 |
| 2 | Iw | `s_tFoc.tCore.qIw` | FLOAT | 1000 |
| 3 | Id | `s_tFoc.tCore.tCurrent.qD` | FLOAT | 1000 |
| 4 | Iq | `s_tFoc.tCore.tCurrent.qQ` | FLOAT | 1000 |
| 5 | Angle | 单独的 float 展示值 | FLOAT | 1000 |
| 6 | Speed | `s_tFoc.tCore.qElectricalSpeed` | FLOAT | 1000 |
| 7 | Vd | `s_tFoc.tCore.tVoltage.qD` | FLOAT | 1000 |
| 8 | Vq | `s_tFoc.tCore.tVoltage.qQ` | FLOAT | 1000 |

BAM32 `foc_angle_t` 不能作为 FLOAT 地址直接注册。ISR 在核心成功后只做一次
`foc_angle_to_turns()`，写入 `s_tFoc.fElectricalAngleTurns`；这是唯一允许的
展示变量，不再复制其他运行量。波形注册放在 `foc_app.c`，因为
`s_tFoc` 保持文件私有；不允许其他文件 `extern` 私有 runtime。

每次 `mwaveform.AddVariable()` 都检查返回值；任一路返回 `0xFF` 时停止启动波形
并记录错误。删除所有 `AddChannel()`、`Push()`、`SnapshotFeed()` 和
`SnapshotStart()`。设置 `MWAVEFORM_SNAPSHOT_ENABLE=0`。

Shell 状态查询通过一个简短临界区复制 `foc_status_t`，不得在 Shell 中逐字段
直读正在更新的运行上下文：

```c
typedef struct {
    foc_run_state_e eState;
    uint32_t        wFaults;
    foc_angle_t     tElectricalAngle;
    foc_scalar_t    qElectricalSpeed;
    foc_dq_t        tCurrent;
    foc_dq_t        tVoltage;
    foc_duty_abc_t  tDuty;
    foc_adc_calib_t tCalibration;
    bool            bPwmEnabled;
} foc_status_t;
```

这不是旧 motor snapshot 框架：它没有环形缓存、事件、序号或第二份常驻状态，
只是在调用栈上一次性复制当前值。

---

## 7. 安全不变量

以下条件必须在代码和测试中体现：

1. IDLE、CALIBRATING、FAULT 下 PWM 输出均关闭。
2. CALIBRATING 只允许读取 ADC 和累加零偏，不运行 FOC、不提交工作 duty。
3. RUNNING 前必须完成本次启动的零偏校准。
4. 进入 FAULT 的第一动作是 `foc_port_EmergencyStop()`。
5. 角度无效、ADC 失败、数学失败和 PWM 提交失败都进入 FAULT。
6. STOP 无论软件状态如何都先关闭 PWM。
7. PI 在每次进入 RUNNING 前复位，防止沿用上次积分量。
8. 中性 duty 必须先写入预装载寄存器，再使能 PWM 主输出。
9. 高频 ISR 中禁止日志、阻塞 I/O、动态内存和 I2C 访问。
10. AS5600 I2C 只在 1 kHz Clock 更新，ISR 只读一致性缓存。

---

## 8. 文件改动清单

### 新建

- `foc/foc_types.h`：公共组合、校准和 float/fixed 核心传输类型，解除 core/modulation
  头文件互相依赖。
- `foc/hal/foc_port.h`：唯一编译期硬件端口接口。
- `peripheral/stm32g431/foc_port.c`：STM32G431 ADC/PWM/校准实现。
- `tests/foc/test_foc_minimal_core.c`：纯核心测试。
- `tests/foc/test_foc_minimal_encoder.c`：编码器更新和外推测试。
- `tests/foc/test_foc_minimal_lifecycle.c`：使用 fake port 的四态生命周期测试。

### 重写或修改

- `foc/middleware/foc_core.h`：增加核心 input/command/state/API。
- `foc/middleware/foc_core.c`：实现纯数学闭环。
- `foc/control/foc_pid.h/.c`：继续使用具体 PI；移除本计划不需要的 Track 文档和依赖。
- `foc/modulation/foc_modulation.h/.c`：只保留 SVPWM 导出和构建。
- `foc/observer/foc_encoder.h/.c`：删除位置源适配，改成直接样本 API。
- `foc/app/foc_app.h/.c`：重写为唯一 runtime、四态生命周期和调度入口；
  float 版注册9路波形，fixed 版关闭波形。
- `foc/foc.h`：只导出极简公共类型和 API。
- `foc/foc_config.h`：选择 float/fixed、SVPWM、单电机，关闭旧功能宏。
- `foc/foc.mk`：只构建极简源文件。
- `src/userconfig.h`：关闭 waveform snapshot，保持 9 通道。
- `target/stm32g431/target.mk`：加入 `peripheral/stm32g431/foc_port.c`。
- `tests/foc/Makefile`：只构建极简核心、编码器和生命周期测试。
- `foc/README.md`、`foc/doc/foc-architecture.md`：更新为极简架构和限制。

### 核心切换并通过真机验证后删除

- `foc/motor/` 全目录。
- `foc/hal/foc_hal.c`、`foc/hal/foc_hal.h`、`foc/hal/foc_hal_adc.h`、
  `foc/hal/foc_hal_pwm.h`、`foc/hal/foc_hf_io.h`。
- `peripheral/stm32g431/foc_hal_mdi_adapter.c/.h`。
- `foc/app/phase_test.c`。
- 旧 motor、position、transition、observation、encapsulation 测试。

未参与极简固件构建的高级算法源码先保留在仓库作为参考，不进入
`FOC_SOURCES`；确认不再需要后另行删除，避免本计划同时做无关的大规模清库。

---

## 9. 实施任务

### Task 0：记录可比较基线

**Files:**
- Inspect: `build/template.map`
- Record: 本文档“实施记录”章节

- [x] **Step 1：构建当前 STM32G431 固件**

Run:

```powershell
.\make.bat clean
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel size
```

Expected：构建成功并生成 ELF、BIN、HEX 和 map。

- [x] **Step 2：记录基线数据**

记录 text/data/bss、`s_tMotor` 大小、`motor_HighFrequencyStep` 符号大小、
ISR DWT cycles 和当前最大栈占用。没有实测值时不得在方案中写预计节省值。

- [x] **Step 3：分别建立两个数值后端基线**

```powershell
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=float
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
```

Expected：两个后端都能完成编译；fixed 版不依赖硬件 FPU。

### Task 1：先建立纯数学极简核心

**Files:**
- Modify: `foc/middleware/foc_core.h`
- Modify: `foc/middleware/foc_core.c`
- Create: `tests/foc/test_foc_minimal_core.c`
- Modify: `tests/foc/Makefile`

- [x] **Step 1：添加测试**

覆盖：NULL、无效角度、电压模式、电流 PI、角度 wrap、SVPWM duty 范围、
Reset 后两个 PI 清零。测试只链接 math、PID、SVPWM 和 `foc_core.c`。

- [ ] **Step 2：确认测试先失败**

Run:

```powershell
mingw32-make -C tests/foc minimal-core
```

Expected：因 `foc_core_step` 和新类型尚不存在而编译失败。

- [x] **Step 3：实现最小核心**

严格按 §2 API 实现，不访问硬件、app 全局变量和位置源接口。

- [x] **Step 4：确认核心测试通过**

Run 同 Step 2。

Expected：float 和 fixed 两个后端均为 `PASS: minimal core (0 failures)`。

### Task 2：建立 STM32G431 编译期端口

**Files:**
- Create: `foc/hal/foc_port.h`
- Create: `peripheral/stm32g431/foc_port.c`
- Modify: `target/stm32g431/target.mk`

- [x] **Step 1：实现校准和采样端口**

从现有适配器迁移已验证的 ADC 通道、1390 counts/pu 和三相拓扑逻辑；删除
上下文、函数表、ABI 和“越界后钳到32768仍成功”的行为。

- [x] **Step 2：实现 PWM 端口**

保留 `port_mdi_MotorPwmSetDuty3()` 和 MDI Enable 路径，确保中性 duty 先提交、
MOE 后使能、急停可在任意状态调用。

- [x] **Step 3：只做编译验证**

Run:

```powershell
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel
```

Expected：端口自身无 warning/error；旧 FOC 仍可暂时保持构建。

### Task 3：重写编码器直接数据路径

**Files:**
- Modify: `foc/observer/foc_encoder.h`
- Modify: `foc/observer/foc_encoder.c`
- Create: `tests/foc/test_foc_minimal_encoder.c`

- [x] **Step 1：添加编码器失败测试**

覆盖首样本、正反转、4095→0 wrap、丢样、磁铁异常、低速禁止外推、50
electrical turn/s 外推，以及输出单位为 mechanical turn/s。

- [x] **Step 2：确认测试先失败**

Run:

```powershell
mingw32-make -C tests/foc minimal-encoder
```

Expected：旧 API 与 §5.2 新 API 不匹配，编译失败。

- [x] **Step 3：实现并通过两个后端测试**

删除 adapter 和 position source 依赖，确保 sequence 不变时不会重复计算速度，
只执行基于最后滤波速度的外推。

分别以 `FOC_NUMERIC_FLOAT` 和 `FOC_NUMERIC_FIXED` 编译；
Expected：两个后端均为 `PASS: minimal encoder (0 failures)`。

实施说明（相对计划 §5.2 API 的调整）：
- `foc_encoder_Step()` 按 L1.1 参数约束（≤4）重构为
  `foc_encoder_Step(ptEncoder, ptSample, ptOutput)` 三参数：
  极对数与高频周期下沉到 `foc_encoder_params_t`（Init 时固定），
  样本输入打包为 `foc_encoder_sample_t`，输出打包为
  `foc_encoder_output_t`。
- 磁铁失效从"连续 4 拍去抖"改为立即 `bValid=false`（计划 §5.2
  "磁铁失效或样本超时立即 bValid=false"）。
- 新增样本超时：无新样本超过 `hwInvalidTimeout`（默认 100 tick =
  5 ms @20 kHz）后置 `bValid=false`。
- `foc_encoder.h` 不再包含 `motor_position.h`；adapter、回调类型和
  位置源接口函数全部删除。
- 旧 `foc_app.c` 仍依赖位置源接口消费编码器，在 app 本地保留
  过渡胶水（`foc_app_EncoderStep/Reset`，标注 Task 4 删除），
  foc_encoder 模块保持纯观测器。旧 motor 回归测试不受影响。

### Task 4：建立四态生命周期并切换 ISR

**Files:**
- Rewrite: `foc/app/foc_app.h`
- Rewrite: `foc/app/foc_app.c`
- Create: `tests/foc/test_foc_minimal_lifecycle.c`

- [x] **Step 1：用 fake port 写生命周期失败测试**

覆盖 §7 十条安全不变量，特别验证：未校准 START 可以进入 CALIBRATING、
第512拍前不使能 PWM、第512拍后先提交中性 duty 再使能、校准失败和运行错误
均先急停再进入 FAULT。

- [x] **Step 2：确认测试先失败**

Run:

```powershell
mingw32-make -C tests/foc minimal-lifecycle
```

Expected：新 app 生命周期 API 尚不存在，编译失败。

- [x] **Step 3：实现最小 runtime 和命令处理**

实现 §2.3、§4 和 §7，不调用旧 motor API。保留 MODUS 对象注册、Shell 命令
和1 kHz Clock入口，但其内部只操作 `s_tFoc`。

- [x] **Step 4：切换高频 ISR**

按 §1.3 顺序接入 port、encoder、core 和 emergency stop。所有错误返回值都
必须检查。

- [x] **Step 5：运行全部极简 host 测试**

Run:

```powershell
mingw32-make -C tests/foc minimal
```

Expected：core、encoder、lifecycle 全部 `0 failures`。

**实施记录（2026-08-11，提交 f37f98a）：**
- `foc_app.h` 重写为最小 API（Start/Stop/ClearFault/SetVoltageReference/
  SetCurrentReference/SetSpeedReference/GetStatus/HighFrequencyISR），
  `foc_status_t` 仅依赖 `foc_types.h`，不再 include modus.h。
- `foc_app.c` 重写：唯一 `static foc_runtime_t s_tFoc`；四态
  IDLE/CALIBRATING/RUNNING/FAULT；命令邮箱 + perfc 中断守卫；20 kHz 链路
  port→encoder→core→port；1 kHz 速度 PI（SPEED 模式）；校准 512 拍 /
  100 ms 超时 / 偏移非法进 FAULT；所有故障先 EmergencyStop；校准完成先
  提交中性 duty 再使能 PWM；MODUS 注册 + 最小 `motor` Shell 命令。
- 新增 `tests/foc/test_foc_minimal_lifecycle.c`（fake foc_port + as5600
  stub + perfc_port_stub.h），覆盖 §7 可测不变量。
- 实测：minimal core/encoder/lifecycle × float/fixed 全部 PASS；
  STM32G431 float 固件 text=54124 data=516 bss=19208；
  fixed 固件 text=53980 bss=19200；fixed+soft-float text=55144。
  旧框架符号（motor_/foc_hal_/foc_controller_/foc_position_）已从链接
  产物剔除（gc-sections）。

### Task 5：波形和Shell状态收敛

**Files:**
- Modify: `src/userconfig.h`
- Modify: `foc/app/foc_app.c`

- [x] **Step 1：注册固定9路变量**

严格使用 §6 表格；不得注册第10路。检查每个 AddVariable 返回值。

- [x] **Step 2：删除Push和快照路径**

删除 `AddChannel`、`Push`、`SnapshotFeed`、`SnapshotStart`，仅在 float
版执行变量注册；fixed 版关闭整个波形编译路径。float 版设置：

```c
#define MWAVEFORM_MAX_CHANNELS      9
#define MWAVEFORM_SNAPSHOT_ENABLE   0
```

- [x] **Step 3：实现栈上状态复制API**

Shell `motor status` 改用 `foc_app_GetStatus()`，一次临界区复制 §6 的
`foc_status_t`。

**实施记录（2026-08-11）：**
- `src/userconfig.h`：`MWAVEFORM_SNAPSHOT_ENABLE 1→0`（`MAX_CHANNELS` 保持 9）。
- `foc_app.c`：新增 `foc_app_WaveformInit()`（仅 `FOC_NUMERIC_FLOAT &&
  MWAVEFORM_ENABLE` 编译）：`mwaveform.Init` → 9 路 `AddVariable`
  （Iu/Iv/Iw/Id/Iq/Angle/Speed/Vd/Vq，FLOAT ×1000，Angle 绑定
  `fElectricalAngleTurns`）→ 逐路检查返回值（`0xFF` 即停）→
  `SetStreamRate(50000,10000)` → `Start`；ISR 末尾每拍
  `mwaveform.Step()`；Shell `motor status` 全字段输出
  （state 名/faults/pwm/angle/speed/Id/Iq/Vd/Vq/duty/calib），新增
  `foc_app_StateName()`。
- **modus 子模块 bug 修复**：`mwaveform.c` 的 `cmd_snapshot` status 分支
  在 `MWAVEFORM_SNAPSHOT_ENABLE=0` 时引用被守卫成员
  `hwSnapshotValidCount` 导致编译失败；加 `#if MWAVEFORM_SNAPSHOT_ENABLE`
  守卫（子模块工作树因此变为 dirty，需用户确认是否随提交带入）。
- 实测（clean 后全量重编）：float text=54264 bss=18208；
  fixed text=53724 bss=18200；bss 较基线（19208/19200）各降 ~1 KB
  （快照缓冲移除）；minimal host 测试 6 PASS 不变。

### Task 6：停止构建并删除旧框架

**Files:**
- Modify: `foc/foc.mk`
- Modify: `foc/foc.h`
- Delete: §8 列出的旧 motor/HAL/adapter 文件
- Delete/Rewrite: 旧 motor 相关测试

- [x] **Step 1：先收敛 FOC_SOURCES**

只构建 math、`foc_core.c`、`foc_pid.c`、SVPWM、encoder、app 和
STM32G431 port。

- [x] **Step 2：构建并运行极简测试**

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc minimal
.\make.bat clean
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
```

Expected：float/fixed 两个后端测试和固件构建均通过，链接结果不存在 `motor_`、`foc_hal_`、
`foc_controller_`、`foc_position_` 符号。

- [x] **Step 3：删除不再被引用的旧文件**

用 `rg` 确认没有引用后，再删除 §8 指定文件。不得删除与极简核心仍共享的
math、PID、SVPWM、AS5600驱动和MODUS代码。

- [x] **Step 4：检查目标和主机产物**

```powershell
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel size
rg -n "motor_|foc_hal_|foc_controller_|foc_position_" build/template.map
```

Expected：第一条给出新尺寸；第二条无旧框架链接符号。

**实施记录（2026-08-11）：**
- `foc/foc.mk`：FOC_SOURCES 收敛为 9 个文件（foc_numeric/foc_angle/
  foc_trig_lut/foc_math/foc_core/foc_pid/foc_modulation/foc_encoder/
  foc_app），wildcard 删除。
- `foc/foc.h`：重写为极简伞头（math_types/core/pid/modulation/encoder/
  app），不再包含 motor/hal/observer/optimization/experimental 旧头。
- 删除：`foc/motor/` 全目录、`foc/hal/{foc_hal.c,foc_hal.h,
  foc_hal_adc.h,foc_hal_pwm.h,foc_hf_io.h}`、
  `peripheral/stm32g431/foc_hal_mdi_adapter.c/.h`、`foc/app/phase_test.c`、
  旧测试（test_foc/test_motor*/test_observer/encapsulation probes）。
  `foc/hal/foc_hal_types.h` 按 §8 清单保留。
- `tests/foc/Makefile`：只保留 minimal-core/encoder/lifecycle 三套 target
  （float+fixed），删除旧 float/fixed/encapsulation 目标与 TEST_SRCS/
  FOC_SRCS 旧列表。
- 实测：host minimal 6 PASS；float 固件 text=54264 bss=18208；
  fixed 固件 text=53724 bss=18200；map 中无任何旧框架 `.text.*` 段
  （foc_hal_mdi_adapter.o 已随文件删除不再编译）。

### Task 7：真机安全验证

**Files:**
- Verify: STM32G431 target
- Record: 本文档“实施记录”章节

- [ ] **Step 1：烧录并只验证校准**

```powershell
.\make.bat auto TARGET_CHIP=stm32g431 BUILD=debug-rel
```

执行 START 后确认：PWM输出保持关闭约25.6ms、512次校准成功、偏移在合法区间、
随后才进入 RUNNING。

- [ ] **Step 2：验证急停和故障**

依次验证 STOP、编码器磁铁异常、采样失败模拟和校准偏移非法；每种情况都必须
先关闭 PWM 再改变软件状态。

- [ ] **Step 3：低电流编码器闭环**

从 `Iq <= 0.015 pu`、`speed <= 2 electrical turn/s` 开始，确认电流方向、
Angle连续、三相和接近0、母线电流和温升在现有安全限制内。

- [ ] **Step 4：验证20kHz周期预算**

用 DWT 分别记录空闲、校准、电压模式和电流模式的最大 cycles。所有 RUNNING
路径必须小于8500 cycles的70%，即 `<5950 cycles`，并且无偶发越界。

- [ ] **Step 5：记录最终收益**

记录 text/data/bss、唯一 `s_tFoc` 大小、ISR最大cycles、最大栈占用，与Task 0
基线逐项比较。只报告实际测量值。

### Task 8：更新文档

**Files:**
- Modify: `foc/README.md`
- Modify: `foc/doc/foc-architecture.md`
- Modify: 本计划

- [x] **Step 1：删除旧能力声明**

文档不得继续宣称多实例、位置闭环、运行时控制器、位置源融合、无感切换、
事件环或fixed后端属于当前固件能力。

- [x] **Step 2：记录接口和限制**

写明STM32G431单电机、float、编码器、电压/电流/速度、SVPWM、四态生命周期、
9路波形和硬件安全限制。

- [x] **Step 3：完成最终一致性检查**

```powershell
rg -n "motor_handle_t|motor_impl_t|motor_hf_plan_t|MOTOR_CONTROL_POSITION" `
  foc docs/superpowers/plans/2026-08-11-foc-core-minimal-refactor.md
```

Expected：源码和当前架构文档中没有旧框架声明；参考历史文档不计入验收。

**实施记录（2026-08-11）：**
- `foc/README.md` 重写：极简单电机架构、三层数据流、四态生命周期、
  安全不变量、编码器外推、9 路波形、构建/测试/Shell、参数整定顺序；
  删除全部旧多实例/位置源/过渡/事件环声明。
- `foc/doc/foc-architecture.md` 重写：分层图、数值后端/BAM32/三角后端
  （保留仍有效章节）、四态生命周期、20 kHz 数据流、外推设计、安全
  不变量、波形、设计决策记录；删除 plan resolver/不透明句柄/事件环/
  位置源接口章节。
- 一致性检查：`foc/` 源码与两份文档已无 `motor_handle_t` /
  `motor_impl_t` / `motor_hf_plan_t` / `MOTOR_CONTROL_POSITION`（参考目录
  `foc/experimental/foc_verify.*` 与 `foc/template/` 保留旧声明，不参与
  构建，不计入验收）。

---

## 10. 验收标准

以下条件必须全部满足，才能判定重构完成：

- STM32G431 debug-rel 的 float/fixed 两个后端均构建成功。
- minimal core、encoder、lifecycle host 测试在 float/fixed 两个后端全部通过。
- 链接产物不存在旧 motor/HAL/position/controller接口符号。
- 只有一个 `foc_runtime_t` 实例，没有896字节opaque motor handle。
- 高频运行路径不存在运行时硬件、角度源、控制器或调制函数指针。
- CALIBRATING期间PWM关闭，校准完成后才使能。
- 所有运行错误先急停再进入FAULT。
- 编码器角度在20kHz下连续外推，低速区不放大量化噪声。
- float版波形恰好9路，AddVariable全部成功，无Push和SnapshotFeed；fixed版不引入波形
  或软浮点依赖。
- 电流模式最大ISR占用 `<5950 cycles @170 MHz`。
- 最终RAM、Flash、栈和cycles均有实测基线对比。
- README和架构文档与实际能力一致。

---

## 11. 实施记录

本节只填写命令实测结果，不填写预测数据。

当前完成 Task 1、Task 2 和 Task 3；旧 motor/app 路径仍在运行，
Task 4 之后才切换生命周期和高频 ISR。`SW_ROOT` 默认值未修改，
以下构建均通过命令行临时覆盖到 `D:/0_software`。

| 指标 | 重构前 | 重构后 | 差值 |
|---|---:|---:|---:|
| text | 75764 (float基线) | 75788 (float) / 75008 (fixed) | float +24 / fixed 无基线 |
| data | 964 | 964 | 0 |
| bss | 20104 | 20096 | -8 |
| FOC运行上下文 | `s_tMotor=960 B` | 旧路径仍为 `s_tMotor=960 B` | 0 |
| 电流模式最大cycles | 未测 | 未测 | 未测 |
| 高频路径最大栈 | 未测 | 未测 | 未测 |

Task 3 完成后实测：
- minimal core、minimal encoder、旧 FOC 回归测试的 float/fixed 均通过
  （core 2×PASS、encoder 2×PASS、旧回归 2×PASS）。
- STM32G431 debug-rel 的 float/fixed 目标构建均通过
  （float text=75788 / fixed text=75008）。
- fixed 另以 `-mfloat-abi=soft` 完成目标构建：text=76184、data=964、
  bss=20104；foc_encoder.o / foc_core.o / foc_port.o 均无浮点辅助符号
  （`__aeabi_ldivmod` 为 64 位整数除法辅助，非浮点，符合 §0.3）。
- 链接产物不再存在 `foc_encoder_source_Init`、
  `foc_encoder_PositionSourceInterface` 符号。
- 新增/修改文件行宽 ≤78 字符（foc_app.c 既有超长行属旧代码，
  Task 4 重写时一并处理）。

文档状态：**Task 1/Task 2/Task 3 已实施，Task 4 及后续待实施。**
