# FOC 高频控制环路 V4 Fast Path 设计

## 目标

将 `motor_HighFrequencyStep()` 从通用对象服务入口重构为固定、短且可测的 20 kHz 控制路径。在保留多芯片、可替换位置源、可替换控制器以及现有公开 `motor_*` API 的前提下，消除高频路径中的重复校验、包装转发、临界区和非必要状态搬运。

本设计适用于现有 STM32G431 和 AT32F413，并规定后续 target 的统一移植契约。`foc/` 仍不得包含厂商头文件或直接访问厂商寄存器。

## 已确认的问题

V3 已经完成成对 sin/cos、G431 三相 PWM 批量提交、采样小函数内联和入口字段瘦身。当前高频路径仍有以下结构问题：

- `motor_private_SampleCurrent()` 经过 private helper、HAL wrapper、ADC adapter和 raw-read helper；其中接口校验只在绑定期有价值。
- `motor_HighFrequencyStep()` 同时处理算法、运行态同步、事件入队、Profile发布和完整诊断状态回写，导致局部变量与状态字段在 ISR 中反复搬运。
- 位置源和控制器使用通用 helper 再转发函数指针；多态需要保留，但 helper的输入校验不应按 20 kHz 重复执行。
- PWM 提交虽已在 G431 合并为三相直写，提交前后的状态检查、临界区和状态发布仍与寄存器提交混在同一函数中。

## 架构

### 快慢路径分离

初始化、启动、停止、校准、诊断、事件读取和快照读取属于慢路径。它们负责配置校验、资源绑定、同步和面向应用的错误处理。高频路径只执行固定顺序的
采样、坐标变换、位置源、控制器、调制、PWM 提交和最小状态发布。

`motor_Init()` 与 `motor_Start()` 将已验证的接口解析为每电机一个`motor_hf_plan_t`。该执行计划在高频期间只读，保存下列内容：

- 高频 I/O 回调、上下文和紧急停机回调；
- 活动位置源的 `pContext + fnStep`；
- Id/Iq 控制器的 `pContext + fnStep`；
- 已解析的调制函数、采样周期和位置配置；
- 启动/切换需要的只读阈值与样本数。

高频过程读写与应用命令分为下列私有区域：

| 区域 | 写入者 | 高频访问 |
| --- | --- | --- |
| `motor_hf_plan_t` | 初始化、启动、受控配置切换 | 只读 |
| `motor_hf_command_t` | 公共 setter、低频控制 | 一次读取 |
| `motor_hf_state_t` | 高频 ISR | 读写 |
| 事件、Profile、诊断数据 | 慢路径或调试构建 | 仅最小通知/发布 |

公共 setter 继续通过 `motor_sync_if_t` 更新命令邮箱；该同步实现必须屏蔽对应的高频 IRQ。高频 ISR 因此不进入临界区即可读取一致命令。快照读者使用
同一同步机制读取 `motor_hf_state_t`，公开语义保持不变。

发布构建以“一个电机只能由一个已配置 PWM/ADC ISR 调用”为调度契约，移除`bHighFrequencyStepInProgress` 的临界区往返。调试构建保留轻量重入断言。

### 统一 target 移植契约

在 `foc/hal/foc_hf_io.h` 定义内部可移植协议：

```c
#define FOC_HF_IO_ABI_VERSION 1U

typedef struct {
    uint16_t wAbiVersion;
    void *pContext;
    foc_result_t (*fnSampleCurrent)(
        void *pContext, phase_current_handle_t *ptCurrent);
    foc_result_t (*fnCommitDuty)(
        void *pContext, const foc_duty_abc_t *ptDuty);
    void (*fnEmergencyStop)(void *pContext);
} foc_hf_io_if_t;
```

`foc_hal_t` 增加 `tHfIo`，但原有 `tPwm`、`tAdc` 和所有公开 HAL helper 保留。`foc_hal_ValidateHighFrequency()` 只在初始化和启动时检查 ABI 版本、上下文和三个回调。未实现 Fast Path 的 target 必须显式失败，不允许在 ISR 静默回退至旧的多层接口链。

回调约束如下：

- `fnSampleCurrent` 读取本周期已就绪的 ADC 结果，并完整更新`phase_current_handle_t` 的 pu 电流、拓扑和校准相关状态；失败时输出不被下游使用。
- `fnCommitDuty` 一次提交 U/V/W 三相预装载值，不改变 preload、break、急停或ADC 触发语义。调制器已经保证 duty 合法，发布回调不得重复做通用范围校验。
- `fnEmergencyStop` 必须可在任意采样或提交失败后调用，独立于常规 PWM 提交，且不得调用 `motor_*` API。
- 三个回调均为常数时间、不可阻塞、不可分配内存、不可日志输出。

target 适配器可调用本 target 的 MDI/port 批量端点；只有适配器和
`peripheral/<chip>/` 可以接触厂商寄存器或厂商头文件。

### 高频执行顺序

`motor_HighFrequencyStep()` 使用局部 `motor_hf_frame_t` 传递单周期临时量，不复制 `motor_control_t` 或接口结构。固定执行顺序为：

```text
读取命令邮箱
-> io.fnSampleCurrent
-> Clarke
-> source.fnStep
-> sincos + Park
-> Id/Iq fnStep
-> IPark
-> 已解析调制函数
-> io.fnCommitDuty
-> 最小状态发布
```

位置源、控制器和调制器仍保持可替换。执行计划只缓存已验证的直接回调，故ISR 不再通过 `foc_position_source_Step()`、`foc_controller_Step()`、
`foc_hal_CurrentReconstruct()` 或 `foc_hal_SetDuty()` 再做一次服务层转发。

任何失败都通过唯一故障出口处理：更新高频故障状态，调用一次`plan.io.fnEmergencyStop()`，再发布结果。采样 helper 内部不再重入
`motor_EmergencyStop()`。

源有效性和切换事件由 ISR 仅投递类型/载荷，`motor_LowFrequencyStep()` 统一入事件环。由此造成的可见延迟最多为一个低频周期；故障和 PWM 停止不延迟。

## 可裁剪性与资源分层

现有 FOC 体量大、状态变量多，而各 target 资源差异显著（如 G431 与
AT32F421/CH592）。Fast Path 落地时必须同步建立编译期功能分层，避免
"一个全量实现跑所有芯片"：

- 核心层（必选）：采样、Clarke/Park、Id/Iq 控制器、IPark、调制、PWM 提交、
  最小状态发布。所有 target 必须通过该层验收。
- 扩展层（可选）：观测器（SMO/NLFO）、DOB、cogging 补偿、前馈等算法扩展。
  资源充足的 target 启用并经 plan 回调解析接入；资源受限的 target 整体裁剪，
  不占用 RAM/Flash。
- 诊断调试层（可选）：Profile、事件环、完整诊断状态回写。仅在调试构建或
  高端 target 启用。

裁剪约束：

- 以编译期配置宏（如 `FOC_CONFIG_ENABLE_*`）裁剪；`motor_hf_plan_t` 和
  `motor_hf_state_t` 的布局随配置同步收缩。ISR 内不得出现针对可选功能的
  运行期分支判断，特性有无只允许在绑定期解析。
- 被裁剪或未绑定的可选功能按现有契约在绑定/启动期显式失败，不允许运行期
  静默跳过；这与"未实现 Fast Path 必须显式失败"的协议要求一致——三个 I/O
  回调本身开销极小，属于核心层，不因裁剪免除。
- 公开 `motor_*` API 语义不因裁剪改变；未启用功能对应的 API 返回统一的
  "不支持"结果，而非让应用层编译失败。
- 验收标准按 target 启用的功能集分层执行：核心层 cycle 门槛适用于所有
  target，扩展层只验收启用了对应功能的构建。

## 目标适配

- STM32G431：复用 `port_mdi_MotorPwmSetDuty3()`；将 ADC 寄存器读取和电流重建并入 Fast Path 采样回调，不再通过 `mdi_adc_get_raw()`。
- AT32F413：增加 target-local 三相批量 MDI/port 提交端点，替代三次`MDI_Write()`；ADC 读取和电流重建合并为单个采样回调。
- 后续 target：在 `foc_hal_mdi_Bind()` 或等价 binder 中填充 `tHfIo` 并通过初始化校验，无需复制 `motor_HighFrequencyStep()`。

## 文件范围

| 文件 | 变更 |
| --- | --- |
| `foc/hal/foc_hf_io.h`（新建） | Fast Path ABI、契约与校验声明 |
| `foc/hal/foc_hal.h`、`foc/hal/foc_hal.c` | 保存并验证 `tHfIo`，保留慢路径 HAL |
| `foc/motor/motor_hf_private.h`（新建） | plan、command、state、frame 私有类型 |
| `foc/motor/motor_hf.c`（新建） | 高频内核和唯一故障出口 |
| `foc/motor/motor_private.h` | 重新组织内部状态布局 |
| `foc/motor/motor.c` | 初始化绑定、命令邮箱、快照和校准衔接 |
| `foc/motor/motor_fsm.c` | 启动期解析/刷新执行计划 |
| `foc/motor/motor_control.c` | 保留低频控制与 setter，移出高频实现 |
| `peripheral/stm32g431/foc_hal_mdi_adapter.*` | G431 Fast Path 回调 |
| `peripheral/at32f413/foc_hal_mdi_adapter.*` | AT32 Fast Path 回调和三相批量提交 |
| `tests/foc/Makefile`、`tests/foc/test_motor*.c` | Fast Path host fake、故障与并发语义覆盖 |
| `foc/README.md`、`foc/doc/foc-architecture.md` | 移植规则和快慢路径说明 |

## 实施顺序

1. 新建协议和 host fake，先验证版本/缺失回调会被拒绝。
2. 两个现有 target 同步实现并绑定 `tHfIo`，确认校准、raw 读取和 PWM 使能仍走
   原慢路径。
3. 引入执行计划并在 `motor_Init()`、`motor_Start()` 绑定控制器、位置源和调制。
4. 引入高频 state/frame，迁移高频内核并删除 nested sample/commit helper。
5. 将事件入队和完整诊断发布迁出 ISR，保留故障即时处理。
6. 运行 host 回归、目标构建、硬件 DWT Profile，更新性能记录。

## 验收标准

- float、fixed、strict-aliasing、封装测试全部通过。
- 覆盖 Fast Path ABI 不匹配、缺回调、采样失败、PWM 提交失败、紧急停机仅一次、命令/快照一致性、开环、电流环、外环、源切换和位置源失效。
- G431 在 170 MHz / 20 kHz / 电流闭环 / `FOC_HF_PROFILE_LEVEL=2` 下，以 V3约 3590 有效 cycles 为基线，目标 mean <= 3000、P99 <= 4250。
- SMO/NLFO 接入后，完整 high-frequency step 目标 mean <= 5500、P99 <= 6800，连续 100000 周期无 overrun。
- AT32 先建立同场景分段基线，验收 Fast Path 前后下降幅度和所有功能无回归，不套用 G431 的绝对 cycle 门槛。

## 非目标

- 不替换公开 `motor_*` API、位置源接口或控制器接口。
- 不在 FOC 内核加入 target 条件编译、厂商头文件或寄存器访问。
- 不改变 PWM preload、break/硬件过流、ADC 触发和校准时序。
- 不以 LTO、全局强制内联或 `-ffast-math` 代替架构优化；这些可作为后续独立
  构建优化评估。
