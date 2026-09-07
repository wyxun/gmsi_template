# FOC App 对象化重写设计

日期：2026-08-31

状态：架构边界已确认，进入实现前文档复核

## 目标

基于当前极简 FOC 实现，将 `foc_app` 重写为符合 MODUS Class 规范的
单电机应用对象。`foc_app_t` 成为全部应用运行状态的唯一拥有者，
`foc_app_cfg_t` 成为初始化参数和外部依赖的唯一入口，同时保留已经
验证过的 FOC 电流环调用链和现有 ADC、PWM、TIM1、CH4 外设初始化。

## 范围

本次重写包括：

- 在 `foc/app/foc_app.h` 完整声明 `foc_app_cfg_t` 和 `foc_app_t`；
- 将当前 `foc_app.c` 中的可变文件静态状态迁移到对象或对象成员；
- 按 `class/template_class.c` 的初始化顺序完成对象、配置、依赖和
  子结构初始化；
- 保持 FOC 数学核心、电流 PI、编码器观测器和目标硬件适配的目录边界；
- 保持单电机 20 kHz 高频控制和 1 kHz 前台服务的实时职责划分；
- 为对象初始化、状态流程、配置校验和多实例状态隔离补充测试。

本次不包括：

- 不把通用 FOC 算法迁移到 `Mdriver`；
- 不新建只为拆文件而存在的 `Mdriver/foc_app/*`；
- 不恢复旧多实例 `motor` 框架；
- 不修改 ADC、PWM、TIM1、CH4 的初始化和采样映射；
- 不改变 `727f8bb2fbba9ec76cace5512d28` 所验证的电流环数学顺序；
- 不实现多电机或多个物理 FOC port 的仲裁。

## 现状和设计依据

当前工作树基于极简 FOC 重构。`foc_app.c` 中的
`foc_runtime_t`、AS5600 实例、ISR 统计、Waveform 状态、轮询计时和
命令状态仍然通过文件静态变量保存，而 `foc_app_t` 和
`foc_app_cfg_t` 只是定义在 `.c` 中的空壳。这导致 MODUS 对象不是实际
运行状态的拥有者，也无法进行显式对象组合和独立实例测试。

`727f8bb2fbba9ec76cace5512d28` 是功能行为参考，尤其用于确认电流环、
ADC/PWM 端口调用顺序和相序相关行为；它不是新的 `foc_app` 结构模板。
新的 Class 结构以 `class/template_class.c` 和
`class/template_class.h` 的对象初始化契约为准。

## 模块归属

| 功能 | 代码位置 | 对象关系 | Mdriver 决策 |
| --- | --- | --- | --- |
| 数值、角度、Clarke、Park、IPark | `foc/math`、`foc/middleware` | `foc_core_state_t` 作为 App 成员 | 保留在 `foc` |
| 电流 PI 和 PID 运算 | `foc/control/foc_pid.*` | 电流 PI 由 Core 持有，速度 PI 由 App 持有 | 保留在 `foc` |
| SVPWM | `foc/modulation` | Core 更新 duty，App 不复制算法状态 | 保留在 `foc` |
| 编码器机械角度/速度观测 | `foc/observer/foc_encoder.*` | `foc_encoder_t` 作为 App 成员 | 保留在 `foc` |
| AS5600 芯片驱动 | `peripheral/driver/as5600.*` | 单电机时由 App 组合其实例；I2C 是外部依赖 | 保留在 `peripheral/driver` |
| ADC、PWM、TIM1 适配 | `foc/hal`、`peripheral/<chip>` | App 只调用 port 接口 | 保留在硬件层 |
| 生命周期、命令和故障 | `foc/app` | 直接属于 `foc_app_t` | 不抽取 |
| 电气零位和角度来源 | `foc/app` | 作为 App 的编码器/启动状态成员 | 不抽取 |
| 编码器校准和启动对齐 | `foc/app` | 由 App PT 和状态成员管理 | 当前不抽取 |
| ISR 统计和 Waveform 通道 | `foc/app` | 统计数据和通道 ID 属于 App | 当前不抽取 |
| Shell 参数解析 | `foc/app` | 只调用对象 API，不拥有运行状态 | 当前不抽取 |

### FOC 框架源码

以下模块与 `foc_app` 无关，可以由其他电机 Class 或 host 测试独立使用，
因此继续位于 `foc/`：

- `foc_types_t` 及 ABC、AlphaBeta、DQ、duty、输入和命令值类型；
- 数值后端、BAM32 角度、三角函数和 SVPWM；
- `foc_core_step()` 及其 Clarke/Park/IPark 链路；
- `foc_pid_t` 的初始化、复位、跟踪和一步运算；
- `foc_encoder_t` 的机械角度、机械速度、样本序号和外推逻辑。

`foc_core_state_t` 已经包含 `tIdPi` 和 `tIqPi`，所以重写 App 时不再
另建一份电流 PI 状态，也不在 App 中重复调用电流 PI。

### MDI 和硬件层

AS5600 的驱动代码仍放在 `peripheral/driver`，因为它是通过 `mdi_iic_t`
访问总线的芯片无关设备驱动，而不是 FOC 业务模块。单电机 App 可以
组合一个 `as5600_t` 实例，但不把 AS5600 的寄存器读写放进 `foc_app.c`。

ADC、PWM、TIM1、CH4 和三相电流采样映射继续由
`peripheral/stm32g431` 实现，`foc/app` 只能调用
`foc_port_*()` 接口。本轮不因 Class 重写修改任何外设初始化。

当前 port 是单电机编译期硬件适配，内部校准累计状态仍归硬件 port
实现；本次只消除 `foc_app` 的隐藏运行时。未来若需要多个物理电机，
应另行设计带 context 的 port 或显式资源 Manager，不通过隐式静态变量
扩展本次 Class。

## `foc_app_t` 组合设计

`foc_app_t` 和它的直接组合类型必须在 `foc_app.h` 中完整可见。直接
组合类型只是为了保持职责分组，不代表它们是独立 MODUS Class，也不
自动意味着应放入 Mdriver。

推荐的对象分组如下：

```text
foc_app_t
├── ptBase
├── tCore                  foc_core_state_t
├── tSpeedPid             foc_pid_t
├── tEncoder              foc_encoder_t
├── tAs5600                AS5600 驱动实例（目标支持时）
├── tCalibration           三相 ADC 校准结果和流程计数
├── tCommand               当前控制模式和参考值
├── tLifecycle             IDLE/CALIBRATING/RUNNING/FAULT、故障和邮箱
├── tStartup               对齐、开环角度和启动切换状态
├── tEncoderApplication    零位、方向和机械速度应用状态
├── tRunPt                 前台 PT 游标和轮询定时器
└── tDiagnostics           ISR 统计、显示值和 Waveform 通道 ID
```

实际结构分组应控制直接成员数量，不得为了追求层次而增加无意义的
指针、拷贝或函数跳转。简单状态可以直接放入 `foc_app_t`，只有共同
职责明显的字段才组成上述小结构。

以下状态不得出现在 `foc_app.c` 的文件静态变量中：

- 当前命令、Pending 命令、生命周期状态和故障位；
- ADC 校准计数、校准结果、启动对齐计数；
- 开环角度、开环速度、角度来源和电气零位；
- 编码器驱动状态、观测器状态和机械速度；
- 速度 PI 状态；
- ISR 的 last/max/total/sample 统计；
- 编码器 Waveform 展示值和通道 ID；
- I2C 轮询的上次时间和连续失败计数；
- 前台 PT 游标和软件定时器。

允许保留在 `.c` 的静态对象仅包括 MODUS 基础对象及其基础配置、只读
默认参数和只读辅助函数。`MODUS_DECLARE_OBJECT` 生成的
`tFocApp` 是唯一 App 实例，不再额外创建 `static foc_runtime_t`。

## `foc_app_cfg_t` 配置契约

配置结构体不再使用 `chReserved`。所有字段必须在 `foc_app_Init()` 中
被校验、复制或绑定。配置包含：

- MODUS RingBuffer 地址和容量；
- 电流 PI 参数；
- 速度 PI 参数；
- 编码器观测参数，包括极对数和高频周期；
- 初始电气零位和方向配置；
- AS5600 所需的 `mdi_iic_t` 依赖，或明确表示本目标不使用编码器。

电流、电压、速度参考属于运行时命令，不放入初始化配置。ADC 偏移
结果属于运行时状态，不由配置伪造。`foc_app_Init()` 必须拒绝空地址、
非法 PID 限幅、非法编码器参数、零极对数和未满足硬件依赖的配置。

## 初始化顺序

`foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)` 遵循
模板顺序：

1. 转换并检查对象和配置地址；
2. 校验配置内容，失败时保持对象处于安全的 PWM 关闭状态；
3. 设置对象的初始生命周期和故障状态；
4. 设置 `ptThis->ptBase`，并将基础配置的 `wParent` 设为对象地址；
5. 绑定 RingBuffer、I2C 和其他显式依赖；
6. 初始化 Core、电流 PI、速度 PI、编码器、AS5600、PT、定时器和诊断；
7. 调用 `foc_port_Init()` 并执行急停，检查可检查的返回值；
8. 初始化 Waveform 等非实时调试资源；
9. 最后调用并检查 `mbase_Init()`。

MODUS 有效路径不得用 `(void)wObjectAddr`、`(void)wObjectCfgAddr` 或
`(void)ptCfg` 跳过对象初始化。初始化失败时不能留下已使能的 PWM。

## 调度和实时边界

### 高频 ISR

保留现有无日志、无 I2C、无阻塞的固定时限链路：

```text
CurrentSample
→ Angle/Encoder observer
→ foc_core_step
→ DutyCommit
```

高频函数接收真实 `foc_app_t` 对象，或由现有无参数中断包装器转发到
MODUS 生成的唯一 `tFocApp`。包装器不是第二套运行时。所有 ISR 状态
仍通过对象成员访问。

### 前台 Run 和 Clock

`Clock()` 只做短小的周期标志或速度环触发，不执行阻塞 I2C。AS5600
更新放在主循环 `Run()` 驱动的对象 PT 中，轮询时间、重试次数和失败状态
全部属于对象成员。编码器读取失败必须通过状态机收敛，不得卡死 SysTick
或高频 ISR。

校准、编码器轮询、启动等待和其他多步骤流程使用对象成员保存的
perfc-PT 状态及 `msoft_timer_t`，不得使用函数内 `static` 变量。

## Mdriver 判定规则

本次不创建 Mdriver。后续只有同时满足以下条件才抽取：

1. 具有独立职责和稳定边界；
2. 需要多个操作 API，而不是一个被动数据结构；
3. 配置和依赖可以脱离 `foc_app_t` 单独描述；
4. 有第二个 Class 的真实复用需求或明确高概率复用场景；
5. 可以独立测试，并且不会破坏高频调用深度。

只服务 `foc_app` 的复杂模块放到
`Mdriver/foc_app/<submodule>/`；可以被多个 Class 独立实例化且不依赖
父 Class 的模块才放到 `Mdriver/<submodule>/`。子模块由父 App 拥有、
初始化、调度和处理错误，不单独调用 `MODUS_DECLARE_OBJECT`。

当前编码器校准、启动对齐、命令邮箱、Waveform 注册和 ISR 统计都不
满足抽取必要性，先作为 App 的直接组合状态。

## 测试边界

实现阶段至少覆盖：

- `foc_app_t` 和 `foc_app_cfg_t` 的完整对象初始化；
- 空对象、空配置和非法配置拒绝；
- 基础对象绑定和 `wParent` 设置；
- 当前 PI、速度 PI、编码器和 AS5600 依赖初始化；
- PT 流程推进、校准超时、I2C 失败退避和错误收敛；
- STOP/FAULT 下 PWM 关闭；
- 两个独立 App 对象的命令、PID、编码器和统计状态互不污染；
- 现有 current loop host 测试在 float/fixed 后端继续通过；
- STM32G431 debug 构建不修改外设初始化且高频 ISR 可正常链接。

真实母线测试仍按安全顺序执行：先 host 测试和无母线固件检查，再由
用户确认母线状态后做 PWM/电流环验证。没有硬件证据时不宣称真机闭环
行为已验证。

## 实施文件边界

预计实现阶段修改：

- `foc/app/foc_app.h`：公开完整配置、对象和直接组合类型；
- `foc/app/foc_app.c`：按对象成员重写初始化、Run、Clock、ISR 和命令；
- `tests/foc/test_foc_minimal_lifecycle.c`：改为显式对象/配置并增加
  多实例和初始化契约测试；
- `tests/foc/Makefile`：保持 float/fixed 两套 App 测试入口；
- 必要时更新 `foc/README.md` 和 `foc/doc/foc-architecture.md` 的对象
  所有权说明。

不预计新增 `Mdriver` 文件，也不预计修改 ADC、PWM、TIM1、CH4 的
初始化文件。
