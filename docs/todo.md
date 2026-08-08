# TODO

> 本文件用于记录待评估事项。事项会逐步增加，先统一评估，再决定是否优化或实施。

## 说明

- 每个事项使用一个编号小节。
- 每项先写清诉求、现状、期望、验收和注意事项。
- 本文件不是“已确认要优化”的清单。
- 统一评估后，在对应事项的“评估结论”中记录结论。

## 待评估事项

### 事项 1：统一 FOC 控制器接口并收敛控制命名

#### 诉求

当前控制器调用链存在三个高度重叠的接口类型：

- `foc_controller_if_t`
- `motor_step_controller_if_t`
- `motor_tracking_controller_if_t`

它们都包含 `fnStep`，只是分别带不带 `fnReset`、`fnTrack`。阅读代码时需要反复切换类型才能理解控制器如何绑定、如何被高频和低频路径调用，结构上比较绕。

希望统一为 `foc_controller_if_t` 作为唯一控制器接口，删除或别名化：

- `motor_step_controller_if_t`
- `motor_tracking_controller_if_t`

最终让初始化、运行时配置、高频执行计划和低频外环都操作同一个控制器接口，减少重复抽象。

同时收敛控制配置命名，避免 `control` 语义重复。例如：

- `tControlConfig` -> `control` / `tControl`
- `tIdController` -> `tId`
- `tIqController` -> `tIq`
- `tSpeedController` -> `tSpeed`
- `tPositionController` -> `tPosition`

最终希望：

```c
impl->control.tId.pController
impl->control.tIq.pController
impl->control.tSpeed.pController
impl->control.tPosition.pController
```

#### 现状

- `motor_control_config_t` 的四个控制器已经使用 `foc_controller_if_t`。
- `motor_control_runtime_config_t` 把 D/Q 电流环转成 `motor_step_controller_if_t`。
- 速度环、位置环转成 `motor_tracking_controller_if_t`。
- `motor_hf_plan_t` 又把 D/Q 电流环复制为 `motor_step_controller_if_t`。
- 高频路径通过 `plan->tId.fnStep()` / `plan->tIq.fnStep()` 调用。
- 低频路径直接使用 `tControlConfig.tSpeedController.fnStep()` / `tPositionController.fnStep()`。

#### 期望结果

- 三个控制器接口合并为一个 `foc_controller_if_t`。
- 删除或兼容别名化 `motor_step_controller_if_t`、`motor_tracking_controller_if_t`。
- `motor_control_runtime_config_t` 和 `motor_hf_plan_t` 不再维护重复接口类型。
- `motor_control_runtime_config_t` 使用 `tId/tIq/tSpeed/tPosition` 短字段名。
- 不再出现 `tControlConfig.tIqController.pController` 这类重复命名链。
- 高频路径仍通过函数指针直接调用，不增加每步 `switch` 或额外校验。
- 不改变现有控制行为、公开 API 和运行参数。

#### 验收标准

- `motor_step_controller_if_t` 和 `motor_tracking_controller_if_t` 不再被业务代码使用。
- `foc_controller_if_t` 能覆盖电流环、速度环、位置环的绑定与调用。
- 控制配置字段名不再重复表达 `control`/`controller`。
- FOC `float`、`fixed` 测试全部通过。
- STM32G431、AT32F413 目标构建通过。
- 高频路径的 cycle 开销没有明显增加，调用链保持为直接函数指针调用。

#### 注意事项

- 这是结构简化重构，不应改变 20 kHz 高频路径的执行顺序。
- 当前运行链路本身不算绕，绕的主要是类型定义和初始化/计划之间的转换。
- 如果项目当前需要稳定运行，可以按独立重构任务处理，不与其他功能改动混在一起。

#### 评估结论

- 状态：已评估（2026-08-08）
- 结论：建议实施。经源码核实，`motor_step_controller_if_t` / `motor_tracking_controller_if_t`
  是 `foc_controller_if_t` 的前缀子集，全库仅 9 处引用、3 个文件
  （`motor_control_types.h`、`motor_hf_private.h`、`motor_fsm.c`）。
  统一后 `motor_Init()` 逐字段拷贝可收敛为单次结构体赋值，`motor_fsm.c` 校验
  可复用 `foc_controller_IsValid()` / `foc_controller_CanTrack()`；
  高频路径 `plan->tId.fnStep()` 调用形式不变，无性能风险。
  命名同步收敛为 `control.tId/tIq/tSpeed/tPosition`（`pController` 由事项 2 落地）。
  注意：`foc_controller_if_t` 比 step 版多 2 个指针，需确认
  `motor_impl_t` 的 `_Static_assert` 存储预算仍有余量。
- 是否实施：是（阶段 A，最先实施）

### 事项 2：pContext 具名化与控制器接口可读性

#### 诉求

当前 FOC 源码大量使用 `pContext`，阅读者很难直接判断它到底是控制器、位置源、HAL 还是其他对象。希望在接口命名上更具现化，让阅读者能看出“这个 context 是什么”。

例如：

- 控制器接口的 `pContext` -> `pController`
- 位置源接口的 `pContext` -> `pSource`
- 高频 I/O 的 `pContext` -> `pIoContext`
- PWM/ADC HAL 的 `pContext` -> `pHalContext`
- 时间接口的 `pContext` -> `pTimeContext`
- 同步接口的 `pContext` -> `pSyncContext`

同时收敛长链命名，避免出现：

```c
ptImpl->tControlConfig.tIqController.pController
```

希望最终为：

```c
impl->control.tIq.pController
```

#### 现状

- `pContext` 出现在约 36 个文件、约 276 处。
- 覆盖控制器、位置源、HAL、高频 I/O、时间、同步等接口。
- 当前是 C 语言常见的回调上下文模式，优点是通用，缺点是阅读时需要看具体实现才能知道类型。

#### 期望结果

- 按接口域具名化 `pContext`，但不改变底层 `void *` 传递方式。
- 不引入真正的 C++ 式继承、运行时 vtable 或联合体对象模型。
- 20 kHz 高频路径保持直接函数指针调用，不增加每步间接开销。
- 不改变现有控制行为、公开 API 语义和运行参数。

#### 验收标准

- `pContext` 在业务接口中按域具名，阅读者能根据字段名判断其角色。
- 控制配置命名同步收敛为 `control.tId/tIq/tSpeed/tPosition` 形式。
- FOC `float`、`fixed` 测试全部通过。
- STM32G431、AT32F413 目标构建通过。
- 高频路径 cycle 开销没有明显增加。

#### 注意事项

- 这是可读性重构，不应为了“像面向对象”而引入运行时多态。
- 联合体保存具体对象指针的方案会增加后续控制器扩展成本，不建议作为首选。
- 建议与“事项 1：统一 FOC 控制器接口”一起统一评估，避免重复改动同一批接口。

#### 工作量参考

- 仅重命名 `pContext` 为具名上下文：约 0.5 到 1 天。
- 重命名并增加类型转换 helper：约 1 到 2 天。
- 改为真正的泛型对象/联合体/类化：约 2 到 5 天，且高频路径有回归风险。

#### 评估结论

- 状态：已评估（2026-08-08）
- 结论：建议实施，按接口域分 4 小批：控制器 `pContext → pController`、
  位置源 `→ pSource`、高频 I/O `→ pIoContext`、HAL/时间/同步
  `→ pHalContext / pTimeContext / pSyncContext`。核实全库约 315 处
  （foc/ 200 处 28 文件、tests/foc 61 处、peripheral 两个 MDI 适配器 54 处）。
  纯字段重命名，`void *` 传递与内存布局不变，零性能风险；但同一字段名在不同
  结构中含义不同，禁止全局 sed，必须按结构体逐个改，每小批后全量编译验证。
  泛型对象/联合体/类化方案确认排除。
- 是否实施：是（阶段 B，事项 1 完成后实施）

### 事项 3：motor 头文件成员命名一致性

#### 诉求

除 `control` 和 `pContext` 外，motor 头文件成员命名还存在以下影响理解的问题：

- 缩写太隐晦，例如 `tRt`、`tHfPlan`、`achPrivate`
- 同一概念在不同结构里命名/类型不一致
- `current`、`angle`、`speed`、`position` 等词被过度复用
- 公共配置和私有运行时副本命名不对称
- 前缀约定不够统一，例如 `w/ch/hw/n/b/t/q/e/p/fn/a`

希望统一评估后，再决定是否做一轮命名一致性整理。

#### 现状

- `motor_private.h` 使用 `tRt` 表示运行时状态，缩写不直观。
- 私有状态使用 `chStartupPhase`，快照使用 `eStartupPhase`。
- 私有状态使用 `chPendingCommand`，快照使用 `ePendingCommand`。
- 私有状态使用 `chActiveValidFlags`，快照使用 `eActiveSourceValidFlags`。
- 公共配置使用 `tControl`，运行时配置使用 `tControlConfig`。
- 公共配置使用 `tPosition`，运行时配置使用 `tPositionConfig`。
- `current` 相关成员有 `tCurrent`、`tPhaseCurrent`、`tCurrentReference`、`tCurrentAlphaBeta`、`tCurrentCalibration`。
- `angle` 相关成员有 `tOpenLoopAngle`、`tActiveAngle`、`tCandidateAngle`、`tElectricalAngle`、`tMechanicalAngle`、`tAngle`。
- `speed` 相关成员有 `qActiveSpeed`、`qCandidateSpeed`、`qElectricalSpeed`、`qMechanicalSpeed`、`qOpenLoopCommandSpeed`、`qSpeed`。
- `position` 相关成员有 `tPosition`、`tPositionConfig`、`tPositionSource`、`tPositionInput`、`tPositionOutput`。

#### 期望结果

- 缩写有明确规则，不使用过于隐晦的短名。
- 同一概念在私有状态、快照、公共配置中使用一致命名。
- `current`、`angle`、`speed`、`position` 按语义区分，例如 `Reference`、`Active`、`Candidate`、`Electrical`、`Mechanical`。
- 公共配置与私有运行时副本命名可以一眼区分。
- 前缀约定统一，并补充必要说明。

#### 验收标准

- 不改变现有控制行为、公开 API 语义和运行参数。
- FOC `float`、`fixed` 测试全部通过。
- STM32G431、AT32F413 目标构建通过。
- 高频路径 cycle 开销没有明显增加。

#### 注意事项

- 这是纯命名/可读性重构，不应改变功能。
- 建议与事项 1、事项 2 一起统一评估，避免同一批接口被反复修改。
- 不要求一次改完，可以先定义命名规范，再分批落地。

#### 评估结论

- 状态：已评估（2026-08-08）
- 结论：范围收窄后实施。`chStartupPhase` vs `eStartupPhase` 这类 `ch/e`
  差异是**有意的**（`ch` 前缀标记私有侧收窄为 `uint8_t` 存储，见
  `motor_private.h` 头部注释），保留并文档化约定，不强行统一。
  实际改动：`tRt → tRuntime` 等少数隐晦缩写；`current/angle/speed/position`
  语义前缀（`Reference/Active/Candidate/Electrical/Mechanical`）整理为命名
  规范文档，存量代码只改最误导的少数几个，不全量翻新。
  `tControl` / `tControlConfig` 不对称随事项 1 顺带解决。
- 是否实施：是（阶段 C，收窄范围后实施）

### 事项 4：motor_impl_t 结构复杂度和职责分层

#### 诉求

当前分析 motor 框架和成员分类比较困难。`motor_impl_t` 直接成员约 50 个，算上嵌套结构字段更多，并且把平台绑定、控制配置、高频计划、默认 PID、位置源、运行时状态、切换状态、事件、诊断等全部平铺在一起。

希望统一评估后，将 `motor_impl_t` 按职责拆成更清晰的子结构，降低阅读和分析成本，但不改变 `motor_handle_t` 固定存储模型。

#### 现状

- `motor_impl_t` 同时包含：
  - HAL、时间、同步等平台接口
  - 控制配置和默认 PID 实例
  - 高频执行计划
  - 位置源接口
  - 高频命令/状态
  - 低频/运行状态
  - 切换状态
  - 事件缓冲区
  - 性能分析快照
  - 各种标志位
- 成员没有按“平台、控制、高频、低频、事件、诊断”等职责明显分组。
- 多个相似概念命名接近，例如 `tPosition`、`tPositionConfig`、`tPositionSource`、`tPositionInput`、`tPositionOutput`。
- 私有状态和快照命名不一致，例如 `chStartupPhase` vs `eStartupPhase`。

#### 期望结果

- `motor_impl_t` 按职责拆成若干子结构，例如：
  - 平台接口
  - 控制绑定
  - 高频运行时
  - 低频/运行状态
  - 切换状态
  - 事件与诊断
- 每个子结构职责单一，成员归属清晰。
- 不改变 `motor_handle_t` 的固定存储容量模型。
- 不改变公开 API、控制行为和高频执行顺序。

#### 验收标准

- 阅读 `motor_impl_t` 时能按子结构判断成员归属。
- `motor_handle_t` 仍是固定大小存储，不引入动态分配。
- FOC `float`、`fixed` 测试全部通过。
- STM32G431、AT32F413 目标构建通过。
- 高频路径 cycle 开销没有明显增加。

#### 注意事项

- 这是私有结构整理，不直接暴露给应用层。
- 拆分应保持内存布局稳定或同步更新 static assert。
- 建议与事项 1、事项 2、事项 3 一起统一评估，避免同一批结构被反复修改。

#### 评估结论

- 状态：已评估（2026-08-08）
- 结论：建议最后单独实施，可裁剪。`motor_impl_t` 当前按对齐降序排布是刻意的
  内存优化，拆分子结构时内部仍需保持对齐降序以避免填充增长，`sizeof` 必须
  仍 ≤ `MOTOR_HANDLE_STORAGE_SIZE`（`_Static_assert` 兜底）。
  `tHfPlan` / `tHfCommand` / `tHfState` 三个高频热成员须保持相邻，
  维持 20 kHz ISR 路径的 D-cache 局部性；改动集中在 motor 实现层 6 个文件，
  量大但机械。完成后实机跑车对照 `foc/doc/temprundata.txt` 基线
  （开环约 2352–2378、闭环约 3176–3194 cycles）终验。
- 是否实施：是（阶段 D，事项 1–3 稳定后单独实施）

## 新增事项模板

后续新增事项可以复制以下结构：

### 事项 N：标题

#### 诉求

...

#### 现状

...

#### 期望结果

...

#### 验收标准

...

#### 注意事项

...

#### 评估结论

- 状态：待评估
- 结论：
- 是否实施：
