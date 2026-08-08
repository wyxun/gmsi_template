# FOC 控制器接口统一与命名收敛 (阶段 A) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 统一 FOC 控制器接口为 `foc_controller_if_t`，清理 `motor_step_controller_if_t` 与 `motor_tracking_controller_if_t` 冗余类型，并将控制配置字段命名收敛为 `tId`, `tIq`, `tSpeed`, `tPosition`。

**Architecture:** 替换 `motor_control_types.h` 和 `motor_hf_private.h` 中的重叠类型与字段名，更新 `motor.c`、`motor_hf.c` 的结构体赋值与 `motor_fsm.c` 的有效性校验，保持 20 kHz ISR 路径为直接函数指针调用。

**Tech Stack:** C99, MCU Embedded Bare-metal (AT32F413 / STM32G431), GCC / MinGW.

---

### Task 1: 移除重叠控制器类型并更新配置结构体定义

**Files:**
- Modify: `foc/motor/motor_control_types.h:35-62`
- Modify: `foc/motor/motor_hf_private.h:41-43`

- [ ] **Step 1: 修改 `motor_control_types.h` 类型定义**

在 [motor_control_types.h](file:///e:/Project/modus_template/foc/motor/motor_control_types.h) 中删除 `motor_step_controller_if_t` 和 `motor_tracking_controller_if_t` 结构体，更新 `motor_control_config_t` 与 `motor_control_runtime_config_t` 为统一接口和短缩短字段名：

```c
typedef struct {
    foc_pid_params_t    tIdParams;          /**< D 轴 PID 参数 */
    foc_pid_params_t    tIqParams;          /**< Q 轴 PID 参数 */
    foc_pid_params_t    tSpeedParams;       /**< 速度环 PID 参数 */
    foc_pid_params_t    tPositionParams;    /**< 位置环 PID 参数 */
    foc_controller_if_t tId;                /**< D 轴控制器接口 */
    foc_controller_if_t tIq;                /**< Q 轴控制器接口 */
    foc_controller_if_t tSpeed;             /**< 速度环控制器接口 */
    foc_controller_if_t tPosition;          /**< 位置环控制器接口 */
    motor_modulation_e eModulation;         /**< 调制方式 */
} motor_control_config_t;

typedef struct {
    foc_controller_if_t tId;                /**< D 轴控制器 */
    foc_controller_if_t tIq;                /**< Q 轴控制器 */
    foc_controller_if_t tSpeed;             /**< 速度环控制器 */
    foc_controller_if_t tPosition;          /**< 位置环控制器 */
    motor_modulation_e eModulation;         /**< 调制方式 */
} motor_control_runtime_config_t;
```

- [ ] **Step 2: 修改 `motor_hf_private.h` 高频 Plan 结构体定义**

在 [motor_hf_private.h](file:///e:/Project/modus_template/foc/motor/motor_hf_private.h) 中更新 `motor_hf_plan_t`：

```c
typedef struct {
    foc_hf_io_if_t      tIo;                /**< 高频采样/提交/急停接口 */
    void               *pSourceContext;     /**< 位置源上下文 */
    foc_result_t       (*fnSourceStep)(void *, const foc_position_input_t *,
                                 foc_position_output_t *); /**< 位置源步进 */
    foc_controller_if_t tId;                /**< D 轴电流环控制器 */
    foc_controller_if_t tIq;                /**< Q 轴电流环控制器 */
    motor_hf_modulate_fn_t fnModulate;      /**< 调制函数 */
    foc_scalar_t        qPeriod;            /**< 高频步周期 */
    foc_position_config_t tPositionConfig;  /**< 位置配置 */
} motor_hf_plan_t;
```

---

### Task 2: 更新初始化逻辑、高频 Plan 解析与状态机校验

**Files:**
- Modify: `foc/motor/motor.c`
- Modify: `foc/motor/motor_hf.c`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_control.c`

- [ ] **Step 1: 更新 `motor.c` 中的控制配置初始化**

将 `motor_Init()` 内对控制器成员的拷贝收敛为：

```c
ptImpl->tControl.tId       = ptConfig->tControl.tId;
ptImpl->tControl.tIq       = ptConfig->tControl.tIq;
ptImpl->tControl.tSpeed    = ptConfig->tControl.tSpeed;
ptImpl->tControl.tPosition = ptConfig->tControl.tPosition;
```

- [ ] **Step 2: 更新 `motor_hf.c` 高频 Plan 填充与 20 kHz 执行步骤**

在 `plan resolver` 中直接进行结构体赋值：
```c
ptPlan->tId = ptImpl->tControl.tId;
ptPlan->tIq = ptImpl->tControl.tIq;
```

在高频步进计算中访问 `plan->tId` 与 `plan->tIq`：
```c
foc_scalar_t qUd = plan->tId.fnStep(plan->tId.pContext, qIdRef, state->tCurrent.qD);
foc_scalar_t qUq = plan->tIq.fnStep(plan->tIq.pContext, qIqRef, state->tCurrent.qQ);
```

- [ ] **Step 3: 更新 `motor_fsm.c` 中的有效性与 Track 能力校验**

复用 `foc_controller_IsValid()` 与 `foc_controller_CanTrack()`，将原先对 `tSpeedController` / `tIqController` 等字段的访问改为新的短字段名 `tSpeed` / `tIq` / `tId` / `tPosition`。

- [ ] **Step 4: 更新 `motor_control.c` 中的动态解耦与串级闭环逻辑**

将涉及 `ptImpl->tControl.tSpeedController` 等旧字段的引用统一修正为 `ptImpl->tControl.tSpeed` 等新字段名。

---

### Task 3: 全量编译构建与测试集验证

**Files:**
- Test: `tests/foc/`
- Target build: `Makefile`, `make.bat`

- [ ] **Step 1: 验证 AT32F413 目标编译与静态断言**

运行：`.\make.bat`
预期：编译成功，无类型不匹配警告，`_Static_assert(sizeof(motor_handle_t) == MOTOR_HANDLE_STORAGE_SIZE)` 校验通过。

- [ ] **Step 2: 验证 STM32G431 目标编译**

运行：`mingw32-make TARGET_CHIP=stm32g431`
预期：编译成功，无错误与警告。

- [ ] **Step 3: 运行 FOC 单元测试集**

运行：`mingw32-make -C tests/foc`
预期：FOC 测试用例全量 Pass。
