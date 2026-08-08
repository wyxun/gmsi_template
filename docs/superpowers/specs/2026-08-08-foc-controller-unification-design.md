# FOC 控制器接口统一与命名收敛设计规范 (阶段 A)

## 1. 摘要与背景

在当前 FOC 控制器实现中，存在三个高度重叠的控制器接口类型：
- `foc_controller_if_t` (公共层接口，包含 `pContext`, `fnReset`, `fnStep`, `fnTrack`)
- `motor_step_controller_if_t` (仅包含 `pContext`, `fnStep`)
- `motor_tracking_controller_if_t` (仅包含 `pContext`, `fnStep`, `fnTrack`)

这种接口划分增加了类型转换与重叠定义的开销，且在配置结构体中存在冗长的命名链（如 `tControlConfig.tIqController.pController`）。

本设计规范针对 `docs/todo.md` 中的“事项 1：统一 FOC 控制器接口并收敛控制命名”，将控制器接口统一为 `foc_controller_if_t`，清理重叠类型，并收敛字段命名为 `tId`, `tIq`, `tSpeed`, `tPosition`。

---

## 2. 设计细节与数据结构变动

### 2.1 删除重叠类型 (`motor_control_types.h`)

从 [motor_control_types.h](file:///e:/Project/modus_template/foc/motor/motor_control_types.h) 中完全移除以下结构体定义：
- `motor_step_controller_if_t`
- `motor_tracking_controller_if_t`

### 2.2 控制配置与运行时配置结构体调整

更新 `motor_control_config_t` 和 `motor_control_runtime_config_t` 结构体：

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

### 2.3 高频 Plan 结构体调整 (`motor_hf_private.h`)

在 [motor_hf_private.h](file:///e:/Project/modus_template/foc/motor/motor_hf_private.h) 中，将高频 Plan 中的 D/Q 电流环成员升级为 `foc_controller_if_t` 类型并缩短字段名：

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

## 3. 业务代码与状态机修改

### 3.1 结构体赋值收敛 (`motor.c` & `motor_hf.c`)

- 在 `motor_Init()` 逻辑中，由原先的手动解包 `pContext` / `fnStep` 改为直接单项结构体赋值：
  ```c
  ptImpl->tControl.tId       = ptConfig->tControl.tId;
  ptImpl->tControl.tIq       = ptConfig->tControl.tIq;
  ptImpl->tControl.tSpeed    = ptConfig->tControl.tSpeed;
  ptImpl->tControl.tPosition = ptConfig->tControl.tPosition;
  ```
- 在 `motor_hf.c` 的 Plan 填充阶段，直接进行结构体拷贝：
  ```c
  ptPlan->tId = ptImpl->tControl.tId;
  ptPlan->tIq = ptImpl->tControl.tIq;
  ```

### 3.2 状态机校验统一 (`motor_fsm.c`)

复用 [foc_controller.h](file:///e:/Project/modus_template/foc/control/foc_controller.h) 导出的 Helper 函数：
- 使用 `foc_controller_IsValid(&ptControl->tId)` 验证控制器指针有效性。
- 使用 `foc_controller_CanTrack(&ptControl->tSpeed)` 验证跟踪能力。

### 3.3 高频 ISR 执行路径 (`motor_hf.c`)

保持 20 kHz ISR 路径的直接函数指针调用不变：
```c
foc_scalar_t qUd = plan->tId.fnStep(plan->tId.pContext, qIdRef, state->tCurrent.qD);
foc_scalar_t qUq = plan->tIq.fnStep(plan->tIq.pContext, qIqRef, state->tCurrent.qQ);
```

---

## 4. 影响范围与内存预算验证

### 4.1 内存预算与静态断言
- `foc_controller_if_t` 占用 4 个指针大小（32 位系统上 16 字节）。
- 在 `motor_impl_t` 中，受影响结构体 `motor_control_runtime_config_t` 和 `motor_hf_plan_t` 仅净增 32 字节。
- `MOTOR_HANDLE_STORAGE_SIZE` 预算（896/960 字节）留有充足空间，`_Static_assert` 保持通过。

### 4.2 涉及文件清单
1. [motor_control_types.h](file:///e:/Project/modus_template/foc/motor/motor_control_types.h)
2. [motor_hf_private.h](file:///e:/Project/modus_template/foc/motor/motor_hf_private.h)
3. [motor_fsm.c](file:///e:/Project/modus_template/foc/motor/motor_fsm.c)
4. [motor.c](file:///e:/Project/modus_template/foc/motor/motor.c)
5. [motor_hf.c](file:///e:/Project/modus_template/foc/motor/motor_hf.c)
6. [motor_control.c](file:///e:/Project/modus_template/foc/motor/motor_control.c)

---

## 5. 验证计划

1. **编译检查**：
   - 执行 `.\make.bat` 验证 AT32F413 目标编译通过。
   - 执行 `mingw32-make TARGET_CHIP=stm32g431` 验证 STM32G431 目标编译通过。
2. **静态断言**：
   - 确保 `sizeof(motor_handle_t) == MOTOR_HANDLE_STORAGE_SIZE` 验证成立。
3. **算法逻辑单元测试**：
   - 运行 `tests/foc` 测试集合，确认 FOC 运算准确性。
