# pContext 按域具名化设计规范 (阶段 B)

## 1. 摘要与背景

在原 FOC 框架中，大量回调接口与上下文结构统一使用通用的 `pContext`（全局约 315 处），导致阅读代码时无法仅通过字段名判断其具体类型和业务职责。

本规范针对 `docs/todo.md` 中的“事项 2：pContext 具名化与控制器接口可读性”，按接口所在的 4 个业务域对 `pContext` 进行类型自解释的具名化更名。更名仅修改字段名与形参名，保持 `void *` 指针传递机制和底层内存布局完全不变，确保零性能与内存开销。

---

## 2. 4 个接口域具名化规范

### 2.1 小批 B1：控制器域 (`foc_controller_if_t`)

将 [foc_controller.h](file:///e:/Project/modus_template/foc/control/foc_controller.h) 及底层控制器 (PID / LADRC / SMC / STA / DOB) 的上下文指针从 `pContext` 具名化为 `pController`：

```c
typedef struct {
    void *pController;                                  /**< 控制器实例上下文 */
    void (*fnReset)(void *pController);                 /**< 复位函数 */
    foc_scalar_t (*fnStep)(void *pController,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);     /**< 步进计算函数 */
    void (*fnTrack)(void *pController,
                    foc_scalar_t qOutput,
                    foc_scalar_t qReference,
                    foc_scalar_t qFeedback);            /**< 无缝跟踪函数 */
} foc_controller_if_t;
```

**连贯调用效果**：
`impl->control.tId.pController`
`impl->control.tId.fnStep(impl->control.tId.pController, qRef, qFb)`

---

### 2.2 小批 B2：位置源域 (`foc_position_source_if_t`)

将 [motor_position.h](file:///e:/Project/modus_template/foc/motor/motor_position.h) 及各位置观察者 (Encoder / Hall / SMO / NLFO / HFI / OpenLoopSource) 的上下文指针具名化为 `pSourceContext`：

```c
typedef struct {
    void *pSourceContext;               /**< 位置源实例上下文 */
    void (*fnReset)(void *pSourceContext);
    foc_result_t (*fnStep)(void *pSourceContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

**连贯调用效果**：
`impl->tPositionSource.pSourceContext`
`impl->tPositionSource.fnStep(impl->tPositionSource.pSourceContext, &tInput, &tOutput)`

---

### 2.3 小批 B3：高频 I/O 域 (`foc_hf_io_if_t`)

将 [foc_hal.h](file:///e:/Project/modus_template/foc/hal/foc_hal.h) 中的 `foc_hf_io_if_t` 硬件上下文具名化为 `pIoContext`：

```c
typedef struct {
    void *pIoContext;                   /**< 高频 I/O 硬件上下文 */
    foc_result_t (*fnSampleCurrent)(void *pIoContext, foc_phase_current_t *ptCurrent);
    foc_result_t (*fnCommitDuty)(void *pIoContext, const foc_duty_abc_t *ptDuty);
    void (*fnEmergencyStop)(void *pIoContext);
} foc_hf_io_if_t;
```

**连贯调用效果**：
`plan->tIo.fnSampleCurrent(plan->tIo.pIoContext, &state->tPhaseCurrent)`

---

### 2.4 小批 B4：HAL / 时间 / 同步域

将 [foc_hal.h](file:///e:/Project/modus_template/foc/hal/foc_hal.h) 与 [motor_types.h](file:///e:/Project/modus_template/foc/motor/motor_types.h) 中的抽象硬件接口具名化：

1. **PWM HAL 接口 (`foc_pwm_if_t`)** -> `pHalContext`
2. **ADC HAL 接口 (`foc_adc_if_t`)** -> `pHalContext`
3. **时间接口 (`motor_time_if_t`)** -> `pTimeContext`
   `impl->tTime.fnGetMilliseconds(impl->tTime.pTimeContext)`
4. **同步接口 (`motor_sync_if_t`)** -> `pSyncContext`
   `impl->tSync.fnEnter(impl->tSync.pSyncContext)`

---

## 3. 实施策略与验证

因为同一字段名 `pContext` 在不同结构体中含义不同，为防止全量误替换引发的语法问题，必须按小批次逐个模块推进：

1. **小批 B1 实施**：更名 `foc_controller_if_t` 及其导出回调，全量编译并跑通测试。
2. **小批 B2 实施**：更名 `foc_position_source_if_t` 及其导出回调，全量编译并跑通测试。
3. **小批 B3 实施**：更名 `foc_hf_io_if_t` 及其导出回调，全量编译并跑通测试。
4. **小批 B4 实施**：更名 PWM/ADC/Time/Sync 接口及其导出回调，全量编译并跑通测试。

每次更名后，必须验证：
- `.\make.bat` (AT32F413) 编译通过
- `mingw32-make TARGET_CHIP=stm32g431` 编译通过
- `mingw32-make -C tests/foc` 单元测试全量 Pass
