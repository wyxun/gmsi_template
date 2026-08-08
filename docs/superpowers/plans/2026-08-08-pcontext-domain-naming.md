# pContext 按域具名化 (阶段 B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将全库抽象通用的 `pContext` 按 4 个业务域具名化为 `pController`、`pSourceContext`、`pIoContext`、`pHalContext` / `pTimeContext` / `pSyncContext`，在保留 `void *` 透明传递的前提下全面提升连贯可读性。

**Architecture:** 按照 4 个独立的子域分小批推进修改，每小批完成后必须执行 AT32F413 / STM32G431 的交叉编译及 `tests/foc` 的单元测试。

**Tech Stack:** C99, MCU Embedded Bare-metal (AT32F413 / STM32G431), GCC / MinGW.

---

### Task 1: 小批 B1 控制器域具名化 (`pContext → pController`)

**Files:**
- Modify: `foc/control/foc_controller.h`
- Modify: `foc/control/foc_controller.c`
- Modify: `foc/control/foc_pid.h` / `foc_pid.c`
- Modify: `foc/control/foc_ladrc.h` / `foc_ladrc.c`
- Modify: `foc/control/foc_smc.h` / `foc_smc.c`
- Modify: `foc/control/foc_sta.h` / `foc_sta.c`
- Modify: `foc/control/foc_dob.h` / `foc_dob.c`
- Modify: `foc/motor/motor_hf.c`, `foc/motor/motor_control.c`, `foc/motor/motor.c`
- Test: `tests/foc/`

- [ ] **Step 1: 修改 `foc_controller.h` 结构体成员与回调函数形参名**

在 [foc_controller.h](file:///e:/Project/modus_template/foc/control/foc_controller.h) 中将 `foc_controller_if_t` 结构体中的 `pContext` 重命名为 `pController`：

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

- [ ] **Step 2: 修改 `foc_controller.c` 中的 helper 实现**

在 [foc_controller.c](file:///e:/Project/modus_template/foc/control/foc_controller.c) 中更新 `Reset` / `Step` / `Track` 等调用的属性访问为 `ptController->pController`。

- [ ] **Step 3: 修改各控制器导出模块 (.h/.c) 及业务/测试调用处**

在 PID / LADRC / SMC / STA / DOB 等模块导出 `.pController` 接口属性，并更新 `motor_hf.c`、`motor_control.c`、`motor.c` 及 `tests/foc/` 测试用例中的关联访问。

- [ ] **Step 4: 编译与单元测试验证**

运行 `.\make.bat` 和 `mingw32-make -C tests/foc` 确认全量 Pass。

---

### Task 2: 小批 B2 位置源域具名化 (`pContext → pSourceContext`)

**Files:**
- Modify: `foc/motor/motor_position.h`
- Modify: `foc/motor/motor_position.c`
- Modify: `foc/observer/foc_encoder.c`, `foc/observer/foc_hall.c`, `foc/observer/foc_smo.c`, `foc/observer/foc_nlfo.c`, `foc/observer/foc_hfi.c`, `foc/observer/foc_open_loop_source.c`
- Modify: `foc/motor/motor_hf.c`, `foc/motor/motor_fsm.c`
- Test: `tests/foc/`

- [ ] **Step 1: 修改 `motor_position.h` 中的位置源接口**

在 [motor_position.h](file:///e:/Project/modus_template/foc/motor/motor_position.h) 中将 `foc_position_source_if_t` 结构体中的 `pContext` 重命名为 `pSourceContext`：

```c
typedef struct {
    void *pSourceContext;               /**< 位置源实例上下文 */
    void (*fnReset)(void *pSourceContext);
    foc_result_t (*fnStep)(void *pSourceContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

- [ ] **Step 2: 修改各观察者算法 (.c) 导出的接口字面量及回调形参**

在 Encoder / Hall / SMO / NLFO / HFI / OpenLoopSource 等实现文件中更新 `.pSourceContext` 属性。

- [ ] **Step 3: 修改 `motor_hf.c` / `motor_fsm.c` 及测试用例**

更新 `plan->pSourceContext` / `impl->tPositionSource.pSourceContext` 以及 `tests/foc/` 中的引用。

- [ ] **Step 4: 编译与单元测试验证**

运行 `.\make.bat` 和 `mingw32-make -C tests/foc` 确认全量 Pass。

---

### Task 3: 小批 B3 高频 I/O 域具名化 (`pContext → pIoContext`)

**Files:**
- Modify: `foc/hal/foc_hal.h`
- Modify: `foc/hal/foc_hal.c`
- Modify: `peripheral/at32f413/foc_hal_mdi_adapter.c` (以及相关外设适配器)
- Modify: `foc/motor/motor_hf.c`, `foc/motor/motor.c`
- Test: `tests/foc/`

- [ ] **Step 1: 修改 `foc_hal.h` 中的 `foc_hf_io_if_t` 接口**

在 [foc_hal.h](file:///e:/Project/modus_template/foc/hal/foc_hal.h) 中将 `foc_hf_io_if_t` 中的 `pContext` 重命名为 `pIoContext`：

```c
typedef struct {
    void *pIoContext;                   /**< 高频 I/O 硬件上下文 */
    foc_result_t (*fnSampleCurrent)(void *pIoContext, foc_phase_current_t *ptCurrent);
    foc_result_t (*fnCommitDuty)(void *pIoContext, const foc_duty_abc_t *ptDuty);
    void (*fnEmergencyStop)(void *pIoContext);
} foc_hf_io_if_t;
```

- [ ] **Step 2: 修改 HAL 适配层实现及高频 Fast Path**

更新 `foc_hal.c`、`foc_hal_mdi_adapter.c` 及 `motor_hf.c` 中对 `pIoContext` 的访问。

- [ ] **Step 3: 编译与单元测试验证**

运行 `.\make.bat` 和 `mingw32-make -C tests/foc` 确认全量 Pass。

---

### Task 4: 小批 B4 HAL / 时间 / 同步域具名化 (`pHalContext` / `pTimeContext` / `pSyncContext`)

**Files:**
- Modify: `foc/hal/foc_hal.h` (`foc_pwm_if_t`, `foc_adc_if_t`)
- Modify: `foc/motor/motor_types.h` (`motor_time_if_t`, `motor_sync_if_t`)
- Modify: `foc/hal/foc_hal.c`, `foc/motor/motor.c`, `foc/motor/motor_fsm.c`
- Modify: 外设与测试用例文件

- [ ] **Step 1: 修改 PWM / ADC HAL 接口上下文属性名**

在 [foc_hal.h](file:///e:/Project/modus_template/foc/hal/foc_hal.h) 中将 `foc_pwm_if_t` 和 `foc_adc_if_t` 中的 `pContext` 重命名为 `pHalContext`。

- [ ] **Step 2: 修改时间与同步接口上下文属性名**

在 [motor_types.h](file:///e:/Project/modus_template/foc/motor/motor_types.h) 中分别更名为 `pTimeContext` 与 `pSyncContext`：

```c
typedef struct {
    void *pTimeContext;                 /**< 时间接口上下文 */
    uint32_t (*fnGetMilliseconds)(void *pTimeContext);
} motor_time_if_t;

typedef struct {
    void *pSyncContext;                 /**< 临界区同步上下文 */
    uintptr_t (*fnEnter)(void *pSyncContext);
    void (*fnExit)(void *pSyncContext, uintptr_t wState);
} motor_sync_if_t;
```

- [ ] **Step 3: 全量搜索确认零残留 `pContext` 泛化调用**

全库搜索确认没有遗漏的 `pContext` 旧接口属性。

- [ ] **Step 4: 交叉编译与全量单元测试终验**

运行 `.\make.bat`（AT32F413）、`mingw32-make TARGET_CHIP=stm32g431` 以及 `mingw32-make -C tests/foc` 确认全部通过。
