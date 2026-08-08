# motor 模块成员命名一致性 (阶段 C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 清理 `motor_impl_t` 中的隐晦缩写 `tRt → tRuntime`，并在规范文档中正式建立 `ch/e` 内存优化前缀约定。

**Architecture:** 替换 `motor_private.h`、`motor.c`、`motor_fsm.c`、`motor_control.c`、`motor_hf.c` 以及测试文件中的 `tRt` 为 `tRuntime`，并完成交叉编译与单元测试验证。

**Tech Stack:** C99, MCU Embedded Bare-metal (AT32F413 / STM32G431), GCC / MinGW.

---

### Task 1: 替换 `motor_private.h` 中的 `tRt` 成员

**Files:**
- Modify: `foc/motor/motor_private.h`

- [ ] **Step 1: 将 `motor_impl_t` 中的 `tRt` 修改为 `tRuntime`**

在 [motor_private.h](file:///e:/Project/modus_template/foc/motor/motor_private.h) 中：

```c
typedef struct motor_impl_s {
    uint32_t                    wMagic;             /**< 魔数 MOTOR_IMPL_MAGIC */
    motor_state_t               tRuntime;           /**< 运行时状态快照（只读导出） */
    ...
} motor_impl_t;
```

---

### Task 2: 替换底层逻辑模块中的 `tRt` 访问

**Files:**
- Modify: `foc/motor/motor.c`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_control.c`
- Modify: `foc/motor/motor_hf.c`

- [ ] **Step 1: 更新 `motor_fsm.c` 中的 `impl->tRt` 为 `impl->tRuntime`**
- [ ] **Step 2: 更新 `motor_hf.c` 中的 `impl->tRt` 为 `impl->tRuntime`**
- [ ] **Step 3: 更新 `motor_control.c` 与 `motor.c` 中的 `impl->tRt` 为 `impl->tRuntime`**

---

### Task 3: 适配测试用例并全量编译/测试

**Files:**
- Modify: `tests/foc/test_motor.c`, `tests/foc/test_motor_fsm.c` (若有)
- Test: MCU Build & Unit Tests

- [ ] **Step 1: 执行 `.\make.bat` 验证默认 AT32F413 目标**
- [ ] **Step 2: 执行 `mingw32-make TARGET_CHIP=stm32g431` 验证 STM32G431 目标**
- [ ] **Step 3: 执行 `mingw32-make -C tests/foc` 运行全量单元测试 (0 Failures)**
