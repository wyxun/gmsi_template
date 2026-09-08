# FOC Motor Object Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **实施状态（2026-09-08）:** 当前工作树已完成基线锁定、独立 Motor 合约/生命周期、Core 状态去重、AS5600 provider，以及三种控制模式的 App wrapper；编码器零位完整对齐流程仍保留 App 的非阻塞服务，等待专项状态机回归后再下沉。这样可以先让当前 AS5600 速度闭环保持原数值口径，再逐步收缩 `foc_app`。

**Goal:** 将当前 `foc_app_t` 中的单电机控制状态迁移到独立 `motor_t`，并在整个迁移过程中保持 AS5600 速度闭环、20 kHz 高频安全时序和现有可观测行为。

**Architecture:** 采用设计文档中的方案 A：`motor_t` 只注册一个活动位置 provider，通过 `ops + context` 连接 ADC、PWM 和位置源。`foc_app_t` 最终只负责 MODUS、Shell、波形和产品级调度；AS5600 在 1 kHz 慢速任务中访问 I2C，20 kHz 高频路径只读取缓存。迁移按“基线测试 → 独立 Motor → 高频生命周期 → AS5600 provider → App wrapper 切换 → 固件/台架验收”顺序进行，每一步保持可构建、可回退。Task 7 在没有 App 层等价回归前不得切换默认控制路径。

**Tech Stack:** C11、GCC/MSYS2 (`D:\0_software\msys64\mingw64\bin`)、float/Q16.15 `foc_scalar_t`、现有 FOC Core/PID/SVPWM、MODUS ops 接口、主机生命周期测试和 STM32G431 固件构建。

---

## Baseline and non-regression contract

本计划以 `39adfe5` 为代码基线，同时保留当前工作树中用户已有的 `makefile`、
`modus`、`diagrams/` 和设计文档修改。不得执行 `git reset`、`git checkout`、
分支切换、提交或推送。

以下行为在迁移期间必须由测试或现场记录锁定：

1. AS5600 的 I2C 读取只能发生在 1 kHz 慢速更新；20 kHz `motor_HighFrequencyStep()` 不得访问 I2C、日志、阻塞等待或任务 PT。
2. 高频运行顺序保持为：消费命令 → ADC 采样 → 位置读取/观察 → FOC Core → duty 提交；采样、位置、Core 或 duty 失败时先急停，再进入 FAULT。
3. ADC 校准期间不提交工作 duty、不使能 PWM；完成后先提交 0.5/0.5/0.5 中性 duty，再使能 PWM。
4. 速度模式仍由 1 kHz 速度 PI 生成 Iq 参考；CURRENT 模式的固定 Iq 不得被速度 PI 覆盖。
5. 机械速度到电速度仍按 `chPolePairs` 换算，方向反相和电气零位偏移的数值口径不变。
6. 2026-09-07 的实测记录（cal offset 极差 0.0051 turn、Id ref 0.10 → actual 0.110、Vd=0.108、100 eHz 的 5 s 波形 mean 98.5 eHz/std 4.9/range 95-105）作为台架回归基线；此前“48 eHz 上限”不再作为验收标准。
7. 5-30 eHz 的 AS5600 1 kHz 量化台阶和约 ±1.7 eHz 误差属于已知限制；50+ eHz 平滑度应保持，不能通过在高频路径加 I2C 或未经评审的插值改变现象。

## File map

新增：

- `foc/motor/motor.h`：`motor_t`、初始化配置、命令、反馈/状态查询和公开 API。
- `foc/motor/motor.c`：命令同步、生命周期、ADC 校准、高频步、慢速步和速度环编排。
- `foc/motor/motor_params.h`：`Rs/Ld/Lq/Flux/PolePairs` 及有效位。
- `foc/motor/motor_position.h`：单活动位置 provider 的最小 `ops + context` 接口。
- `tests/foc/test_motor.c`：float/fixed 共享的 Motor 主机测试和 stub provider。

修改：

- `foc/app/foc_app.h/.c`：保留 App 外壳和兼容 API，逐步改为组合 `motor_t`。
- `foc/hal/foc_port.h`：ADC/PWM 回调增加 `void *pContext`，移除通用层对编码器专属参数的依赖。
- `foc/middleware/foc_core.h/.c`：删除位置角度、速度和三相输入的持久副本，改由每拍输入提供。
- `foc/observer/foc_encoder.h/.c`：保留编码器滤波/超时状态，适配 `motor_position_ops_t`。
- `peripheral/driver/as5600.h/.c`：保留 1 kHz 慢速 I2C 和无锁缓存读取，增加 Motor provider 适配。
- `peripheral/stm32g431/foc_port.c`：绑定 ADC/PWM/AS5600 ops 与 context。
- `foc/foc.mk`、`tests/foc/Makefile`：加入 Motor 源文件和 float/fixed 测试目标。
- `tests/foc/test_foc_minimal_lifecycle.c`：迁移前后 App API 和 AS5600 速度闭环行为回归。
- `foc/README.md`、`foc/doc/foc-architecture.md`：更新所有权、高频边界和测量基线。

## Task 1: Freeze the current AS5600 speed-loop behavior

**Files:**

- Modify: `tests/foc/test_foc_minimal_lifecycle.c`
- Test: existing `minimal-lifecycle-float` and `minimal-lifecycle-fixed` targets

- [ ] **Step 1: Add a deterministic speed-loop fixture before changing production code.**

  Extend the existing injected sensor stub with a controllable mechanical speed and add a test that sets the sensor to `100 / 7` eHz mechanical speed, starts `FOC_MODE_SPEED` with a 100 eHz electrical reference, runs one 1 kHz hook, and asserts that the current reference is no longer the zero-speed value. Keep the existing calibrated-encoder and fixed-Iq tests unchanged.

  The assertion must use the existing public test hook and no private `foc_app_t` field:

  ```c
  s_tFakeSensor.qMechanicalSpeed = FOC_SCALAR(100.0f / 7.0f);
  tCommand.eMode = FOC_MODE_SPEED;
  tCommand.qSpeedReference = FOC_SCALAR(100.0f);
  foc_app_TestMarkEncoderCalibrated(&s_tAppA);
  foc_app_Start(&s_tAppA, &tCommand);
  run_isr(&s_tAppA, 512U);
  foc_app_TestRun1kHz(&s_tAppA);
  if (foc_app_TestGetCurrentIqReference(&s_tAppA) == FOC_ZERO) {
      return 1;
  }
  ```

- [ ] **Step 2: Run the new test and confirm the baseline is green.**

  Run from PowerShell:

  ```powershell
  $env:Path = "D:\0_software\msys64\mingw64\bin;D:\0_software\msys64\usr\bin;$env:Path"
  mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc clean
  mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc minimal
  ```

  Expected: core, encoder and lifecycle float/fixed executables all report `PASS (0 failures)`, including the new speed-loop case.

- [ ] **Step 3: Record the source data flow before extraction.**

  Confirm with `rg` that `foc_app_SpeedLoop()` consumes `tCore.qElectricalSpeed`, writes only the active command's Iq reference, and that `foc_app_EncoderPoll()` is the only AS5600 slow-side update path. Save no generated binaries or logs in Git.

## Task 2: Add the independent Motor contracts

**Files:**

- Create: `foc/motor/motor_params.h`
- Create: `foc/motor/motor_position.h`
- Create: `foc/motor/motor.h`
- Create: `tests/foc/test_motor.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Write compile-and-contract tests first.**

  Add tests for null configuration, missing ADC/PWM/position ops, zero pole pairs, one valid provider, and two identical ops bound to two distinct contexts. The test must assert that `motor_Init()` copies only runtime-required values and never stores a `motor_control_cfg_t` object as a member.

  The provider contract used by the test must be:

  ```c
  typedef struct {
      foc_angle_t tElectricalAngle;
      foc_scalar_t qElectricalSpeed;
      bool bValid;
  } motor_position_feedback_t;

  typedef struct {
      foc_result_t (*fnInit)(void *pContext,
                             const motor_params_t *ptMotor,
                             foc_scalar_t qHighFrequencyPeriod);
      void (*fnReset)(void *pContext);
      int32_t (*fnSlowUpdate)(void *pContext);
      foc_result_t (*fnObserve)(void *pContext,
                                const foc_observer_input_t *ptInput);
      foc_result_t (*fnRead)(void *pContext,
                             motor_position_feedback_t *ptFeedback);
      foc_result_t (*fnCaptureElectricalZero)(void *pContext);
  } motor_position_ops_t;
  ```

- [ ] **Step 2: Run only the new test target and verify the expected RED state.**

  Run `mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc motor-float` after adding the test source but before adding the Motor implementation. Expected: compilation fails because the new Motor symbols are not defined; fix only test/include errors until the failure is specifically missing production symbols.

- [ ] **Step 3: Implement the minimum data contracts.**

  Define `motor_params_t` with `qResistance`, `qInductanceD`, `qInductanceQ`, `qFlux`, `wValidMask` and `chPolePairs`; define `motor_position_t` as `const motor_position_ops_t *ptOps` plus `void *pContext`. Keep `motor.h` independent of AS5600, Hall, SMO, MCU and `foc_identify` headers.

- [ ] **Step 4: Run both numeric Motor contract tests.**

  Expected: the two context-binding tests and all invalid-config tests pass for float and Q16.15 without logging or warnings.

## Task 3: Move the hard-real-time lifecycle into Motor

**Files:**

- Modify: `foc/hal/foc_port.h`
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Add failing lifecycle tests.**

  Cover these exact cases: first high-frequency dispatch starts ADC calibration and does not run Core; calibration keeps PWM disabled; completion commits neutral duty before enabling PWM; Stop immediately calls emergency stop; sample/position/Core/duty failures call emergency stop before latching FAULT; ClearFault clears the fault only while PWM is disabled; a second context receives no calls intended for the first context.

- [ ] **Step 2: Run the lifecycle tests and verify RED.**

  Expected: failures identify absent `motor_HighFrequencyStep()`, `motor_Start()`, `motor_Stop()` or the missing lifecycle implementation, not malformed stubs.

- [ ] **Step 3: Implement the minimum lifecycle and one-slot command mailbox.**

  Use interrupt protection only around publishing/consuming `motor_command_sync_t.ePending`. The high-frequency switch must handle `INITIALIZING`, `IDLE`, `CALIBRATING`, `RUNNING` and `FAULT`, with a default branch that emergency-stops and latches a state fault. The running path must be exactly:

  ```c
  eResult = ptMotor->ptAdcOps->fnCurrentSample(
      ptMotor->pAdcContext, &ptMotor->tAdcCalibration, &tInput);
  if (eResult != FOC_RESULT_OK) {
      motor_EnterFault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
      return;
  }
  motor_PositionStep(ptMotor, &tInput);
  eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand, &tInput);
  if (eResult != FOC_RESULT_OK) {
      motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
      return;
  }
  eResult = ptMotor->ptPwmOps->fnDutyCommit(
      ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
  if (eResult != FOC_RESULT_OK) {
      motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
  }
  ```

  `motor_EnterFault()` must call `fnEmergencyStop(pPwmContext)` before changing lifecycle state. No `printf`, mdebug, I2C, `perfc_task_pt` or blocking call may appear in `motor_HighFrequencyStep()` or functions reachable only from it.

- [ ] **Step 4: Run the focused lifecycle tests, then the complete host suite.**

  Expected: all new lifecycle tests and all existing core/encoder/lifecycle tests pass for both numeric backends.

## Task 4: Shrink Core state without changing math

**Files:**

- Modify: `foc/middleware/foc_core.h`
- Modify: `foc/middleware/foc_core.c`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_foc_minimal_core.c`
- Modify: `tests/foc/test_motor.c`

- [ ] **Step 1: Add structural and behavior tests.**

  Assert that the Core consumes `foc_core_input_t.tElectricalAngle` and `qElectricalSpeed` for each call, that two consecutive calls with different input angles do not reuse an old angle, and that the resulting current/duty behavior matches the existing transform tests in float and fixed modes.

- [ ] **Step 2: Verify RED on the new input-ownership assertion.**

  Run `mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc minimal-core`; the new assertion must fail against the old persistent angle/speed ownership until the Core signature and implementation are changed.

- [ ] **Step 3: Remove only duplicate Core fields and update call sites.**

  Remove `tElectricalAngle`, `qElectricalSpeed`, `qIu`, `qIv` and `qIw` from `foc_core_state_t`; keep PI instances, αβ/dq intermediates, voltage and duty outputs. Keep all Clarke/Park/SVPWM equations unchanged. Pass current and position values in `foc_core_input_t` from Motor.

- [ ] **Step 4: Run core and Motor tests in both numeric modes.**

  Expected: the original transform vectors remain unchanged and the new structural test passes.

## Task 5: Adapt AS5600 and encoder position providers

**Files:**

- Modify: `foc/observer/foc_encoder.h`
- Modify: `foc/observer/foc_encoder.c`
- Modify: `peripheral/driver/as5600.h`
- Modify: `peripheral/driver/as5600.c`
- Modify: `foc/hal/foc_port.h`
- Modify: `peripheral/stm32g431/foc_port.c`
- Modify: `tests/foc/test_foc_minimal_encoder.c`
- Modify: `tests/foc/test_motor.c`

- [ ] **Step 1: Add failing provider-separation tests.**

  Use a fake I2C context that counts transactions. Call the provider's slow update and assert one I2C transaction and an updated cache; call `fnRead()` repeatedly and assert zero I2C transactions. Add a wrapped-angle test, an invalid-timeout test, direction inversion test, pole-pair electrical conversion test, and optional electrical-zero capture test.

- [ ] **Step 2: Verify RED for the new `motor_position_ops_t` adapter.**

  Expected: the provider test cannot compile or cannot bind until the AS5600 adapter and encoder conversion are implemented.

- [ ] **Step 3: Implement the adapter with the existing numerical behavior.**

  Keep the existing AS5600 sample cache and `foc_encoder_Step()` filter/extrapolation state in the provider context. `fnSlowUpdate()` performs the existing 1 kHz I2C read; `fnRead()` copies the latest cached result and returns the previous validity/timeout decision. Pole-pair multiplication, direction sign and electrical-zero addition move into the provider so `motor.c` consumes only `motor_position_feedback_t`.

- [ ] **Step 4: Run provider tests in float and fixed modes.**

  Expected: no provider test performs bus access from the fast read function, and existing encoder wrap/filter vectors remain unchanged.

## Task 6: Move the 1 kHz speed loop and calibration command into Motor

**Files:**

- Modify: `foc/motor/motor.c`
- Modify: `foc/motor/motor.h`
- Modify: `foc/app/foc_app.c`
- Modify: `foc/app/foc_app.h`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`

- [ ] **Step 1: Add failing Motor speed-loop and position-calibration tests.**

  Assert that `motor_ClockStep()` runs the speed PI only in SPEED mode, leaves fixed Iq unchanged in CURRENT mode, and uses the provider's electrical speed. Assert that a provider with `fnCaptureElectricalZero == NULL` returns `FOC_RESULT_DISABLED` without changing lifecycle or PWM state.

- [ ] **Step 2: Verify RED.**

  Expected: the tests fail because speed PI and encoder calibration are still owned by `foc_app_t`.

- [ ] **Step 3: Implement Motor clock/background operations.**

  `motor_ClockStep()` must run the existing speed PI gains and output limits without creating a second speed state. `motor_BackgroundStep()` calls the active provider's `fnSlowUpdate()` and services calibration state; it must be the only path allowed to invoke AS5600 I2C. The current phase exposes `motor_RequestAdcCalibration()` and `motor_CaptureElectricalZero()`; the App wrapper will later add the existing safe alignment sequence before invoking the optional provider capture callback.

- [ ] **Step 4: Run speed-loop and calibration tests in both numeric backends.**

  Expected: the new Motor speed-loop result matches the Task 1 App fixture, fixed-Iq remains fixed, and unsupported position calibration returns `FOC_RESULT_DISABLED`.

## Task 7: Compose Motor inside foc_app and preserve external API

**Files:**

- Modify: `foc/app/foc_app.h`
- Modify: `foc/app/foc_app.c`
- Modify: `peripheral/stm32g431/foc_port.c`
- Modify: `foc/foc.mk`
- Modify: `tests/foc/Makefile`
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`

- [ ] **Step 1: Add a compile-level App wrapper test.**

  Keep calls to `foc_app_Start`, `foc_app_Stop`, `foc_app_SetSpeedReference`, `foc_app_GetFeedback` and `foc_app_GetStatus` unchanged in the existing tests. Add an assertion that App status is a copied query result and not a second persistent Motor status object.

- [ ] **Step 2: Verify RED after temporarily wiring the App test to Motor symbols.**

  Expected: the wrapper test identifies each missing forwarding function or mismatched status field before the App implementation is changed.

- [ ] **Step 3: Replace duplicated App control state with one `motor_t` member.**

  Keep MODUS base, Shell registration, waveform handles, diagnostics counters and product command parsing in `foc_app_t`. Forward ISR, Clock, Start/Stop, references, feedback, status and encoder calibration to Motor. Move default physical parameters and control configuration into `motor_cfg_t`; do not copy the complete initialization config into `motor_t`.

  Update the STM32G431 port to bind `g_tFocAdcOps`, `g_tFocPwmOps` and the AS5600 provider with explicit contexts. The default single-board context may be `NULL`, but every callback signature must receive it.

- [ ] **Step 4: Update build source lists and run the full host suite.**

  Add `foc/motor/motor.c` to `FOC_SOURCES` and add `test_motor.c` to float/fixed test source lists. Expected: all existing tests and all Motor tests pass without compiling advanced observer files that are still outside the firmware source list.

## Task 8: Firmware build, static boundary checks and bench validation

**Files:**

- Modify: `foc/README.md`
- Modify: `foc/doc/foc-architecture.md`
- Modify: `docs/superpowers/specs/2026-09-08-foc-motor-object-refactor-design.md`

- [ ] **Step 1: Run source-boundary checks.**

  Run:

  ```powershell
  rg -n "as5600|I2C|mdebug|printf|perfc_task_pt|HAL_" D:\2_xundoc\project\modus_template\foc\motor\motor.c
  rg -n "tElectricalAngle|qElectricalSpeed|qIu|qIv|qIw" D:\2_xundoc\project\modus_template\foc\middleware\foc_core.h
  git -C D:\2_xundoc\project\modus_template diff --check
  ```

  Expected: the Motor implementation has no concrete provider/HAL/logging/bus dependency; Core no longer owns the removed duplicate input fields; `git diff --check` reports no whitespace errors.

- [ ] **Step 2: Run the complete host suite with the fixed toolchain.**

  ```powershell
  $env:Path = "D:\0_software\msys64\mingw64\bin;D:\0_software\msys64\usr\bin;$env:Path"
  mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc clean
  mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc minimal
  ```

  Expected: core, encoder, lifecycle and Motor float/fixed tests all report `PASS (0 failures)` with no compiler warnings.

- [ ] **Step 3: Build the STM32G431 firmware without flashing.**

  ```powershell
  $env:MAKE_EXE = 'D:\0_software\msys64\mingw64\bin\mingw32-make.exe'
  $env:Path = "D:\0_software\msys64\mingw64\bin;D:\0_software\msys64\usr\bin;$env:Path"
  .\make.bat BUILD=debug
  .\make.bat size BUILD=debug
  ```

  Expected: the firmware build exits zero, the Motor source is present in the link map, and no size regression exceeds the documented budget without review. Do not invoke `flash`, `auto` or any CPU-halt/debug action as part of this plan.

- [ ] **Step 4: Perform the hardware regression on the existing test setup.**

  Repeat the AS5600 `motor enc 0.05 100` procedure: at least four calibration offsets, Id reference 0.10, 5 s Speed waveform at 100 eHz, and Id/Iq waveform capture. Record raw values and compare against the baseline: offset range ≤0.0051 turn unless a mechanical setup change is documented; Id actual approximately 0.110 and Vd approximately 0.108; speed mean approximately 98.5 eHz, standard deviation approximately 4.9 eHz, range 95-105 eHz; 12 V operation at 100 eHz remains stable with BEMF approximately 2.1 V.

- [ ] **Step 5: Update the architecture/report documents with measured results.**

  State explicitly that 5-30 eHz quantization is an AS5600 1 kHz limitation, 50+ eHz is the smooth operating region, and 1 kHz waveform steps are normal. If any value regresses, stop at the failing boundary, add a reproducing test, and investigate the data flow before changing control gains.

## Completion gate

The migration is not complete until the source boundary checks, full host float/fixed suite, STM32G431 debug build and hardware regression all have fresh output. A successful host build alone does not prove that the AS5600 speed loop is preserved; the final report must separate automated evidence from bench evidence and must name any unavailable hardware measurement instead of inferring it.
