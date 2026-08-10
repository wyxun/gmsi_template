# 有感轻载 -> 无感观测验证 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使用 AS5600 编码器让电机在轻载、低电流、低发热条件下安全运行，并并行运行 SMO/NLFO 无感观测器，对比观测角度与编码器角度。

**Architecture:** 先建立“编码器直接闭环 + 可选无感观测器并行输出”的测试模式；控制角度始终来自编码器，无感观测器只写入 candidate 角度/速度，不参与控制。待观测器输出稳定后再评估是否切换为真正无感运行。

**Tech Stack:** STM32G431、AS5600、MODUS FOC、SMO、NLFO、MStudio Waveform。

---

## 安全与发热预算

当前电机配置：

```text
RatedVoltage = 12 V
RatedCurrent = 0.8 A
PhaseResistance = 2.5 Ω
PolePairs = 7
Vbus = 12 V
Idc measured ≈ 120 mA
```

输入功率：

```text
P_in = 12 V * 0.12 A = 1.44 W
```

如果 Iq 真是 `0.05 pu`：

```text
Iq = 0.05 * 0.8 A = 0.04 A
CopperLoss ≈ 3 * I^2 * R = 3 * 0.04^2 * 2.5 = 0.012 W
```

因此实测 `120 mA / 1.44 W` 远高于纯 40mA 电流闭环的理论损耗。计划第一步必须先确认实际 Iq、电流标定和母线电流来源，避免电机异常发热。

安全限制：

```text
Iq ref <= 0.05 pu
Vq ref <= 0.05 pu
Speed <= 2 Hz electrical
Bus current <= 300 mA
Phase current <= 0.2 A
Duty <= 10%
单次运行 <= 10 s
壳体温升 <= 10 °C
```

若任一超限，立即 `motor stop`。

---

## File Structure

- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor_hf_private.h`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_hf.c`
- Modify: `foc/app/foc_app.c`
- Modify: `foc/experimental/foc_verify.c`
- Test: `foc/tests/`（主机侧接口测试）

---

### Task 1: 电流标定与基线

**Files:**
- Verify: `foc/app/foc_app.c`

- [ ] **Step 1: 上电后执行 ADC 偏移校准**

```text
motor start
motor status
```

Expected：`Calib done=1`，电流接近零。

- [ ] **Step 2: 记录静止状态母线电流**

```text
motor status
```

Expected：记录电机未使能时的 Idle 电流，后续减去该基线。

- [ ] **Step 3: 使用最低 Iq 电流闭环试运行**

```text
motor_verify current 0.015 2
motor status
```

Expected：`Iq` 接近 `0.015 pu`，母线电流尽量接近静止基线。

- [ ] **Step 4: 若 Iq 与设定不一致，先校准 ADC 标定/电流增益**

检查 `foc/template/foc_hal_adapter.c` 的 `hal_adc_normalize` 和 ADC 基值。

---

### Task 2: 新增编码器电流闭环命令

**Files:**
- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 修改 `motor enc` 命令，默认进入编码器电流闭环**

将 `cmd_motor` 中 `enc` 分支的电压开环配置改为：

```c
} else if (strncmp(args, "enc", 3) == 0) {
    float fIq = 0.05f;
    float fSpeed = 2.0f;
    if (sscanf(args + 3, "%f %f", &fIq, &fSpeed) >= 1) {
        fIq = fIq > 0.10f ? 0.10f : fIq;
        fIq = fIq < -0.10f ? -0.10f : fIq;
    }
    s_tMotorRunConfig.eControlMode = MOTOR_CONTROL_CURRENT;
    s_tMotorRunConfig.ptInitialPositionSource = &s_tEncoderSourceIf;
    s_tMotorRunConfig.ptTargetPositionSource  = &s_tEncoderSourceIf;
    s_tMotorRunConfig.qInitialAngle = FOC_ZERO;
    s_tMotorRunConfig.qOpenLoopSpeed = FOC_SCALAR(fSpeed);
    s_tMotorRunConfig.qAcceleration = FOC_SCALAR(2.0f);
    s_tMotorRunConfig.tCurrentReference.qD = FOC_ZERO;
    s_tMotorRunConfig.tCurrentReference.qQ = FOC_SCALAR(fIq);
    foc_app_Start(&tFocApp);
}
```

- [ ] **Step 2: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected：编译通过。

---

### Task 3: 支持“并行观测器但不参与控制”

**Files:**
- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor_hf_private.h`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_hf.c`

- [ ] **Step 1: `motor_run_config_t` 增加观测源**

在 `ptTargetPositionSource` 后增加：

```c
const foc_position_source_if_t *ptObservationPositionSource;
```

- [ ] **Step 2: `motor_impl_t` 增加观测源状态**

在 `tPositionSource` 后增加：

```c
foc_position_source_if_t tObservationSource;
```

在位标志区增加：

```c
uint8_t bObservationSourceBound : 1;
```

- [ ] **Step 3: `motor_hf_plan_t` 增加观测源函数**

```c
void *pObservationSourceContext;
foc_result_t (*fnObservationStep)(
    void *, const foc_position_input_t *, foc_position_output_t *);
```

- [ ] **Step 4: `motor_fsm.c` 校验并保存观测源**

在 `validate_run()` 增加：

```c
if (run->ptObservationPositionSource != NULL &&
    !foc_position_source_IsValid(run->ptObservationPositionSource)) {
    return FOC_RESULT_INVALID_ARGUMENT;
}
```

在 `save_run()` 增加：

```c
impl->bObservationSourceBound =
    run->ptObservationPositionSource != NULL;
impl->tObservationSource = run->ptObservationPositionSource != NULL
    ? *run->ptObservationPositionSource
    : (foc_position_source_if_t){0};
impl->tHfPlan.pObservationSourceContext =
    run->ptObservationPositionSource != NULL
    ? run->ptObservationPositionSource->pSourceContext
    : NULL;
impl->tHfPlan.fnObservationStep =
    run->ptObservationPositionSource != NULL
    ? run->ptObservationPositionSource->fnStep
    : NULL;
```

- [ ] **Step 5: `motor_hf.c` 并行步进观测源**

在位置源步进之后增加：

```c
foc_position_output_t observation_output = {0};
if (plan->fnObservationStep != NULL) {
    result = plan->fnObservationStep(
        plan->pObservationSourceContext,
        &frame.tPositionInput,
        &observation_output);
    if (result != FOC_RESULT_OK ||
        observation_output.wFaults != 0U) {
        observation_output = (foc_position_output_t){0};
    }
}
```

在状态发布阶段，将 `tCandidateAngle` / `qCandidateSpeed` 指向观测输出：

```c
const foc_position_output_t *candidate =
    plan->fnObservationStep != NULL
    ? &observation_output
    : &frame.tPositionOutput;
impl->tCandidateAngle = candidate->tElectricalAngle;
impl->qCandidateSpeed = candidate->qElectricalSpeed;
impl->qAngleError = foc_angle_diff(
    candidate->tElectricalAngle,
    state->tElectricalAngle);
candidate_valid = plan->fnObservationStep != NULL
    ? (uint8_t)observation_output.eValidFlags
    : 0U;
```

- [ ] **Step 6: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected：编译通过，且运行时控制角度仍来自编码器，观测器失败不影响电机。

---

### Task 4: 应用层注册 SMO / NLFO 观测源

**Files:**
- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 增加 SMO/NLFO 实例与位置源接口**

```c
static foc_smo_t   s_tSmo;
static foc_nlfo_t  s_tNlfo;
static foc_position_source_if_t s_tSmoSource;
static foc_position_source_if_t s_tNlfoSource;
```

- [ ] **Step 2: 初始化 SMO**

```c
static void foc_app_InitSmo(void)
{
    foc_smo_params_t tParams = {
        .qModelGain       = FOC_SCALAR(0.25f),
        .qResistance      = FOC_SCALAR(0.1667f),
        .qSlidingGain     = FOC_SCALAR(0.10f),
        .qBoundaryInverse = FOC_SCALAR(10.0f),
        .qEmfFilterAlpha  = FOC_SCALAR(0.10f),
        .qMinimumBemf     = FOC_SCALAR(0.005f),
    };
    if (foc_smo_Init(&s_tSmo, &tParams) == FOC_RESULT_OK) {
        s_tSmoSource = foc_smo_PositionSourceInterface(&s_tSmo);
    }
}
```

- [ ] **Step 3: 初始化 NLFO**

```c
static void foc_app_InitNlfo(void)
{
    foc_nlfo_params_t tParams = {
        .qIntegratorGain    = FOC_SCALAR(0.01f),
        .qResistance        = FOC_SCALAR(0.1667f),
        .qAverageInductance = FOC_SCALAR(0.50f),
        .qFlux              = FOC_SCALAR(0.05f),
        .qCorrectionGain    = FOC_SCALAR(0.02f),
        .qMinimumFluxRatio  = FOC_SCALAR(0.05f),
    };
    if (foc_nlfo_Init(&s_tNlfo, &tParams) == FOC_RESULT_OK) {
        s_tNlfoSource = foc_nlfo_PositionSourceInterface(&s_tNlfo);
    }
}
```

- [ ] **Step 4: Shell 命令选择观测器**

在 `cmd_motor` 增加：

```c
} else if (strncmp(args, "obs", 3) == 0) {
    const char *p = args + 3;
    while (*p == ' ') p++;
    if (strncmp(p, "smo", 3) == 0) {
        s_tMotorRunConfig.ptObservationPositionSource = &s_tSmoSource;
        MLOG(I, "Observer: SMO\r\n");
    } else if (strncmp(p, "nlfo", 4) == 0) {
        s_tMotorRunConfig.ptObservationPositionSource = &s_tNlfoSource;
        MLOG(I, "Observer: NLFO\r\n");
    } else {
        s_tMotorRunConfig.ptObservationPositionSource = NULL;
        MLOG(I, "Observer: disabled\r\n");
    }
}
```

- [ ] **Step 5: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected：编译通过。

---

### Task 5: 有感轻载运行

**Files:**
- Verify: 硬件

- [ ] **Step 1: 编译烧录并启动 RTT**

Run: `.\make.bat auto`

- [ ] **Step 2: 选择 SMO 观测，编码器电流闭环运行**

```text
motor obs smo
motor enc 0.05 2
motor status
```

Expected：

```text
Iq ≈ 0.05 pu
Bus current 越低越好，不应显著超过 300 mA
Active angle 来自编码器
Candidate angle 来自 SMO
```

- [ ] **Step 3: 使用 MStudio 记录**

开启波形：

```text
TestSin Only 可先关闭
Show All
```

观察：

- `ActiveAngle` / `CandidateAngle`
- `ActiveSpeed` / `CandidateSpeed`
- `Iq`

- [ ] **Step 4: 若母线电流或发热超限，立即停止并回退 Iq**

```text
motor stop
motor_verify current 0.02 2
```

---

### Task 6: SMO 对比与调参

**Files:**
- Verify: `foc/app/foc_app.c`

- [ ] **Step 1: 以 2 Hz electrical 运行并记录误差**

Expected：

- `CandidateAngle` 与编码器 `ActiveAngle` 的误差随时间收敛。
- 误差稳定后不超过 0.1 turn（36° 电角度）。

- [ ] **Step 2: 若误差大，依次调节**

```text
qSlidingGain     0.10 -> 0.20 -> 0.30
qBoundaryInverse 10 -> 20
qEmfFilterAlpha  0.10 -> 0.05
```

- [ ] **Step 3: 每次修改后重新编译、烧录、重复 Task 5**

---

### Task 7: NLFO 对比

**Files:**
- Verify: `foc/app/foc_app.c`

- [ ] **Step 1: 切换到 NLFO 观测**

```text
motor obs nlfo
motor enc 0.05 2
```

- [ ] **Step 2: 调节 NLFO 参数**

优先调：

```text
qFlux             0.05 -> 0.03 -> 0.08
qAverageInductance 0.50 -> 0.30 -> 0.80
qIntegratorGain    0.01 -> 0.005 -> 0.02
```

- [ ] **Step 3: 与 SMO 记录同一转速/电流下的误差**

Expected：记录两组误差表，用于决定后续真正无感切换优先使用哪个观测器。

---

### Task 8: 评估真正无感切换

**Files:**
- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 只有当 SMO/NLFO 误差稳定后再考虑切换**

```text
motor start
```

当前默认仍为开环，不在本计划第一阶段切换。

- [ ] **Step 2: 若决定切换，将 run config 改为**

```c
.ptInitialPositionSource = NULL,
.ptTargetPositionSource = &s_tSmoSource,
.qOpenLoopSpeed = FOC_SCALAR(2.0f),
.qAcceleration = FOC_SCALAR(2.0f),
```

- [ ] **Step 3: 切换后再次检查角度误差、电流冲击和母线电流**

---

## Self-Review

覆盖：

- 有感轻载闭环：Task 2、Task 5。
- 无感候选观测：Task 3、Task 4。
- SMO/NLFO 对比：Task 6、Task 7。
- 发热/电流反推：安全预算章节与 Task 1。
- 真正无感切换：Task 8。

无占位符。所有接口名称在任务间保持一致：`ptObservationPositionSource`、`fnObservationStep`、`tCandidateAngle`。
