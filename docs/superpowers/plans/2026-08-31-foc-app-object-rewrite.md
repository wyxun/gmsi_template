# FOC App Object 化重写 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `foc_app` 重写为符合 MODUS Class 规范的单电机对象，同时保留已验证的 FOC 电流环链路和现有外设初始化。

**Architecture:** 通用算法继续由 `foc/` 提供，AS5600 继续由 `peripheral/driver` 提供，目标硬件继续由 `peripheral/<chip>` 提供。`foc_app_t` 拥有所有可变 App 状态，`foc_app_cfg_t` 注入配置和 MDI 硬件依赖；本次不新增 Mdriver。

**Tech Stack:** C11、MODUS、perfc-PT、perf_counter、FOC float/fixed 双数值后端、host fake port、STM32G431 debug 构建。

---

## 文件边界

修改以下文件：

- `foc/app/foc_app.h`：公开完整的配置、对象、直接组合类型和对象 API。
- `foc/app/foc_app.c`：实现对象初始化、前台 PT、实时 ISR、Shell 和诊断。
- `tests/foc/test_foc_minimal_lifecycle.c`：改用显式对象/配置并覆盖对象隔离。
- `tests/foc/Makefile`：补充对象化 App 测试编译开关和入口。
- `foc/README.md`：更新对象所有权、初始化和调度说明。
- `foc/doc/foc-architecture.md`：更新 App/Core/port 的依赖图。

不修改以下文件：

- `peripheral/stm32g431/haladc.c`；
- `peripheral/stm32g431/haltim1.c`；
- `peripheral/stm32g431/foc_port.c`；
- `target/stm32g431/stm32g4xx_it.c`；
- `foc/middleware`、`foc/control`、`foc/observer` 中已经验证的算法实现。

不创建 `Mdriver` 文件。当前没有一个同时满足独立职责、多个 API、明确复用价值、可独立配置和可独立测试的 App 子功能。

除非用户另行要求，本计划不包含 Git commit、分支切换或推送。

## Task 1: 先建立对象 API 的失败测试

**Files:**

- Modify: `tests/foc/test_foc_minimal_lifecycle.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: 将 host 测试改成显式对象和配置调用**

在测试文件中加入两个独立对象、RingBuffer 和配置构造器。测试配置必须提供所有初始化依赖，不能再调用 `foc_app_Init(0U, 0U)`：

```c
static uint8_t s_chRingA[64];
static uint8_t s_chRingB[64];
static foc_app_t s_tAppA;
static foc_app_t s_tAppB;

static foc_app_cfg_t test_make_config(uint8_t *pchRing)
{
    foc_app_cfg_t tConfig = {
        .pchRingBuffer = pchRing,
        .hwRingSize = 64U,
        .ptHardware = &HW,
        .tCurrentPiParams = {
            .tKp = {0, FOC_SCALAR(0.20f)},
            .tKiTs = {0, FOC_SCALAR(0.005f)},
            .tKdOverTs = {0, FOC_ZERO},
            .qOutputMinimum = FOC_SCALAR(-0.55f),
            .qOutputMaximum = FOC_SCALAR(0.55f),
            .qIntegratorMinimum = FOC_SCALAR(-0.50f),
            .qIntegratorMaximum = FOC_SCALAR(0.50f),
        },
        .tSpeedPiParams = {
            .tKp = {0, FOC_SCALAR(0.20f)},
            .tKiTs = {0, FOC_SCALAR(0.005f)},
            .tKdOverTs = {0, FOC_ZERO},
            .qOutputMinimum = FOC_SCALAR(-0.10f),
            .qOutputMaximum = FOC_SCALAR(0.10f),
            .qIntegratorMinimum = FOC_SCALAR(-0.10f),
            .qIntegratorMaximum = FOC_SCALAR(0.10f),
        },
        .tEncoderParams = {
            .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
            .hwInvalidTimeout = 100U,
            .chPolePairs = 7U,
            .qHighFrequencyPeriod = FOC_SCALAR(0.00005f),
        },
        .tElectricalZero = {0U},
        .bEncoderEnabled = false,
        .bDirectionInverted = false,
    };

    return tConfig;
}
```

所有生命周期测试改为使用 `(uintptr_t)&s_tAppA` 和配置地址，所有 App API 调用增加对象指针，例如：

```c
foc_app_Start(&s_tAppA, &tCommand);
foc_app_Stop(&s_tAppA);
foc_app_GetStatus(&s_tAppA, &tStatus);
foc_app_HighFrequencyStep(&s_tAppA);
```

- [ ] **Step 2: 增加初始化契约和对象隔离测试**

加入空对象、空配置、零极对数和两对象命令隔离测试。两对象测试只验证 App 状态不互相覆盖，不同时运行当前单实例 port：

```c
static int test_init_rejects_null_and_invalid_config(void)
{
    foc_app_cfg_t tConfig = test_make_config(s_chRingA);
    int nFailures = 0;

    if (foc_app_Init(0U, (uintptr_t)&tConfig) == FOC_RESULT_OK) {
        nFailures++;
    }
    if (foc_app_Init((uintptr_t)&s_tAppA, 0U) == FOC_RESULT_OK) {
        nFailures++;
    }
    tConfig.tEncoderParams.chPolePairs = 0U;
    if (foc_app_Init((uintptr_t)&s_tAppA,
                     (uintptr_t)&tConfig) == FOC_RESULT_OK) {
        nFailures++;
    }
    return nFailures;
}
```

- [ ] **Step 3: 运行失败测试并记录基线**

运行：

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc minimal-lifecycle-float
```

预期：因新对象类型和对象 API 尚未实现而编译或链接失败；不得修改测试来绕过失败。

## Task 2: 在头文件中完成 Class 类型和 API

**Files:**

- Modify: `foc/app/foc_app.h`

- [ ] **Step 1: 声明依赖和 App 专属轻量分组**

在头文件中包含 `modus.h`、`mdi_hw.h`、`foc_core.h`、`foc_encoder.h`、`foc_pid.h`，目标支持 AS5600 时再包含 `as5600.h`。声明 `foc_app_angle_source_e`、`foc_app_lifecycle_t`、`foc_app_position_t` 和 `foc_app_diagnostics_t`。

这些分组只用于提高内聚性，不是独立 MODUS Class，不创建 Mdriver。诊断结构必须包含 ISR 计时、轮询时间、PT 游标、float 显示值和 Waveform 通道 ID；不得再由 `.c` 的静态变量保存。

- [ ] **Step 2: 声明完整配置和对象**

使用以下字段，不保留 `chReserved`：

```c
typedef struct {
    uint8_t *pchRingBuffer;
    uint16_t hwRingSize;
    const mdi_hardware_t *ptHardware;
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    foc_encoder_params_t tEncoderParams;
    foc_angle_t tElectricalZero;
    bool bEncoderEnabled;
    bool bDirectionInverted;
} foc_app_cfg_t;

typedef struct {
    modus_base_t *ptBase;
    const mdi_hardware_t *ptHardware;
    foc_core_state_t tCore;
    foc_pid_t tSpeedPid;
    foc_encoder_t tEncoder;
#if defined(MDI_HW_HAS_I2C_ENCODER)
    as5600_t tAs5600;
#endif
    foc_adc_calib_t tCalibration;
    foc_core_command_t tCommand;
    foc_app_lifecycle_t tLifecycle;
    foc_app_position_t tPosition;
    foc_app_diagnostics_t tDiagnostics;
} foc_app_t;
```

`ptHardware` 是显式 MDI 依赖。STM32G431 默认配置使用 `&HW`，AS5600 使用 `ptHardware->ptI2c1`；无编码器目标关闭 `bEncoderEnabled`，不访问不存在的成员。

- [ ] **Step 3: 更新对象 API 原型**

所有读写运行状态的 API 都接收对象指针：

```c
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);
void foc_app_HighFrequencyISR(void);
void foc_app_HighFrequencyStep(foc_app_t *ptThis);
foc_result_t foc_app_Start(foc_app_t *ptThis,
                           const foc_core_command_t *ptCommand);
void foc_app_Stop(foc_app_t *ptThis);
foc_result_t foc_app_ClearFault(foc_app_t *ptThis);
foc_result_t foc_app_SetVoltageReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ);
foc_result_t foc_app_SetCurrentReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ);
foc_result_t foc_app_SetSpeedReference(
    foc_app_t *ptThis, foc_scalar_t qMechanicalTurnPerSecond);
foc_result_t foc_app_GetStatus(const foc_app_t *ptThis,
                               foc_status_t *ptStatus);
```

`foc_app_HighFrequencyISR()` 保留为现有 ADC 向量使用的单实例包装器，内部转发至 `foc_app_HighFrequencyStep(&tFocApp)`。

- [ ] **Step 4: 编译确认类型错误边界**

运行：

```powershell
mingw32-make -C tests/foc minimal-lifecycle-float
```

预期：实现函数体尚未迁移时可以链接失败，但不再因为缺少 `foc_app_t`、`foc_app_cfg_t` 或对象参数而编译失败。

## Task 3: 按模板契约重写初始化

**Files:**

- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 清除 App 文件中的隐藏 runtime**

删除 `s_tFoc`、ISR 统计静态变量、Waveform 通道静态变量、AS5600 静态实例和所有函数内运行时 `static` 变量。保留 MODUS 基础对象、基础配置、只读默认值和 `MODUS_DECLARE_OBJECT` 生成的 `tFocApp`。

- [ ] **Step 2: 创建默认配置**

使用 `.ptHardware = &HW`、RingBuffer、当前 PI/速度 PI 参数、编码器参数、零位和方向配置调用 `MODUS_DECLARE_OBJECT(foc_app, FocApp, ...)`。STM32G431 的 `bEncoderEnabled` 为 `true`，无 I2C 编码器目标为 `false`。

- [ ] **Step 3: 按固定顺序实现 `foc_app_Init()`**

实现顺序必须是：

```text
检查对象和配置地址
→ 检查 RingBuffer、硬件、PID 限幅和编码器参数
→ 清零对象并置 IDLE/PWM 关闭
→ 绑定 ptBase、wParent、ptHardware 和配置
→ 初始化 Core、电流 PI、速度 PI、编码器和 AS5600
→ 初始化 PT、定时器和诊断成员
→ 调用 foc_port_Init()，随后执行 foc_port_EmergencyStop()
→ 初始化 Waveform
→ 最后调用并检查 mbase_Init()
```

错误路径必须保持 PWM 关闭；MODUS 有效路径不得使用 `(void)wObjectAddr`、`(void)wObjectCfgAddr` 或 `(void)ptCfg`。

- [ ] **Step 4: 运行生命周期测试**

运行：

```powershell
mingw32-make -C tests/foc minimal-lifecycle-float
```

预期：`minimal lifecycle: PASS (0 failures)`。

## Task 4: 迁移命令、角度、校准和实时状态

**Files:**

- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 为私有动作增加对象上下文**

`foc_app_EnterFault`、`foc_app_ConsumeCommand`、`foc_app_CalibrationStep`、`foc_app_AngleStep`、`foc_app_CurrentStartupStep` 和 `foc_app_RunningStep` 全部接收 `foc_app_t *ptThis`，只读写对象成员。

- [ ] **Step 2: 保持已验证的 ISR 链路**

`foc_app_HighFrequencyStep(ptThis)` 只执行：

```text
消费命令
→ CALIBRATING 时做 ADC 校准
→ RUNNING 时 CurrentSample
→ 角度/编码器观测
→ foc_core_step（Clarke/Park/电流 PI/IPark/SVPWM）
→ DutyCommit
```

ISR 不执行日志、I2C、PT 或阻塞等待。所有错误先急停，再将对象状态置为 `FOC_STATE_FAULT`。

- [ ] **Step 3: 让 Shell 只调用对象 API**

Shell 通过 `foc_app_t *ptThis = &tFocApp` 调用 Start、Stop、引用 setter、状态读取和开环配置操作，不直接写命令、角度来源、故障位或 PID 积分。需要配置开环角度时提供完整的 App 操作 API，由 API 校验 IDLE 和更新对象成员。

- [ ] **Step 4: 将编码器轮询和校准改成非阻塞 PT**

`cmd_encoder cal` 只提交校准请求，不再调用 `delay_ms(1500)`。`foc_app_Run()` 使用 `ptThis->tDiagnostics.chRunPt` 和对象计时字段推进 AS5600 更新、轮询退避和校准状态；`foc_app_Clock()` 只从 `wObjectAddr` 获取对象并执行 1 kHz 速度 PI 或设置事件，禁止在 SysTick 中执行 I2C。

- [ ] **Step 5: 运行 float/fixed 生命周期测试**

运行：

```powershell
mingw32-make -C tests/foc minimal-lifecycle-float
mingw32-make -C tests/foc minimal-lifecycle-fixed
```

预期：两套都输出 `minimal lifecycle: PASS (0 failures)`。

## Task 5: 迁移诊断和 Waveform

**Files:**

- Modify: `foc/app/foc_app.c`

- [ ] **Step 1: 使用对象诊断成员记录 ISR 时间**

将 last/max/total/sample、上次报告时间和计数饱和逻辑迁移到 `ptThis->tDiagnostics`。统计快照和 RTT 输出在 `Run()` 中完成，ISR 只记录数值。

- [ ] **Step 2: 将 Waveform 绑定到对象字段**

`foc_app_WaveformInit(foc_app_t *ptThis)` 直接注册 `ptThis->tCore.qIu`、`ptThis->tCore.tCurrent.qD/qQ`、角度/速度、Vd/Vq 和 `ptThis->tDiagnostics.fEncoderMechanicalTurns`。AddVariable、SetStreamRate、SetChannelRate 和 Start 的返回值按现有 API 检查；通道 ID 写入对象成员。

- [ ] **Step 3: 运行全套 host 测试**

运行：

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc minimal
```

预期：core、encoder、lifecycle 的 float/fixed 测试全部通过。

## Task 6: 固件构建和文档同步

**Files:**

- Modify: `foc/README.md`
- Modify: `foc/doc/foc-architecture.md`
- Verify: `foc/foc.mk`

- [ ] **Step 1: 确认构建边界**

确认 `foc/foc.mk` 只链接现有算法和 `foc/app/foc_app.c`，不加入 Mdriver 或旧 `foc/motor` 源文件。

- [ ] **Step 2: 更新文档**

删除旧的“`foc_app.c` 持有 `static foc_runtime_t`”描述，改为 `MODUS_DECLARE_OBJECT` 生成唯一 `foc_app_t tFocApp`；写明 `foc_app_cfg_t` 注入 RingBuffer、MDI 硬件和算法参数，并明确当前没有 Mdriver、ADC/PWM/TIM1/CH4 初始化不在本次改动范围。

- [ ] **Step 3: 构建 STM32G431 debug-rel float**

运行：

```powershell
.\make.bat clean
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel
```

预期：固件成功生成，现有 `ADC1_2_IRQHandler` 仍调用 `foc_app_HighFrequencyISR()`。

- [ ] **Step 4: 构建 fixed 后端**

运行：

```powershell
.\make.bat clean
.\make.bat TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
```

预期：fixed 固件成功生成，float Waveform 条件代码被排除，外设初始化文件无改动。

- [ ] **Step 5: 做源码静态检查**

运行：

```powershell
rg -n "static .*foc_runtime|static .*as5600|static .*Isr|static .*Waveform|static uint32_t s_wLast|\(void\)wObjectAddr|\(void\)wObjectCfgAddr|chReserved" foc/app/foc_app.c foc/app/foc_app.h
git diff --check
```

预期：App 源码不再有隐藏 runtime、函数内运行时静态变量、占位配置或忽略对象地址；只允许基础对象、基础配置和只读对象留在文件静态区。

## Task 7: 硬件验证前的安全检查

**Files:**

- No source changes unless a preceding build exposes a real integration error.

- [ ] **Step 1: 无母线完成固件启动检查**

保持母线关闭，仅使用 MCU 控制板 USB/SWD 供电，执行 `reset/run`、等待 `mshell ready`、`motor status` 和 `encoder`。验收：无 HardFault、无 ISR 卡死、IDLE 时 PWM 关闭；无母线 ADC 值不作为电流环证据。

- [ ] **Step 2: 用户确认母线后执行最小风险指令**

依次执行：

```text
motor lock 0
motor stop
encoder cal
motor current 0.05 0
motor stop
motor enc 0.10 2
motor stop
```

每条指令之间读取 `motor status` 和 ISR timing，记录 state、faults、PWM、Id/Iq、Vd/Vq、编码器机械角度和 ISR 时间。出现 fault 时先停机并保存日志。

- [ ] **Step 3: 只根据证据判断闭环**

只有在 ISR 统计持续有样本、current 模式的 Iq 未被速度环覆盖、Id/Iq 和编码器 Waveform 持续更新、Stop 后 PWM 关闭、speed 模式反馈来自编码器时，才判定本轮重写的电流环和速度环通过。

