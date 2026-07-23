# Modus FOC — 电机控制库

该目录提供一个与 MCU 架构无关、支持多电机实例的 PMSM/BLDC 矢量控制库。核心算法同时支持浮点和 Q16.15 定点运算。电机对象是不透明的静态句柄：应用只通过 `motor.h` 的命令式公共 API 操作电机，不能访问其私有运行状态、控制器、采样值、HAL 或校准数据。

本文以"把库移植到一个新平台并把一个电机实例运行起来"为主线：

- 第 4 章给出两条接口边界总览和最小移植步骤；
- 第 5 章定义硬件层接口（平台必须实现的输入侧）；
- 第 6 章定义应用层接口（库提供的输出侧）；
- 第 7 章给出四种运行场景的完整用法示例；
- 第 8 章说明统一位置源适配。

本文中的数值仅用于说明 API，不能直接作为未知电机的上电参数。

## 1. 能力与边界

当前库包含：

- 不透明多实例电机对象：静态分配、无动态内存、内部 `fsm_rt_t` 生命周期。
- 命令式公共 API：Start/Stop/引用设定/快照/事件环，状态迁移由内部 FSM 决定。
- Clarke、Park、反 Park 坐标变换。
- SVPWM、SPWM、三次谐波注入 SPWM。
- 电压开环、电流、速度和位置级联控制。
- PID、LADRC、SMC、STA、PLL、LTD、DOB、滤波和 PMSM 解耦前馈。
- 统一位置源接口：Hall、SMO、NLFO、HFI，以及 motor 内部的资格判定与开环到闭环平滑接管。
- MTPA、弱磁、死区、相位延迟和齿槽转矩补偿。
- 默认关闭的 NSD 转子极性检测、电机参数辨识和硬件诊断输出。
- 浮点与 Q16.15 定点后端。
- 固定容量事件环与一致性快照。

库不会替产品完成以下工作：

- 配置 PWM 定时器、互补输出、死区、刹车输入和 ADC 触发点。
- 决定电流采样电阻、运放增益、ADC 偏置和物理量归一化比例。
- 提供平台临界区和毫秒时钟；二者由产品通过 `motor_sync_if_t` 与 `motor_time_if_t` 绑定。
- 把 `motor_HighFrequencyStep()` / `motor_LowFrequencyStep()` 挂到真实中断和调度器。
- 自动整定电机参数或保护阈值。
- 替代硬件过流、过压和功率级急停保护。

主机测试和交叉编译只能证明接口与数值路径可构建，不能替代真实电机的限流上电和波形验证。

## 2. 目录和依赖关系

```text
foc/
├── math/          数值后端、归一化角度、三角函数和平方根
├── middleware/    Clarke/Park 变换
├── modulation/    SVPWM、SPWM、三次谐波调制
├── control/       控制器、滤波、PLL、DOB 和前馈
├── observer/      Hall、SMO、NLFO、HFI 和位置源适配器
├── optimization/  MTPA、弱磁、死区、相位和齿槽补偿
├── experimental/  NSD 和参数辨识安全状态机
├── diagnostic/    硬件诊断输出（默认不进构建）
├── hal/           平台无关的 PWM/ADC 接口
├── motor/         电机对象、生命周期 FSM、位置源管理、高低频控制
├── app/           当前项目的应用示例
├── doc/           内部架构文档
├── foc_config.h   编译期配置
└── foc.h          对外统一头文件
```

推荐依赖方向：

```text
产品应用 -> motor（公共 API）/ observer / optimization
         -> control / modulation / middleware
         -> math

产品应用 -> FOC HAL -> peripheral/<chip>/ MDI 适配 -> 芯片驱动
```

业务代码不应包含厂商 HAL 头文件。芯片相关代码放在 `peripheral/<chip>/` 或 `target/<chip>/`。

## 3. 数值、单位和角度约定

### 3.1 选择数值后端

每次构建必须且只能选择一种后端：

```c
#define FOC_NUMERIC_FLOAT 1
/* 或 */
#define FOC_NUMERIC_FIXED 1
```

```powershell
.\make.bat TARGET_CHIP=at32f413 FOC_NUMERIC=float
.\make.bat TARGET_CHIP=at32f413 FOC_NUMERIC=fixed
```

在代码中统一使用 `foc_scalar_t`（`q_type` 是其兼容别名）。

### 3.2 Q16.15

定点后端使用带符号 32 位存储、15 位小数：`FOC_ONE` 表示 1.0，`FOC_HALF` 表示 0.5。高频中断中应使用预先计算的 `foc_scalar_t` 和 `foc_gain_t`，避免反复进行浮点转换。

### 3.3 归一化量

- 相电流、D/Q 电流：额定或允许峰值电流为 1 pu。
- D/Q 电压：可用母线电压或最大调制电压为 1 pu。
- 占空比：范围 `[0, 1]`。
- 速度引用：机械 turn/s；开环角速度：电角 turn/s。

### 3.4 角度

`foc_angle_t` 内部使用 BAM32（二进制角度，32-bit 无符号满量程）存储，使用"圈"作为公共单位：

```c
foc_angle_t tAngle = foc_angle_from_turns(0.25f); /* 90° */
foc_scalar_t qSine = foc_angle_sin(tAngle);
foc_scalar_t qDelta = foc_angle_diff(tTarget, tActual);
/* wrap 由 uint32_t 自然溢出实现，零开销 */
```

0.0 表示 0°，0.25 表示 90°，0.5 表示 180°，1.0 会环绕到 0.0。

### 3.5 增益

`foc_gain_t` 把整数和小数部分分开，推荐只通过构造函数创建：

```c
foc_gain_t tGain;
if (foc_gain_from_float(2.5f, &tGain) != FOC_RESULT_OK) {
    /* 参数错误，禁止启动电机。 */
}
```

## 4. 移植总览：两条接口边界

FOC 库与外部世界只有两条边界。把库搬到新平台，只需要实现输入侧绑定，然后按输出侧 API 驱动电机。

### 4.1 两侧接口一览

输入侧（平台必须实现，motor 库消费）：

| 绑定 | 类型 | 必需性 |
|---|---|---|
| PWM 输出 | `foc_pwm_if_t` | 三个回调全部必需 |
| 电流采样 | `foc_adc_if_t` | `fnReconstruct` 必需；`fnOffsetCalib` 校准必需 |
| 临界区 | `motor_sync_if_t` | 实时步骤在 ISR 中运行时必需成对绑定 |
| 毫秒时钟 | `motor_time_if_t` | 启动延时或开环到闭环切换时必需 |
| 位置源硬件 | 如 `foc_hall_read_code_fn_t` | 有传感器方案必需 |

输出侧（库提供，应用调用）：

| 分组 | API |
|---|---|
| 生命周期 | `motor_Init` / `motor_Reset` |
| 命令 | `motor_Start` / `motor_Stop` / `motor_ClearFault` / `motor_EmergencyStop` |
| 流程驱动 | `motor_RunFSM`（主循环） |
| 实时步骤 | `motor_HighFrequencyStep`（PWM/ADC ISR） / `motor_LowFrequencyStep`（ms 级调度） |
| 引用设定 | `motor_SetVoltageReference` / `motor_SetCurrentReference` / `motor_SetSpeedReference` / `motor_SetPositionReference` |
| 诊断 | `motor_GetSnapshot` / `motor_DebugReadEvent` / `motor_GetRawCurrent` |

应用只需一个头文件：

```c
#include "foc/foc.h"
```

### 4.2 最小移植步骤

1. 实现 `foc_pwm_if_t` 三个回调（经 MDI 访问定时器，不包含厂商头）。
2. 实现 `foc_adc_if_t`，重点是 `fnReconstruct` 的采样、零偏、归一化和拓扑重构。
3. 绑定 `motor_sync_if_t`：推荐 perf_counter 多架构全局中断守卫。
4. 绑定 `motor_time_if_t`：包装系统毫秒时钟。
5. 填写 `motor_config_t` 并调用 `motor_Init()`，检查返回值。
6. 把 `motor_HighFrequencyStep()` 挂到 PWM/ADC 控制中断，频率与 `qHighFrequencyPeriod` 一致。
7. 把 `motor_LowFrequencyStep()` 挂到 ms 级调度，主循环每轮调用 `motor_RunFSM()`。
8. 需要传感器或观测器时，实现位置源并在 `motor_run_config_t` 中绑定。
9. 用 `motor_GetSnapshot()` 确认 IDLE → STARTING → RUNNING 迁移与故障位行为后，再按第 15 章顺序小信号上电。

完整真实范例见 `foc/app/foc_app.c` 与 `peripheral/at32f413/foc_hal_mdi_adapter.c`。

## 5. 硬件层接口（输入侧：平台必须实现）

所有回调都接收自己的 `pContext`，禁止依赖全局"当前电机"。所有回调必须有界、非阻塞。

### 5.1 PWM 接口 `foc_pwm_if_t`

```c
typedef struct {
    void *pContext;
    foc_result_t (*fnSetDuty)(void *pContext,
                              q_type qDutyU, q_type qDutyV, q_type qDutyW);
    foc_result_t (*fnEnable)(void *pContext, bool bEnable);
    void (*fnEmergencyStop)(void *pContext);
} foc_pwm_if_t;
```

- `fnSetDuty`：输入 `[0, 1]` 的三相占空比，由高频控制中断调用。要求三相同步更新、非阻塞。
- `fnEnable`：`false` 用于正常停机。由 motor FSM 在主循环临界区内调用，实现必须是有界寄存器写。
- `fnEmergencyStop`：在任意上下文调用，必须立即关闭功率输出。

### 5.2 电流采样接口 `foc_adc_if_t`

```c
typedef struct {
    void *pContext;
    foc_result_t (*fnStartConversion)(void *pContext);
    foc_result_t (*fnOffsetCalib)(void *pContext, foc_adc_calib_t *ptCalib);
    foc_result_t (*fnGetRaw)(void *pContext, uint32_t *pwRawU,
                             uint32_t *pwRawV, uint32_t *pwRawW);
    foc_result_t (*fnReconstruct)(void *pContext,
                                  phase_current_handle_t *ptHandle);
} foc_adc_if_t;
```

- `fnReconstruct`（必需）：由高频控制中断调用。负责读取 ADC/DMA 结果、减去零偏、按采样电阻和运放增益归一化、按拓扑重构三相电流。
- `fnOffsetCalib`（启动校准必需）：在 PWM 关闭时执行。实现把零偏写入 `ptCalib` 并置 `bIsCalibrated`。

### 5.3 临界区接口 `motor_sync_if_t`

```c
typedef struct {
    void *pContext;
    uintptr_t (*fnEnter)(void *pContext);
    void (*fnExit)(void *pContext, uintptr_t wState);
} motor_sync_if_t;
```

推荐使用 perf_counter 多架构全局中断守卫：

```c
static uintptr_t port_EnterCritical(void *pContext)
{
    (void)pContext;
    return (uintptr_t)perfc_port_disable_global_interrupt();
}
static void port_ExitCritical(void *pContext, uintptr_t wState)
{
    (void)pContext;
    perfc_port_resume_global_interrupt((uint32_t)wState);
}
```

### 5.4 毫秒时钟接口 `motor_time_if_t`

```c
typedef struct {
    void *pContext;
    uint32_t (*fnGetMilliseconds)(void *pContext);
} motor_time_if_t;
```

### 5.5 调用上下文与频率总表

| 调用点 | 函数 | 频率 |
|---|---|---|
| PWM/ADC 控制中断 | `motor_HighFrequencyStep()` | PWM 载波频率（如 20 kHz） |
| ms 级系统调度 | `motor_LowFrequencyStep()` | 1 kHz 级 |
| 主循环 | `motor_RunFSM()` | 每轮主循环，线程上下文 |
| 主循环按需 | `motor_GetSnapshot()` 等诊断/命令 API | 按需，线程上下文 |

真实调度频率必须与 `motor_config_t.qHighFrequencyPeriod` / `qLowFrequencyPeriod` 一致。

### 5.6 参考适配器

- `peripheral/at32f413/foc_hal_mdi_adapter.c`
- `peripheral/stm32g431/foc_hal_mdi_adapter.c`

它们通过 MDI 访问硬件并提供 `foc_hal_mdi_Bind()` / `foc_hal_mdi_BindDefault()` 一键绑定。

## 6. 应用层接口（输出侧：库提供）

### 6.1 生命周期与命令

```c
foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig);
void motor_Reset(motor_handle_t *ptMotor);
foc_result_t motor_Start(motor_handle_t *ptMotor,
                         const motor_run_config_t *ptRunConfig);
foc_result_t motor_Stop(motor_handle_t *ptMotor);
fsm_rt_t motor_RunFSM(motor_handle_t *ptMotor);
foc_result_t motor_ClearFault(motor_handle_t *ptMotor);
void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_e eFault);
```

- `motor_Init()`：拷贝配置并复位对象，成功后状态为 IDLE。校验极对数、采样拓扑、周期、方向、门限和接口绑定。
- `motor_Start()`：校验启动配置后把 START 命令放入内部邮箱并立即返回；状态迁移由 `motor_RunFSM()` 执行。`FOC_RESULT_OK` 表示命令已接受；`FOC_RESULT_BUSY` 表示有挂起命令；`FOC_RESULT_INVALID_ARGUMENT` 表示配置非法或不在 IDLE。
- `motor_Stop()`：STARTING/RUNNING 时接受，STOPPING 期间先关 PWM 再回 IDLE。
- `motor_RunFSM()`：主循环每轮调用，负责校准、启动延时、开环拖动、源资格判定、角度融合、停止和故障流程。
- `motor_EmergencyStop()`：唯一绕过 FSM、立即关闭功率输出的公共接口，ISR 安全。

### 6.2 实时步骤

```c
foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor);
foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor);
```

`motor_HighFrequencyStep()` 必须在 PWM/ADC 控制中断中以真实载波频率调用，内部固定数据流：

```text
接管引用值 -> 电流采样 -> 更新位置源 -> 选择/融合控制角度
-> Clarke/Park -> 电流控制 -> 反 Park -> 调制 -> PWM 输出 -> 更新诊断
```

`motor_LowFrequencyStep()` 负责位置环生成速度引用、速度环生成 Iq 引用；电流环始终在高频。

### 6.3 引用值设定

```c
void motor_SetVoltageReference(motor_handle_t *, foc_scalar_t qD, foc_scalar_t qQ);
void motor_SetCurrentReference(motor_handle_t *, foc_scalar_t qD, foc_scalar_t qQ);
void motor_SetSpeedReference(motor_handle_t *, foc_scalar_t qSpeed);
void motor_SetPositionReference(motor_handle_t *, foc_scalar_t qPosition);
```

| 引用 | 单位 | 生效环 |
|---|---|---|
| `qD`/`qQ`（电压） | pu | 电压开环模式直接输出 |
| `qD`/`qQ`（电流） | pu | 高频 Id/Iq 环 |
| `qSpeed` | 机械 turn/s | 低频速度环输出 Iq |
| `qPosition` | 环绕机械 turn | 低频位置环输出速度 |

### 6.4 诊断：快照与事件环

```c
foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot);
bool motor_DebugReadEvent(motor_handle_t *ptMotor, motor_event_t *ptEvent);
```

`motor_GetSnapshot()` 在临界区内一次拷贝全部字段，不返回任何私有指针。关键字段：`eRunState`、`eControlMode`、`wFaults`、`tActiveAngle`、`qActiveSpeed`、`tCurrent`、`tVoltage`、`tPhaseCurrent`、`tDuty`、`bPwmEnabled`、`eStartupPhase`、`ePendingCommand`。

事件环为固定容量 4 的私有环形缓冲，记录命令接受/拒绝、状态迁移、位置源有效性变化、切换开始/完成/超时和故障。

### 6.5 配置结构

```c
typedef struct {
    motor_params_t          tParams;        /* 电机元数据 */
    foc_hal_t               tHal;           /* PWM/ADC 绑定 */
    motor_control_config_t  tControl;       /* 控制器与调制 */
    current_sensing_type_t  eTopology;      /* 1P/2P/3P */
    motor_time_if_t         tTime;          /* 毫秒时钟 */
    motor_sync_if_t         tSync;          /* 临界区 */
    foc_scalar_t            qHighFrequencyPeriod;   /* 秒（如 0.00005 = 20kHz） */
    foc_scalar_t            qLowFrequencyPeriod;    /* 秒（如 0.001 = 1kHz） */
    foc_position_config_t   tPosition;      /* 机械到电转换 */
    /* 开环到闭环切换门限 */
    foc_scalar_t            qTransitionMinimumConfidence;
    foc_scalar_t            qTransitionMinimumSpeed;
    foc_scalar_t            qTransitionMaximumAngleError;
    uint32_t                wTransitionTimeoutMs;
    uint16_t                hwTransitionQualificationSamples;
    uint16_t                hwTransitionBlendSamples;
    uint32_t                wStartupDelayMs;
} motor_config_t;
```

### 6.6 启动配置与位置源组合规则

```c
typedef struct {
    motor_control_mode_e eControlMode;
    const foc_position_source_if_t *ptInitialPositionSource;
    const foc_position_source_if_t *ptTargetPositionSource;
    foc_scalar_t qInitialAngle;       /* 电角 turn */
    foc_scalar_t qOpenLoopSpeed;      /* 电角 turn/s */
    foc_scalar_t qAcceleration;       /* 电角 turn/s^2 */
    foc_dq_t tVoltageReference;       /* pu */
    foc_dq_t tCurrentReference;       /* pu */
    foc_scalar_t qSpeedReference;     /* 机械 turn/s */
    foc_scalar_t qPositionReference;  /* 环绕机械 turn */
} motor_run_config_t;
```

| 初始源 | 目标源 | 语义 |
|---|---|---|
| NULL | NULL | 持续使用内部开环角度发生器 |
| 非空 | NULL | 直接使用传感器/观测器闭环 |
| NULL | 非空 | 开环拖动，资格判定后平滑切入目标源 |
| 同一指针 | 同一指针 | 直接闭环，不执行切换 |
| 两个不同非空源 | — | 第一版拒绝 |

### 6.7 故障与返回值

故障位（掩码组合）：`MOTOR_FAULT_HARDWARE`、`MOTOR_FAULT_CURRENT_SAMPLE`、`MOTOR_FAULT_INVALID_COMMAND`、`MOTOR_FAULT_POSITION_SOURCE`、`MOTOR_FAULT_TRANSITION_TIMEOUT`。

返回值：`FOC_RESULT_OK`、`FOC_RESULT_NULL`、`FOC_RESULT_INVALID_ARGUMENT`、`FOC_RESULT_BUSY`、`FOC_RESULT_OUT_OF_RANGE`、`FOC_RESULT_DIVIDE_BY_ZERO`、`FOC_RESULT_DISABLED`、`FOC_RESULT_SAFETY`。

原则：所有 `Init` 返回值必须检查；高频路径失败立即急停；FAULT 下不能再次启动；不要在中断中打印错误。

## 7. 四种运行场景完整示例

以下骨架为四个场景共用。示例中的 PID 参数仅用于说明，不能直接上电。

### 7.0 公共骨架

```c
#include "foc/foc.h"
#include "perfc_port.h"

static motor_handle_t s_tMotor;

static uint32_t port_GetMilliseconds(void *pContext) {
    (void)pContext;
    return (uint32_t)get_system_ms();
}
static uintptr_t port_EnterCritical(void *pContext) {
    (void)pContext;
    return (uintptr_t)perfc_port_disable_global_interrupt();
}
static void port_ExitCritical(void *pContext, uintptr_t wState) {
    (void)pContext;
    perfc_port_resume_global_interrupt((uint32_t)wState);
}

static motor_config_t s_tMotorConfig = {
    .tParams = { .chPolePairs = 4U },
    .eTopology = SENSING_TOPOLOGY_3P,
    .tTime = { .pContext = NULL, .fnGetMilliseconds = port_GetMilliseconds },
    .tSync = { .pContext = NULL, .fnEnter = port_EnterCritical,
               .fnExit = port_ExitCritical },
    .qHighFrequencyPeriod = FOC_SCALAR(0.00005f), /* 20 kHz */
    .qLowFrequencyPeriod  = FOC_SCALAR(0.001f),   /* 1 kHz */
    .tPosition = { .chPolePairs = 4U, .chDirection = 1 },
    .qTransitionMinimumConfidence = FOC_SCALAR(0.8f),
    .qTransitionMinimumSpeed      = FOC_SCALAR(0.05f),
    .qTransitionMaximumAngleError = FOC_SCALAR(0.25f),
    .wTransitionTimeoutMs = 1000U,
    .hwTransitionQualificationSamples = 3U, .hwTransitionBlendSamples = 8U,
    .wStartupDelayMs = 200U,
};

static foc_result_t demo_Init(void) {
    /* 初始化 PID 并绑定到 tControl；
       调用 board_BindFocHal(&s_tMotorConfig.tHal) 绑定平台 HAL。 */
    return motor_Init(&s_tMotor, &s_tMotorConfig);
}

/* 高频：PWM/ADC 控制中断（20 kHz） */
void ADC_Preempt_IRQHandler(void) {
    (void)motor_HighFrequencyStep(&s_tMotor);
}
/* 低频：ms 级系统调度（1 kHz） */
void SystemClock_1ms(void) {
    (void)motor_LowFrequencyStep(&s_tMotor);
}
/* 主循环：驱动 FSM 并 drain 事件环 */
void demo_MainLoop(void) {
    motor_event_t tEvent;
    for (;;) {
        (void)motor_RunFSM(&s_tMotor);
        while (motor_DebugReadEvent(&s_tMotor, &tEvent)) {
            demo_log_event(&tEvent);
        }
    }
}
```

### 7.1 场景一：D/Q 电压开环

```c
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
    .ptInitialPositionSource = NULL, .ptTargetPositionSource = NULL,
    .qInitialAngle = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(1.0f), .qAcceleration = FOC_SCALAR(5.0f),
    .tVoltageReference = { .qD = FOC_ZERO, .qQ = FOC_SCALAR(0.05f) },
};
/* 运行中调压：motor_SetVoltageReference(&s_tMotor, FOC_ZERO, FOC_SCALAR(0.08f)); */
```

### 7.2 场景二：开环角度 + D/Q 电流闭环

```c
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_CURRENT,
    .ptInitialPositionSource = NULL, .ptTargetPositionSource = NULL,
    .qInitialAngle = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(1.0f), .qAcceleration = FOC_SCALAR(5.0f),
    .tCurrentReference = { .qD = FOC_ZERO, .qQ = FOC_SCALAR(0.10f) },
};
/* motor_Start() 强制校验 Id/Iq 控制器绑定。 */
```

### 7.3 场景三：有传感器直接闭环（Hall）

```c
static foc_hall_t s_tHall;
static foc_hall_source_adapter_t s_tHallAdapter;
static foc_position_source_if_t s_tHallSource;

static uint8_t board_ReadHallCode(void *pHardwareContext) {
    (void)pHardwareContext;
    return board_hall_read();  /* 经 MDI 读三相 Hall，0-7 */
}
static foc_result_t demo_InitHall(void) {
    foc_hall_params_t tParams;
    foc_hall_DefaultParams(&tParams);
    /* 按实际接线修改 tParams.achSectorByCode */
    foc_hall_Init(&s_tHall, &tParams);
    foc_hall_source_Init(&s_tHallAdapter, &s_tHall, NULL, board_ReadHallCode);
    s_tHallSource = foc_hall_PositionSourceInterface(&s_tHallAdapter);
    return FOC_RESULT_OK;
}
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_SPEED,
    .ptInitialPositionSource = &s_tHallSource,
    .ptTargetPositionSource  = &s_tHallSource,  /* 同源：直接闭环 */
    .qSpeedReference = FOC_SCALAR(2.0f),        /* 机械 turn/s */
};
```

### 7.4 场景四：开环拖动到观测器接管

```c
static foc_smo_t s_tSmo;
static foc_position_source_if_t s_tSmoSource;

static foc_result_t demo_InitSmo(void) {
    const foc_smo_params_t tParams = {
        .qModelGain = FOC_SCALAR(0.5f), .qResistance = FOC_SCALAR(0.1f),
        .qSlidingGain = FOC_SCALAR(0.2f), .qBoundaryInverse = FOC_SCALAR(2.0f),
        .qEmfFilterAlpha = FOC_SCALAR(0.1f), .qMinimumBemf = FOC_SCALAR(0.01f),
    };
    foc_smo_Init(&s_tSmo, &tParams);
    s_tSmoSource = foc_smo_PositionSourceInterface(&s_tSmo);
    return FOC_RESULT_OK;
}
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_CURRENT,
    .ptInitialPositionSource = NULL,          /* 先开环拖动 */
    .ptTargetPositionSource  = &s_tSmoSource, /* 资格判定后接管 */
    .qInitialAngle = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(1.0f), .qAcceleration = FOC_SCALAR(5.0f),
    .tCurrentReference = { .qD = FOC_ZERO, .qQ = FOC_SCALAR(0.10f) },
};
/* 快照跟踪：eStartupPhase: QUALIFY_SOURCE -> BLEND_ANGLE -> COMPLETE */
```

## 8. 统一位置源适配说明

### 8.1 接口与有效位

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

输出通过有效位声明能力：

| 有效位 | 含义 |
|---|---|
| `FOC_POSITION_VALID_ELECTRICAL_ANGLE` | 电角度有效（环绕 turn） |
| `FOC_POSITION_VALID_ELECTRICAL_SPEED` | 电角速度有效（turn/s） |
| `FOC_POSITION_VALID_MECHANICAL_ANGLE` | 机械角有效（环绕 turn） |
| `FOC_POSITION_VALID_MECHANICAL_SPEED` | 机械速度有效（turn/s） |
| `FOC_POSITION_VALID_MULTI_TURN` | 多圈计数有效 |

### 8.2 各类源的填法

| 源类型 | 应置有效位 | 说明 |
|---|---|---|
| Hall | `ELECTRICAL_ANGLE` + `ELECTRICAL_SPEED` | 天然电角；无效码 0/7 清有效位并报故障 |
| AB/ABZ 编码器 | `MECHANICAL_ANGLE` + `MECHANICAL_SPEED` | 只报机械量，电角由 motor 换算 |
| 观测器（SMO/NLFO） | `ELECTRICAL_ANGLE` + `ELECTRICAL_SPEED` | 天然电角，附 `qConfidence` |

### 8.3 机械角到电角转换

极对数、方向、机械零位和电角偏移由 motor 统一管理：

```c
typedef struct {
    foc_angle_t tMechanicalZero;
    foc_angle_t tElectricalOffset;
    uint8_t chPolePairs;
    int8_t chDirection;      /* 仅 +1 或 -1 */
} foc_position_config_t;
```

提供机械角的源只填机械量，不自行换算电角；天然电角源直接填电角并通过有效位标识。

## 9. 控制器与辅助算法概览

统一控制器接口 `foc_controller_if_t`：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_scalar_t (*fnStep)(void *pContext, foc_scalar_t qReference,
                           foc_scalar_t qFeedback);
    void (*fnTrack)(void *pContext, foc_scalar_t qOutput,
                    foc_scalar_t qReference, foc_scalar_t qFeedback);
} foc_controller_if_t;
```

现成适配器：`foc_controller_FromPid()`、`foc_ladrc_ControllerInterface()`。PID 和 LADRC 可直接绑定到 `motor_control_config_t`。SMC、STA 需产品写小型上下文适配器。

| 模块 | 典型用途 |
|---|---|
| PID | 电流、速度、位置环 |
| LADRC | 带扩张状态观测的鲁棒控制 |
| SMC / STA | 一阶 / 超螺旋二阶滑模控制 |
| LPF / Biquad | 一阶低通 / Butterworth、Chebyshev、Bessel |
| PLL | 角度跟踪、速度估计 |
| LTD | 参考轨迹限速、限加速度 |
| DOB | 速度和负载扰动估计 |
| Feedforward | D/Q 交叉耦合与反电势前馈 |

## 10. 调制

通过 `motor_control_config_t.eModulation` 选择 `MOTOR_MODULATION_SVPWM` / `SPWM` / `THIRD_HARMONIC`。输出占空比限制在 `[0, 1]`。

## 11. 观测器

Hall、SMO、NLFO 统一实现 `foc_position_source_if_t`，由 motor 在高频步骤中调度。

- Hall：`foc_hall_source_Init()` + `foc_hall_PositionSourceInterface()`。
- SMO/NLFO：`foc_smo_PositionSourceInterface()` / `foc_nlfo_PositionSourceInterface()`。
- HFI：产生 D 轴高频注入并同步解调，当前以算法模块形式提供，包装成位置源由产品自行完成。
- 观测器间切换：`foc_observer_selector_t` 是两个位置源之间的资格检查和平滑切换器。

## 12. 优化和补偿

- MTPA：`foc_mtpa_Calculate()`，适用于凸极差异电机。
- 弱磁：`foc_field_weakening_Step()`，超过基速后生成负 D 轴电流。
- 死区补偿：`foc_deadtime_Compensate()`，按相电流方向修正占空比。
- 相位延迟：`foc_phase_delay_Compensate()`。
- 齿槽补偿：`foc_cogging_Get()`，表格存储由产品负责。

## 13. 实验功能

NSD 和参数辨识会主动给停止的电机施加电压，默认关闭：

```c
#define FOC_ENABLE_EXPERIMENTAL_NSD       0
#define FOC_ENABLE_EXPERIMENTAL_IDENTIFY  0
```

启用并不代表允许直接上电。每个实验必须配置安全护栏（限流、限速、限母线电压、超时样本数和急停函数）。

## 14. 构建、测试与 Shell 调试命令

### 14.1 构建

```powershell
# AT32F413 浮点
.\make.bat clean TARGET_CHIP=at32f413
.\make.bat BUILD=debug TARGET_CHIP=at32f413 FOC_NUMERIC=float

# 定点
.\make.bat clean TARGET_CHIP=at32f413
.\make.bat BUILD=debug TARGET_CHIP=at32f413 FOC_NUMERIC=fixed

# 主机算法测试
Set-Location tests\foc
mingw32-make clean
mingw32-make all
```

`all` 依次执行：封装负面编译检查 + 浮点测试套件 + 定点测试套件。

### 14.2 应用层控制命令 (`motor`)

在 RTT Shell 终端中注册的标准业务控制命令集：

- `motor start`：按既定配置启动电机。
- `motor stop`：常规平滑软停机。
- `motor vq <volts-pu>`：在线更新开环 Vq 参考电压（如 `motor vq 0.05`）。
- `motor pid <kp> <ki>`：在线整定 D/Q 轴电流环 PID 增益。
- `motor status`：读取快照，打印运行状态、故障字、电角度、占空比、相电流、ADC 偏置。
- `motor clear`：清除挂起的故障状态。

### 14.3 电机验证与调试模块 (`motor_verify`)

`motor_verify` 命令专为新硬件上电初期的相序定位、静态锁角、开环旋转与电流闭环测试设计。只需在 `foc_config.h` 中置 `FOC_ENABLE_MOTOR_VERIFY 1`，无需修改应用层代码。

```bash
motor_verify static <turns> [vq=0.05]    # 静态电角度锁定
motor_verify run <speed-hz> [vq=0.05]    # 开环旋转强拖
motor_verify current <iq-pu> [speed-hz]  # 电流闭环测试
motor_verify stop                        # 平滑停止
```

## 15. 参数整定顺序

1. 验证急停、硬件过流、PWM 极性和安全占空比。
2. 验证 ADC 触发点、三相零偏、电流方向和 Clarke 结果。
3. 验证编码器/Hall 电角度方向、极对数和零位。
4. 使用很小 D/Q 电压验证开环相序和 Park/反 Park 方向（场景 7.1）。
5. 在开环角度下整定 Id、Iq 电流环（场景 7.2）。
6. 整定速度环，再整定位置环（场景 7.3）。
7. 验证 SMO/NLFO 置信度与开环到闭环接管门限（场景 7.4）。
8. 最后启用前馈、MTPA、弱磁、死区和齿槽补偿。
9. NSD 和参数辨识只在专门测试流程中启用。

## 16. 移植检查表

- [ ] 选择且只选择一个数值后端。
- [ ] `foc/` 未包含新平台的厂商头文件。
- [ ] `foc_pwm_if_t` 三回调齐全，占空比范围 `[0, 1]`。
- [ ] 正常使能和硬件急停是两条独立路径。
- [ ] `fnReconstruct` 采样点避开开关噪声和不可重构区。
- [ ] `fnOffsetCalib` 在无相电流条件下执行并置 `bIsCalibrated`。
- [ ] `motor_sync_if_t` 成对绑定且中断安全。
- [ ] `motor_time_if_t` 已绑定（有启动延时或开环到闭环切换时必需）。
- [ ] 高频 ISR 频率与 `qHighFrequencyPeriod` 一致，低频调度与 `qLowFrequencyPeriod` 一致。
- [ ] 位置源有效位、时间戳和故障位填写符合第 8 章约定。
- [ ] 机械角源不自行换算电角，`tPosition` 配置正确。
- [ ] 控制器增益按各自周期离散化。
- [ ] 切换门限经真实噪声验证。
- [ ] 所有故障路径最终调用功率级急停。
- [ ] 多电机没有共享可变算法对象或硬件上下文。
- [ ] 实验功能与硬件诊断在量产配置中保持关闭。

## 17. 常见问题

### 启动后立即过流

优先检查相序、电流方向、电角度方向、零位和 PWM 极性，不要先调 PID。

### `motor_Start()` 返回 `FOC_RESULT_INVALID_ARGUMENT`

依次检查：当前是否在 IDLE；故障位是否未清除；位置源组合是否合法；所选模式要求的控制器是否都已绑定；开环到闭环切换是否缺毫秒时钟绑定。

### 切换一直停在资格判定相位

观察快照 `eCandidateSourceValidFlags`、`qAngleError` 和门限值；`wTransitionTimeoutMs` 超时后默认急停。

### 定点结果和浮点略有差异

Q16.15 存在量化和饱和。检查基准值、增益范围和中间量是否合理。

### M0 上是否可以使用

支持不带 FPU 的 Cortex-M0。应选择 `FOC_NUMERIC=fixed`，避免在中断中调用 `foc_from_float()`。

## 18. 公共接口索引

| 头文件 | 职责 |
|---|---|
| `math/foc_numeric.h` | 标量、饱和运算、增益和 `foc_result_t` |
| `math/foc_angle.h` | BAM32 归一化角度、环绕、三角函数 |
| `middleware/foc_core.h` | Clarke、Park、反 Park |
| `hal/foc_hal.h` | PWM/ADC 统一包装与校验 |
| `motor/motor.h` | 电机对象公共 API |
| `motor/motor_types.h` | 句柄存储、配置、快照、事件、故障与状态枚举 |
| `motor/motor_position.h` | 统一位置源接口、有效位与转换配置 |
| `control/foc_controller.h` | 可绑定的标量控制器接口 |
| `control/foc_pid.h` / `foc_ladrc.h` / `foc_smc.h` / `foc_sta.h` | 各控制器 |
| `control/foc_filter.h` / `foc_pll.h` / `foc_ltd.h` / `foc_dob.h` | 滤波与辅助控制 |
| `control/foc_feedforward.h` | PMSM 解耦前馈 |
| `modulation/foc_modulation.h` | 三种调制算法 |
| `observer/foc_hall.h` / `foc_smo.h` / `foc_nlfo.h` / `foc_hfi.h` | 观测器与适配器 |
| `observer/foc_observer_selector.h` | 位置源间切换器 |
| `optimization/foc_optimization.h` / `foc_cogging.h` | MTPA、弱磁、死区、齿槽补偿 |
| `experimental/foc_experiment.h` / `foc_nsd.h` / `foc_identify.h` | 实验安全与辨识 |
| `experimental/foc_verify.h` | 上电初试验证例程 |
| `diagnostic/motor_diagnostic.h` | 硬件诊断输出（默认不进构建） |

应用通常只需 `#include "foc/foc.h"`。
