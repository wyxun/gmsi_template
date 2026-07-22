# 通用 FOC 电机控制库

该目录提供一个与 MCU 架构无关、支持多电机实例的 PMSM/BLDC 矢量控制库。核心算法同时支持浮点和 Q16.15 定点运算。电机对象是不透明的静态句柄：应用只通过 `motor.h` 的命令式公共 API 操作电机，不能访问其私有运行状态、控制器、采样值、HAL 或校准数据。

本文以“把库移植到一个新平台并把一个电机实例运行起来”为主线：

- 第 4 章给出两条接口边界总览和最小移植步骤；
- 第 5 章定义硬件层接口（平台必须实现的输入侧）；
- 第 6 章定义应用层接口（库提供的输出侧）；
- 第 7 章给出四种运行场景的完整用法示例；
- 第 8 章说明统一位置源适配。

本文中的数值仅用于说明 API，不能直接作为未知电机的上电参数。

## 1. 能力与边界

当前库包含：

- 不透明多实例电机对象：静态分配、无动态内存、内部 `fsm_rt_t` 生命周期。
- 命令式公共 API：Start/Stop/引用设定/快照/事件环，状态迁移由内部 FSM 决定，不提供 `motor_SetState()`。
- Clarke、Park、反 Park 坐标变换。
- SVPWM、SPWM、三次谐波注入 SPWM。
- 电压开环、电流、速度和位置级联控制。
- PID、LADRC、SMC、STA、PLL、LTD、DOB、滤波和 PMSM 解耦前馈。
- 统一位置源接口：Hall、SMO、NLFO、HFI，以及 motor 内部的资格判定与开环到闭环平滑接管。
- MTPA、弱磁、死区、相位延迟和齿槽转矩补偿。
- 默认关闭的 NSD 转子极性检测、电机参数辨识和硬件诊断输出。
- 浮点与 Q16.15 定点后端。
- 固定容量事件环与一致性快照，封装不形成调试黑盒。

库不会替产品完成以下工作：

- 配置 PWM 定时器、互补输出、死区、刹车输入和 ADC 触发点。
- 决定电流采样电阻、运放增益、ADC 偏置和物理量归一化比例。
- 提供平台临界区和毫秒时钟；二者由产品通过 `motor_sync_if_t` 与 `motor_time_if_t` 绑定（见 5.3、5.4）。
- 把 `motor_HighFrequencyStep()` / `motor_LowFrequencyStep()` 挂到真实中断和调度器；调度频率必须与配置的控制周期一致（见 5.6、6.2）。
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
├── app/           当前项目的应用示例，不属于通用移植必需层
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

`motor_handle_t` 只暴露固定大小、正确对齐的私有存储（`MOTOR_HANDLE_STORAGE_SIZE` 字节），业务成员全部在 `foc/motor/motor_private.h`。应用代码包含 `motor.h` 即可，任何直接成员访问都无法通过编译（`tests/foc` 的 encapsulation 目标对此有负面编译检查）。

## 3. 数值、单位和角度约定

### 3.1 选择数值后端

每次构建必须且只能选择一种后端：

```c
#define FOC_NUMERIC_FLOAT 1
/* 或 */
#define FOC_NUMERIC_FIXED 1
```

项目 Makefile 已提供统一变量：

```powershell
.\make.bat TARGET_CHIP=at32f413 FOC_NUMERIC=float
.\make.bat TARGET_CHIP=at32f413 FOC_NUMERIC=fixed
```

在代码中统一使用 `foc_scalar_t`（`q_type` 是其兼容别名）。不要根据后端直接声明 `float` 或 `int32_t` 保存控制量。

### 3.2 Q16.15

定点后端使用带符号 32 位存储、15 位小数：

- `FOC_ONE` 表示 1.0。
- `FOC_HALF` 表示 0.5。
- `FOC_SCALAR(0.25f)` 生成后端对应的常量。
- `foc_from_float()` 和 `foc_to_float()` 用于非实时边界转换。

高频中断中应使用预先计算的 `foc_scalar_t` 和 `foc_gain_t`，避免反复进行浮点转换。

### 3.3 归一化量

控制器和调制器使用归一化值。通常约定：

- 相电流、D/Q 电流：额定或允许峰值电流为 1 pu。
- D/Q 电压：可用母线电压或最大调制电压为 1 pu。
- 占空比：范围 `[0, 1]`。
- 速度引用：机械 turn/s；开环角速度：电角 turn/s（见 6.3 单位表）。
- 置信度：范围 `[0, 1]`。

`motor_params_t` 中的毫欧、微亨等字段是电机元数据。当前基础控制编排不会自动把这些物理量转换为控制器增益。

### 3.4 角度

`foc_angle_t` 使用“圈”而不是弧度：

- 0.0 表示 0°。
- 0.25 表示 90°。
- 0.5 表示 180°。
- 1.0 会环绕到 0.0。

常用接口：

```c
foc_angle_t tAngle = foc_angle_from_turns(0.25f);
foc_scalar_t qSine = foc_angle_sin(tAngle);
foc_scalar_t qDelta = foc_angle_diff(tTarget, tActual);
tAngle = foc_angle_wrap(tAngle);
```

快照中的 `tActiveAngle`/`tElectricalAngle` 是电角度。机械角到电角的转换（极对数、方向、机械零位、电角偏移）由 motor 按 `motor_config_t.tPosition` 统一完成，应用和位置源适配器不得自行换算（见第 8 章）。

### 3.5 增益

`foc_gain_t` 把整数和小数部分分开，可表达大于 1 的增益，同时保持统一的定点实现。推荐只通过构造函数创建：

```c
foc_gain_t tGain;

if (foc_gain_from_float(2.5f, &tGain) != FOC_RESULT_OK) {
    /* 参数错误，禁止启动电机。 */
}
```

初始化函数会拒绝非法的增益小数部分。不要手工写入超出 `[-1, 1]` 的 `qFraction`。

## 4. 移植总览：两条接口边界

FOC 库与外部世界只有两条边界。把库搬到新平台，只需要实现输入侧绑定，然后按输出侧 API 驱动电机。

### 4.1 两侧接口一览

输入侧（平台必须实现，motor 库消费）：

| 绑定 | 类型 | 必需性 |
|---|---|---|
| PWM 输出 | `foc_pwm_if_t` | 三个回调全部必需 |
| 电流采样 | `foc_adc_if_t` | `fnReconstruct` 必需；`fnOffsetCalib` 校准必需；`fnGetRaw`、`fnStartConversion` 可选 |
| 临界区 | `motor_sync_if_t` | 实时步骤在 ISR 中运行时必需成对绑定 |
| 毫秒时钟 | `motor_time_if_t` | 启动延时或开环到闭环切换时必需 |
| 位置源硬件 | 如 `foc_hall_read_code_fn_t` | 有传感器方案必需 |

各回调的调用方与上下文：`fnSetDuty` 于高频 ISR；`fnEnable` 于 FSM 临界区内；`fnEmergencyStop` 于任意故障路径；`fnReconstruct` 于高频 ISR；`fnOffsetCalib` 于启动校准（PWM 关闭）；sync 用于快照拷贝、命令邮箱、事件环和引用更新；time 用于 FSM（主循环上下文）；位置源硬件回调于高频 Step 内经 `foc_position_source_if_t` 间接调用。

输出侧（库提供，应用调用）：

| 分组 | API |
|---|---|
| 生命周期 | `motor_Init` / `motor_Reset` |
| 命令 | `motor_Start` / `motor_Stop` / `motor_ClearFault` / `motor_EmergencyStop` |
| 流程驱动 | `motor_RunFSM`（主循环） |
| 实时步骤 | `motor_HighFrequencyStep`（PWM/ADC ISR） / `motor_LowFrequencyStep`（ms 级调度） |
| 引用设定 | `motor_SetVoltageReference` / `motor_SetCurrentReference` / `motor_SetSpeedReference` / `motor_SetPositionReference` |
| 诊断 | `motor_GetSnapshot` / `motor_DebugReadEvent` / `motor_GetRawCurrent` |

应用通常只需一个头文件：

```c
#include "foc/foc.h"
```

### 4.2 最小移植步骤

1. 实现 `foc_pwm_if_t` 三个回调（经 MDI 访问定时器，不包含厂商头）。
2. 实现 `foc_adc_if_t`，重点是 `fnReconstruct` 的采样、零偏、归一化和拓扑重构。
3. 绑定 `motor_sync_if_t`：推荐 perf_counter 多架构全局中断守卫（见 5.3）。
4. 绑定 `motor_time_if_t`：包装系统毫秒时钟（见 5.4）。
5. 填写 `motor_config_t`（控制周期、`tPosition`、切换门限）并调用 `motor_Init()`，检查返回值。
6. 把 `motor_HighFrequencyStep()` 挂到 PWM/ADC 控制中断，频率与 `qHighFrequencyPeriod` 一致。
7. 把 `motor_LowFrequencyStep()` 挂到 ms 级调度，主循环每轮调用 `motor_RunFSM()`。
8. 需要传感器或观测器时，实现位置源并在 `motor_run_config_t` 中绑定（见第 8 章）。
9. 用 `motor_GetSnapshot()` 确认 IDLE → STARTING → RUNNING 迁移与故障位行为后，再按第 16 章顺序小信号上电。

完整真实范例见 `foc/app/foc_app.c`（唯一已迁移的应用）与 `peripheral/at32f413/foc_hal_mdi_adapter.c`（MDI 适配器）。

## 5. 硬件层接口（输入侧：平台必须实现）

本章逐项列出移植者必须实现的回调。所有回调都接收自己的 `pContext`，禁止依赖全局“当前电机”。所有回调必须有界、非阻塞；motor 不会替回调做超时保护。

### 5.1 PWM 接口 `foc_pwm_if_t`

```c
typedef struct {
    void *pContext;
    foc_result_t (*fnSetDuty)(void *pContext,
                              q_type qDutyU,
                              q_type qDutyV,
                              q_type qDutyW);
    foc_result_t (*fnEnable)(void *pContext, bool bEnable);
    void (*fnEmergencyStop)(void *pContext);
} foc_pwm_if_t;
```

- `fnSetDuty`：输入 `[0, 1]` 的三相占空比。由 `motor_HighFrequencyStep()` 在高频控制中断中调用，每个高频周期一次。要求：有界寄存器写、三相同步更新、非阻塞。返回错误会触发急停。
- `fnEnable`：`false` 用于正常停机。由 motor FSM 在主循环上下文中调用，且调用发生在临界区内——这是 motor 允许的唯一“临界区内 HAL 调用”例外，因此实现必须是有界寄存器写，并禁止回调任何 motor API。
- `fnEmergencyStop`：在任意上下文（含 ISR 与故障路径）调用，必须立即关闭功率输出，不能只设置软件标志。

### 5.2 电流采样接口 `foc_adc_if_t`

```c
typedef struct {
    void *pContext;
    foc_result_t (*fnStartConversion)(void *pContext);
    foc_result_t (*fnOffsetCalib)(void *pContext,
                                  foc_adc_calib_t *ptCalib);
    foc_result_t (*fnGetRaw)(void *pContext,
                             uint32_t *pwRawU,
                             uint32_t *pwRawV,
                             uint32_t *pwRawW);
    foc_result_t (*fnReconstruct)(void *pContext,
                                  phase_current_handle_t *ptHandle);
} foc_adc_if_t;
```

- `fnReconstruct`（必需）：由 `motor_HighFrequencyStep()` 在高频控制中断中调用，每个高频周期一次。负责：读取本周期 ADC/DMA 结果；减去 `ptHandle->tCalib` 中的零偏；按采样电阻和运放增益换算并归一化；按 `ptHandle->eTopology`（1P/2P/3P）重构 `qIu/qIv/qIw`；检查结果有效性，异常返回错误（motor 立即以 `MOTOR_FAULT_CURRENT_SAMPLE` 急停）。
- `fnOffsetCalib`（启动校准必需）：由 motor FSM 在启动校准相位调用，此时 PWM 已关闭、电机无相电流。实现把零偏写入 `ptCalib->wOffsetU/V/W` 并置 `bIsCalibrated`。产品若在生产阶段保存零偏，应在此回调中返回保存值；不得绕过该回调直写校准数据。
- `fnGetRaw`（可选）：主循环诊断用，经 `motor_GetRawCurrent()` 调用。
- `fnStartConversion`（可选）：是否使用取决于板级时序。典型中心对齐 PWM 由定时器事件触发 ADC，控制中断直接消费 DMA 结果，不需要软件启动转换。

### 5.3 临界区接口 `motor_sync_if_t`

```c
typedef struct {
    void *pContext;
    uintptr_t (*fnEnter)(void *pContext);
    void (*fnExit)(void *pContext, uintptr_t wState);
} motor_sync_if_t;
```

motor 在快照拷贝、命令邮箱、事件环读写和引用值更新处进入临界区，保证主循环与 ISR 之间不产生撕裂读取。`fnEnter` 返回的状态字原样传回 `fnExit`。要求：有界、中断安全、允许嵌套；两个回调必须同时为空或同时非空（`motor_Init()` 校验）。实时步骤在 ISR 中运行的目标必须绑定；纯单线程主机测试可以全部留空。

目标侧推荐 perf_counter 多架构全局中断守卫（Cortex-M 用 PRIMASK，RISC-V 用 mstatus.MIE，其他架构空实现，不含厂商头）：

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

返回单调递增的毫秒值，供启动延时（`wStartupDelayMs`）和切换超时（`wTransitionTimeoutMs`）使用。由 motor FSM 在主循环上下文中调用，无硬实时要求。`wStartupDelayMs != 0` 或启动配置要求开环到闭环切换时必须绑定，否则 `motor_Init()` / `motor_Start()` 拒绝。典型实现是包装 `get_system_ms()`：

```c
static uint32_t port_GetMilliseconds(void *pContext)
{
    (void)pContext;
    return (uint32_t)get_system_ms();
}
```

### 5.5 位置源硬件回调

物理位置源的硬件读取封装在各自适配器内。以 Hall 为例，产品提供读码回调：

```c
typedef uint8_t (*foc_hall_read_code_fn_t)(void *pHardwareContext);
```

该回调由 motor 的高频 Step 经 `foc_position_source_if_t.fnStep` 间接调用，因此同样要求有界、非阻塞，返回 0-7 的 Hall 码。AB/ABZ、光栅、磁编码器由产品实现自己的 `fnStep`，在其中经 MDI 读取硬件并填写统一输出（见第 8 章）。观测器（SMO/NLFO）没有硬件回调，直接消费统一输入中的电流、电压和采样周期。

### 5.6 调用上下文与频率总表

| 调用点 | 函数 | 频率 |
|---|---|---|
| PWM/ADC 控制中断 | `motor_HighFrequencyStep()` | PWM 载波频率（如 20 kHz） |
| ms 级系统调度 | `motor_LowFrequencyStep()` | 1 kHz 级 |
| 主循环 | `motor_RunFSM()` | 每轮主循环，线程上下文 |
| 主循环按需 | `motor_GetSnapshot()` 等诊断/命令 API | 按需，线程上下文 |

高频步骤在 ISR 上下文执行，每电机实例不可重入，重入返回 `FOC_RESULT_BUSY`。主循环按需组包括 `motor_DebugReadEvent()`、`motor_Set*Reference()`、`motor_Start()`、`motor_Stop()` 和 `motor_ClearFault()`。

真实调度频率必须与 `motor_config_t.qHighFrequencyPeriod` /`qLowFrequencyPeriod` 一致：开环角按配置周期推进，级联控制器按配置周期离散化，失配会导致转速和控制周期错误。高频中断应固定周期、低抖动，并在下一次 PWM 更新前完成；日志、阻塞延时和参数保存不得放入该中断。

### 5.7 参考适配器

当前仓库中的参考实现：

- `peripheral/at32f413/foc_hal_mdi_adapter.c`
- `peripheral/stm32g431/foc_hal_mdi_adapter.c`

它们通过 MDI 访问硬件并提供 `foc_hal_mdi_Bind()` /`foc_hal_mdi_BindDefault()` 一键绑定，不应把其中的厂商实现复制到通用业务层。注意：当前 HAL 没有母线电压采样回调，`motor_snapshot_t.qVbus` 为保留字段，本版本不填充（恒为 0）。

## 6. 应用层接口（输出侧：库提供）

本章逐项说明 `motor.h` 公共 API 的输入、输出、语义和单位。所有函数先校验句柄已初始化，非法句柄返回 `FOC_RESULT_NULL` 或 `FOC_RESULT_INVALID_ARGUMENT`。

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

- `motor_Init()`：拷贝配置并复位对象，成功后状态为 IDLE。校验内容：极对数非 0（`tParams` 与 `tPosition` 两处）；采样拓扑在 `SENSING_TOPOLOGY_1P..3P`；高、低频周期均大于 0；`tPosition.chDirection` 为 ±1；切换置信度在 `[0, 1]`、最小速度非负、最大角误差在 `[0, 0.5]`；`wStartupDelayMs != 0` 时必须绑定毫秒时钟；`tSync` 两个回调必须成对；HAL 四个必需回调齐全。它不会自动校准 ADC，也不会自动使能 PWM。
- `motor_Reset()`：清除运行状态回到 IDLE。不应作为未经故障诊断的自动重新启动手段。
- `motor_Start()`：校验启动配置后把 START 命令放入内部邮箱并立即返回；状态迁移由 `motor_RunFSM()` 执行，PWM 在校准完成后才使能。返回值：
  - `FOC_RESULT_OK`：命令已接受；
  - `FOC_RESULT_BUSY`：已有挂起命令，稍后重试；
  - `FOC_RESULT_INVALID_ARGUMENT`：配置非法（组合规则、控制器绑定、时钟缺失）或当前不在 IDLE。

校验规则：模式枚举有效；位置源接口有效（`fnStep` 非空）；两个不同的非空位置源直接拒绝；SPEED 及以上模式必须绑定位置源；CURRENT 起要求 Id/Iq 控制器，SPEED 起要求速度控制器，POSITION 起要求位置控制器；开环到闭环切换要求毫秒时钟，且 SPEED/POSITION 切换要求对应外环控制器支持 `fnTrack` 预置。
- `motor_Stop()`：STARTING/RUNNING 时接受；存在挂起 START 时把它改判为 STOP；其余状态拒绝。STOPPING 期间 FSM 先关闭 PWM 再回到 IDLE。
- `motor_RunFSM()`：主循环每轮调用，返回 `fsm_rt_t`。负责校准、启动延时、开环拖动、源资格判定、角度融合、停止和故障流程，启动子相位由快照的 `eStartupPhase` 暴露。它不承担硬实时控制计算。
- `motor_ClearFault()`：仅在 FAULT 且 PWM 已关闭时成功，清除故障位并回到 IDLE。
- `motor_EmergencyStop()`：唯一绕过 FSM、立即关闭功率输出的公共接口。置故障位、进入 FAULT 并调用 `fnEmergencyStop`；ISR 安全。ISR 内只置位和急停，日志在主循环输出。

命令被拒绝只产生 `MOTOR_EVENT_COMMAND_REJECTED` 事件，不会置故障位；故障位（`MOTOR_FAULT_*`）只由真实危险路径设置，必须诊断后用 `motor_ClearFault()` 清除。两者在应用日志中应分开打印。

### 6.2 实时步骤与调度义务

```c
foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor);
foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor);
```

`motor_HighFrequencyStep()` 必须由 PWM/ADC 控制中断以真实载波频率调用，内部固定数据流：

```text
接管引用值 -> 电流采样 -> 更新当前和候选位置源
-> 选择或融合控制角度 -> Clarke/Park -> 电流控制
-> 反 Park -> 调制 -> 三相占空比写入 -> 更新诊断快照
```

只有 STARTING/RUNNING 状态才输出 PWM；任一采样、变换、调制或占空比写入失败都会触发急停。开环角按 `qHighFrequencyPeriod` 推进，应用不得自行推进角度。

`motor_LowFrequencyStep()` 由 ms 级调度调用，负责位置环生成速度引用、速度环生成 Iq 引用；电流环始终在高频。缺失的机械角/速度有效位会导致急停，缺失的控制器绑定在 `motor_Start()` 时已被拒绝。

### 6.3 引用值设定

```c
void motor_SetVoltageReference(motor_handle_t *ptMotor,
                               foc_scalar_t qD, foc_scalar_t qQ);
void motor_SetCurrentReference(motor_handle_t *ptMotor,
                               foc_scalar_t qD, foc_scalar_t qQ);
void motor_SetSpeedReference(motor_handle_t *ptMotor,
                             foc_scalar_t qSpeed);
void motor_SetPositionReference(motor_handle_t *ptMotor,
                                foc_scalar_t qPosition);
```

四个 setter 无返回值，在临界区内原子写入，下一控制周期生效；句柄未初始化时不动作。单位约定：

| 引用 | 单位 | 生效环 |
|---|---|---|
| `qD`/`qQ`（电压） | pu | 电压开环模式直接输出 |
| `qD`/`qQ`（电流） | pu | 高频 Id/Iq 环 |
| `qSpeed` | 机械 turn/s，正负表示方向 | 低频速度环输出 Iq |
| `qPosition` | 环绕机械 turn | 低频位置环输出速度 |

### 6.4 诊断：快照与事件环

```c
foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot);
bool motor_DebugReadEvent(motor_handle_t *ptMotor,
                          motor_event_t *ptEvent);
```

`motor_GetSnapshot()` 在临界区内一次拷贝全部字段，保证多字段一致性，不返回任何私有指针。字段语义（单位已冻结，改动即 ABI 变化）：

| 字段 | 含义与单位 |
|---|---|
| `eRunState` | 生命周期状态：`MOTOR_STATE_IDLE/STARTING/STOPPING/RUNNING/FAULT` |
| `eStartupPhase` | 启动子相位：`MOTOR_STARTUP_IDLE/CALIBRATE/WAIT_DELAY/ENABLE/QUALIFY_SOURCE/BLEND_ANGLE/COMPLETE` |
| `ePendingCommand` | 邮箱中挂起的命令：`MOTOR_COMMAND_NONE/START/STOP` |
| `eControlMode` | 控制模式：`MOTOR_CONTROL_VOLTAGE_OPEN_LOOP/CURRENT/SPEED/POSITION` |
| `wFaults` | 故障位掩码（见 6.7） |
| `tOpenLoopAngle` | 独立跟踪的开环启动角，环绕电角 turn |
| `tActiveAngle` / `qActiveSpeed` | 当前控制使用的电角度（turn）与电角速度（turn/s） |
| `tCandidateAngle` / `qCandidateSpeed` | 切换中目标源的电角度与电角速度 |
| `tElectricalAngle` / `qElectricalSpeed` | 与 active 字段同源的兼容别名 |
| `qAngleError` | 候选角与开环角的最短环绕误差，turn |
| `qBlendFactor` | 角度融合系数，`[0, 1]` |
| `tCurrentReference` / `tCurrent` | D/Q 电流引用与反馈，pu |
| `tVoltageReference` / `tVoltage` | D/Q 电压引用与输出，pu |
| `qSpeedReference` / `qPositionReference` | 外环引用（机械 turn/s、机械 turn） |
| `tPhaseCurrent` | 三相电流 `qIu/qIv/qIw`，pu |
| `tDuty` | 三相占空比 `qU/qV/qW`，`[0, 1]` |
| `eActiveSourceValidFlags` | 活动源有效位（`FOC_POSITION_VALID_*`），决定源字段是否有意义 |
| `eCandidateSourceValidFlags` | 候选源有效位，同上 |
| `tCurrentCalibration` | 电流零偏 `wOffsetU/V/W` 与 `bIsCalibrated` |
| `bPwmEnabled` | PWM 使能状态 |
| `qVbus` | 保留字段，当前版本恒为 0（HAL 无母线电压回调） |
| `wEventSequence` | 最近一条事件的单调序号 |
| `wEventOverwriteCount` | 事件环溢出覆盖计数 |

active 指当前控制使用的角度来源，candidate 指资格判定/融合期间的目标源，open-loop 是独立跟踪的启动角。源特有字段只有在对应有效位置位时才有意义。

事件环为固定容量 4 的私有环形缓冲，覆盖最旧记录并计数。记录内容：命令接受/拒绝、状态迁移、位置源有效性变化、切换开始/完成/超时和故障。`motor_DebugReadEvent()` 按 FIFO 读出，返回 `false` 表示空。`motor_event_t` 字段随 `eType` 解释：

| `eType` | 有效字段 |
|---|---|
| `MOTOR_EVENT_COMMAND_ACCEPTED` / `MOTOR_EVENT_COMMAND_REJECTED` | `eCommand`、`eResult` |
| `MOTOR_EVENT_STATE_CHANGED` | `eFromState`、`eToState` |
| `MOTOR_EVENT_SOURCE_VALIDITY_CHANGED` | `ePositionRole`（active/candidate）、`wPreviousValue`/`wCurrentValue`（旧/新有效位） |
| `MOTOR_EVENT_TRANSITION_STARTED` / `COMPLETED` / `TIMEOUT` | `wPreviousValue`/`wCurrentValue`（切换前后启动相位） |
| `MOTOR_EVENT_FAULT` | `wFaults`、`eFromState`（恒迁移到 FAULT） |

事件环只做日志和诊断；正常控制路径不得依赖事件消费。事件记录不在高频路径格式化文本。

### 6.5 初始化配置 `motor_config_t`

```c
typedef struct {
    motor_params_t          tParams;
    foc_hal_t               tHal;
    motor_control_config_t  tControl;
    current_sensing_type_t  eTopology;
    motor_time_if_t         tTime;
    motor_sync_if_t         tSync;
    foc_scalar_t            qHighFrequencyPeriod;
    foc_scalar_t            qLowFrequencyPeriod;
    foc_position_config_t   tPosition;
    foc_scalar_t            qTransitionMinimumConfidence;
    foc_scalar_t            qTransitionMinimumSpeed;
    foc_scalar_t            qTransitionMaximumAngleError;
    uint32_t                wTransitionTimeoutMs;
    uint16_t                hwTransitionQualificationSamples;
    uint16_t                hwTransitionBlendSamples;
    uint32_t                wStartupDelayMs;
} motor_config_t;
```

关键字段：

- `tHal`：第 5.1/5.2 节的平台绑定。
- `tControl`：Id/Iq/速度/位置控制器绑定（`foc_controller_if_t`，可用 `foc_controller_FromPid()` 构造）与调制方式（`MOTOR_MODULATION_SVPWM/SPWM/THIRD_HARMONIC`）。电压开环模式不使用控制器，但建议预先绑定，`motor_Start()` 对闭环模式强制校验。
- `qHighFrequencyPeriod` / `qLowFrequencyPeriod`：每步秒数，`FOC_SCALAR()` 归一化，必须与真实调度频率一致（如 20 kHz 填 `FOC_SCALAR(0.00005f)`，1 kHz 填 `FOC_SCALAR(0.001f)`）。
- `tPosition`：机械到电转换参数，见 8.3。
- 切换门限组（仅开环到闭环切换使用）：置信度下限、最小电角速度（turn/s）、最大环绕角误差（turn）、超时毫秒数、连续资格样本数、融合样本数。资格或超时失败默认急停（第一版不自动退回开环）。
- `wStartupDelayMs`：校准完成到开环拖动之间的非阻塞延时，0 表示跳过。

### 6.6 启动配置 `motor_run_config_t` 与组合规则

```c
typedef struct {
    motor_control_mode_e eControlMode;
    const foc_position_source_if_t *ptInitialPositionSource;
    const foc_position_source_if_t *ptTargetPositionSource;
    foc_scalar_t qInitialAngle;
    foc_scalar_t qOpenLoopSpeed;
    foc_scalar_t qAcceleration;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReference;
    foc_scalar_t qPositionReference;
} motor_run_config_t;
```

`motor_Start()` 拷贝整个结构；位置源描述符被拷贝，但其 `pContext` 必须在整个运行期间保持有效。字段单位：`qInitialAngle` 电角 turn；`qOpenLoopSpeed` 电角 turn/s；`qAcceleration` 电角 turn/s^2；电压/电流引用 pu；速度引用机械 turn/s；位置引用环绕机械 turn。

位置源组合规则：

| 初始源 | 目标源 | 语义 |
|---|---|---|
| NULL | NULL | 持续使用内部开环角度发生器 |
| 非空 | NULL | 直接使用传感器/观测器闭环 |
| NULL | 非空 | 开环拖动，资格判定后平滑切入目标源 |
| 同一指针 | 同一指针 | 直接闭环，不执行切换 |
| 两个不同非空源 | — | 第一版拒绝（`FOC_RESULT_INVALID_ARGUMENT`） |

四种合法组合对应第 7 章的四个示例。

### 6.7 故障与返回值

故障位（`motor_fault_e`，掩码组合）：

| 故障位 | 触发路径 |
|---|---|
| `MOTOR_FAULT_HARDWARE` | 平台/应用上报的硬件故障 |
| `MOTOR_FAULT_CURRENT_SAMPLE` | 高频电流重构失败 |
| `MOTOR_FAULT_INVALID_COMMAND` | 实时路径发现非法控制条件 |
| `MOTOR_FAULT_POSITION_SOURCE` | 活动位置源失效 |
| `MOTOR_FAULT_TRANSITION_TIMEOUT` | 资格判定或融合超时 |

`foc_result_t` 返回值：

| 返回值 | 含义 |
|---|---|
| `FOC_RESULT_OK` | 成功 |
| `FOC_RESULT_NULL` | 必需指针为空 |
| `FOC_RESULT_INVALID_ARGUMENT` | 配置、状态或枚举非法；命令被拒绝 |
| `FOC_RESULT_BUSY` | 有挂起命令或步骤重入，稍后重试 |
| `FOC_RESULT_OUT_OF_RANGE` | 数值超出允许范围 |
| `FOC_RESULT_DIVIDE_BY_ZERO` | 除数为零 |
| `FOC_RESULT_DISABLED` | 实验功能未编译启用 |
| `FOC_RESULT_SAFETY` | 实验安全条件触发 |

原则：

- 所有 `Init` 返回值必须检查，任一失败都禁止启动 PWM。
- 高频采样、变换、调制或占空比写入失败时 motor 立即急停。
- `MOTOR_STATE_FAULT` 下不能再次启动，必须先诊断并 `motor_ClearFault()`。
- 不要在中断中打印错误；记录故障码，在主循环中上报。

## 7. 四种运行场景完整示例

四个示例对应 6.6 的四种合法位置源组合。7.0 给出四者共享的初始化与调度骨架；7.1-7.4 各自给出完整的使用流程（配置差异、run config、启动、诊断、停止与错误处理），骨架中与场景无关的部分不再重复。

示例命名遵循项目约定：`w`=uint32_t、`hw`=uint16_t、`ch`=uint8_t、`b`=bool、`q`=foc_scalar_t；函数 `module_Action()`；类型 `_t`。

### 7.0 公共骨架：初始化与调度挂接

```c
#include "foc/foc.h"
#include "perfc_port.h"        /* perfc_port_*_global_interrupt */

static motor_handle_t s_tMotor;

/* ---- 平台绑定：临界区与毫秒时钟（见 5.3、5.4） ---- */
static uint32_t port_GetMilliseconds(void *pContext)
{
    (void)pContext;
    return (uint32_t)get_system_ms();
}

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

/* ---- 控制器对象：电压开环不用，闭环模式由 motor_Start 强制校验 ---- */
static foc_pid_t s_tPidId;
static foc_pid_t s_tPidIq;
static foc_pid_t s_tPidSpeed;
static foc_pid_t s_tPidPosition;

static motor_config_t s_tMotorConfig = {
    .tParams = {
        .chPolePairs = 4U,
        /* 其余元数据按实际电机填写，不影响基础控制编排。 */
    },
    .eTopology = SENSING_TOPOLOGY_3P,
    .tTime = { .pContext = NULL,
               .fnGetMilliseconds = port_GetMilliseconds },
    .tSync = { .pContext = NULL,
               .fnEnter = port_EnterCritical,
               .fnExit  = port_ExitCritical },
    /* 20 kHz 高频、1 kHz 低频；必须与真实调度频率一致。 */
    .qHighFrequencyPeriod = FOC_SCALAR(0.00005f),
    .qLowFrequencyPeriod  = FOC_SCALAR(0.001f),
    .tPosition = { .chPolePairs = 4U, .chDirection = 1 },
    /* 开环到闭环切换门限（仅场景 7.4 使用）。 */
    .qTransitionMinimumConfidence = FOC_SCALAR(0.8f),
    .qTransitionMinimumSpeed      = FOC_SCALAR(0.05f),
    .qTransitionMaximumAngleError = FOC_SCALAR(0.25f),
    .wTransitionTimeoutMs         = 1000U,
    .hwTransitionQualificationSamples = 3U,
    .hwTransitionBlendSamples         = 8U,
    .wStartupDelayMs = 200U,
};

static foc_result_t demo_Init(void)
{
    const foc_pid_params_t tCurrentParams = {
        .tKp = {0, FOC_SCALAR(0.5f)},
        .tKiTs = {0, FOC_SCALAR(0.01f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.5f),
        .qOutputMaximum = FOC_SCALAR(0.5f),
        .qIntegratorMinimum = FOC_SCALAR(-0.3f),
        .qIntegratorMaximum = FOC_SCALAR(0.3f),
    };
    const foc_pid_params_t tOuterParams = {
        .tKp = {0, FOC_SCALAR(0.2f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.5f),
        .qOutputMaximum = FOC_SCALAR(0.5f),
        .qIntegratorMinimum = FOC_SCALAR(-0.3f),
        .qIntegratorMaximum = FOC_SCALAR(0.3f),
    };

    if (foc_pid_Init(&s_tPidId, &tCurrentParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidIq, &tCurrentParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidSpeed, &tOuterParams) != FOC_RESULT_OK ||
        foc_pid_Init(&s_tPidPosition, &tOuterParams) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    s_tMotorConfig.tControl.tIdController =
        foc_controller_FromPid(&s_tPidId);
    s_tMotorConfig.tControl.tIqController =
        foc_controller_FromPid(&s_tPidIq);
    s_tMotorConfig.tControl.tSpeedController =
        foc_controller_FromPid(&s_tPidSpeed);
    s_tMotorConfig.tControl.tPositionController =
        foc_controller_FromPid(&s_tPidPosition);
    s_tMotorConfig.tControl.eModulation = MOTOR_MODULATION_SVPWM;

    /* 平台 HAL 绑定（见 5.1/5.2；仓库内参考
       foc_hal_mdi_BindDefault()）。 */
    if (board_BindFocHal(&s_tMotorConfig.tHal) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return motor_Init(&s_tMotor, &s_tMotorConfig);
}

/* ---- 调度挂接：频率必须与 s_tMotorConfig 的周期字段一致 ---- */

/* 高频：PWM/ADC 控制中断（本例 20 kHz），唯一高频调用点。 */
void ADC_Preempt_IRQHandler(void)
{
    (void)motor_HighFrequencyStep(&s_tMotor);
}

/* 低频：ms 级系统调度（本例 1 kHz），唯一低频调用点。 */
void SystemClock_1ms(void)
{
    (void)motor_LowFrequencyStep(&s_tMotor);
}

/* 主循环：驱动生命周期 FSM 并把事件环 drain 成日志。 */
void demo_MainLoop(void)
{
    motor_event_t tEvent;

    for (;;) {
        (void)motor_RunFSM(&s_tMotor);
        while (motor_DebugReadEvent(&s_tMotor, &tEvent)) {
            /* 仅日志：正常控制不得依赖事件消费。 */
            demo_log_event(&tEvent);
        }
        /* 产品事件处理、命令下发、状态打印…… */
    }
}
```

快照诊断与停止/错误处理为四个场景共用：

```c
static void demo_PrintStatus(void)
{
    motor_snapshot_t tSnapshot;

    if (motor_GetSnapshot(&s_tMotor, &tSnapshot) != FOC_RESULT_OK) {
        return;
    }
    demo_log("state=%d phase=%d mode=%d pwm=%d flt=0x%lX\r\n",
             (int)tSnapshot.eRunState, (int)tSnapshot.eStartupPhase,
             (int)tSnapshot.eControlMode, (int)tSnapshot.bPwmEnabled,
             (unsigned long)tSnapshot.wFaults);
    demo_log("angle=%.1fdeg speed=%.3f e-turn/s dutyU=%.1f%% Iq=%.3f\r\n",
             (double)foc_angle_to_turns(tSnapshot.tActiveAngle) * 360.0,
             _D(tSnapshot.qActiveSpeed),
             _D(tSnapshot.tDuty.qU) * 100.0,
             _D(tSnapshot.tCurrent.qQ));
}

static void demo_Stop(void)
{
    foc_result_t eResult = motor_Stop(&s_tMotor);

    if (eResult == FOC_RESULT_OK) {
        demo_log("stop accepted\r\n");   /* FSM 关 PWM 后回 IDLE */
    } else {
        demo_log("stop rejected: %d\r\n", (int)eResult);
    }
}

/* 命令拒绝与故障分开处理：拒绝可重试，故障必须先诊断再清除。 */
static bool demo_StartChecked(const motor_run_config_t *ptRunConfig)
{
    motor_snapshot_t tSnapshot;
    foc_result_t eResult;

    if (motor_GetSnapshot(&s_tMotor, &tSnapshot) == FOC_RESULT_OK &&
        tSnapshot.wFaults != MOTOR_FAULT_NONE) {
        demo_log("faults 0x%lX active, clear first\r\n",
                 (unsigned long)tSnapshot.wFaults);
        if (motor_ClearFault(&s_tMotor) != FOC_RESULT_OK) {
            return false;   /* 不在 FAULT 或 PWM 未关 */
        }
    }
    eResult = motor_Start(&s_tMotor, ptRunConfig);
    if (eResult == FOC_RESULT_OK) {
        return true;        /* 命令入邮箱，状态迁移由 RunFSM 完成 */
    }
    if (eResult == FOC_RESULT_BUSY) {
        demo_log("start busy, retry later\r\n");
    } else {
        demo_log("start rejected: %d\r\n", (int)eResult);
    }
    return false;
}
```

### 7.1 场景一：D/Q 电压开环

最简单的上电验证场景：内部开环角度发生器 + 直接 D/Q 电压输出，不采样角度、不使用控制器（仍会做电流采样诊断）。

```c
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
    .ptInitialPositionSource = NULL,    /* 内部开环角度发生器 */
    .ptTargetPositionSource  = NULL,
    .qInitialAngle  = FOC_ZERO,         /* 电角 turn */
    .qOpenLoopSpeed = FOC_SCALAR(1.0f), /* 电角 turn/s */
    .qAcceleration  = FOC_SCALAR(5.0f), /* 电角 turn/s^2 */
    .tVoltageReference = {
        .qD = FOC_ZERO,
        .qQ = FOC_SCALAR(0.05f),        /* 0.05 pu，小信号验证 */
    },
};

void demo_RunVoltageOpenLoop(void)
{
    if (demo_Init() != FOC_RESULT_OK) {
        return;                          /* 初始化失败禁止启动 */
    }
    (void)demo_StartChecked(&s_tRunConfig);
    /* 主循环中：demo_PrintStatus() 观察角度推进与占空比；
       运行中调压（下一高频周期生效）： */
    motor_SetVoltageReference(&s_tMotor, FOC_ZERO, FOC_SCALAR(0.08f));
    /* 停机： */
    demo_Stop();
}
```

调度与 7.0 相同：ADC ISR 调 `motor_HighFrequencyStep()`，1 ms 调度调 `motor_LowFrequencyStep()`（本场景无外环，低频步骤直接返回），主循环调 `motor_RunFSM()`。快照中 `tOpenLoopAngle` 与 `tActiveAngle` 同源推进，`qBlendFactor` 恒为 0。

### 7.2 场景二：开环角度 + D/Q 电流闭环

角度仍由内部发生器提供，但 D/Q 走电流闭环：每个高频周期采样相电流、做 Clarke/Park 并运行 Id/Iq 控制器。`motor_Start()` 强制校验 Id/Iq 控制器绑定。

```c
static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_CURRENT,
    .ptInitialPositionSource = NULL,
    .ptTargetPositionSource  = NULL,
    .qInitialAngle  = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(1.0f),
    .qAcceleration  = FOC_SCALAR(5.0f),
    .tCurrentReference = {
        .qD = FOC_ZERO,
        .qQ = FOC_SCALAR(0.10f),        /* 0.10 pu 转矩电流 */
    },
};

void demo_RunOpenAngleCurrentLoop(void)
{
    if (demo_Init() != FOC_RESULT_OK) {
        return;                          /* 含 Id/Iq PID 绑定 */
    }
    (void)demo_StartChecked(&s_tRunConfig);
    /* 运行中改电流引用： */
    motor_SetCurrentReference(&s_tMotor, FOC_ZERO, FOC_SCALAR(0.15f));
    /* 诊断：快照 tCurrent（反馈）应跟踪 tCurrentReference（引用）；
       tCurrentCalibration.bIsCalibrated 在启动校准完成后为真。 */
    demo_Stop();
}
```

调度与 7.0 相同。校准失败（`fnOffsetCalib` 缺失或返回错误）会导致启动中止并进入 FAULT，快照 `wFaults` 置位，需 `motor_ClearFault()`。

### 7.3 场景三：有传感器直接闭环（Hall）

初始源与目标源为同一个 Hall 位置源：不使用开环角度发生器，启动后直接以传感器电角度闭环。Hall 适配器由库提供，产品只需给读码回调（见 5.5）。

```c
static foc_hall_t s_tHall;
static foc_hall_source_adapter_t s_tHallAdapter;
static foc_position_source_if_t s_tHallSource;

static uint8_t board_ReadHallCode(void *pHardwareContext)
{
    (void)pHardwareContext;
    return board_hall_read();            /* 经 MDI 读三相 Hall，0-7 */
}

static foc_result_t demo_InitHall(void)
{
    foc_hall_params_t tParams;

    foc_hall_DefaultParams(&tParams);
    /* 按实际接线修改 tParams.achSectorByCode。 */
    if (foc_hall_Init(&s_tHall, &tParams) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (foc_hall_source_Init(&s_tHallAdapter, &s_tHall,
                             NULL, board_ReadHallCode) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    s_tHallSource = foc_hall_PositionSourceInterface(&s_tHallAdapter);
    return FOC_RESULT_OK;
}

static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_SPEED,
    .ptInitialPositionSource = &s_tHallSource,
    .ptTargetPositionSource  = &s_tHallSource,  /* 同源：直接闭环 */
    .qSpeedReference = FOC_SCALAR(2.0f),        /* 机械 turn/s */
};

void demo_RunSensoredClosedLoop(void)
{
    if (demo_Init() != FOC_RESULT_OK ||
        demo_InitHall() != FOC_RESULT_OK) {
        return;
    }
    (void)demo_StartChecked(&s_tRunConfig);
    /* 速度环在低频步骤中生成 Iq 引用；运行中调速： */
    motor_SetSpeedReference(&s_tMotor, FOC_SCALAR(3.0f));
    /* 诊断：快照 eActiveSourceValidFlags 应含
       FOC_POSITION_VALID_ELECTRICAL_ANGLE/SPEED；
       Hall 无效码会使有效位清零并触发
       MOTOR_EVENT_SOURCE_VALIDITY_CHANGED，持续失效导致
       MOTOR_FAULT_POSITION_SOURCE 急停。 */
    demo_Stop();
}
```

调度与 7.0 相同。速度模式要求速度控制器绑定（7.0 骨架已含）；切换配置（`qTransition*` 字段）本场景不参与。速度引用单位是机械 turn/s，由 motor 按 `tPosition` 极对数换算内部电角量。

### 7.4 场景四：开环拖动到观测器接管

初始源为空、目标源为 SMO 观测器：先以开环角拖动，候选源在置信度、最小速度、方向一致、角误差受限且连续满足 `hwTransitionQualificationSamples` 个样本后，经 `hwTransitionBlendSamples` 个样本的最短路径融合接管角度。全程保持 D/Q 引用连续；资格或融合超时（`wTransitionTimeoutMs`）默认急停，不自动退回开环。

```c
static foc_smo_t s_tSmo;
static foc_position_source_if_t s_tSmoSource;

static foc_result_t demo_InitSmo(void)
{
    const foc_smo_params_t tParams = {
        .qModelGain       = FOC_SCALAR(0.5f),
        .qResistance      = FOC_SCALAR(0.1f),
        .qSlidingGain     = FOC_SCALAR(0.2f),
        .qBoundaryInverse = FOC_SCALAR(2.0f),
        .qEmfFilterAlpha  = FOC_SCALAR(0.1f),
        .qMinimumBemf     = FOC_SCALAR(0.01f),
    };

    if (foc_smo_Init(&s_tSmo, &tParams) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    s_tSmoSource = foc_smo_PositionSourceInterface(&s_tSmo);
    return FOC_RESULT_OK;
}

static motor_run_config_t s_tRunConfig = {
    .eControlMode = MOTOR_CONTROL_CURRENT,
    .ptInitialPositionSource = NULL,          /* 先开环拖动 */
    .ptTargetPositionSource  = &s_tSmoSource, /* 资格判定后接管 */
    .qInitialAngle  = FOC_ZERO,
    .qOpenLoopSpeed = FOC_SCALAR(1.0f),
    .qAcceleration  = FOC_SCALAR(5.0f),
    .tCurrentReference = {
        .qD = FOC_ZERO,
        .qQ = FOC_SCALAR(0.10f),
    },
};

void demo_RunOpenLoopToObserver(void)
{
    if (demo_Init() != FOC_RESULT_OK ||
        demo_InitSmo() != FOC_RESULT_OK) {
        return;
    }
    (void)demo_StartChecked(&s_tRunConfig);
    /* 诊断：快照跟踪切换全过程——
       eStartupPhase: QUALIFY_SOURCE -> BLEND_ANGLE -> COMPLETE；
       qBlendFactor 从 0 平滑到 1；qAngleError 为最短环绕误差；
       完成后 tActiveAngle 来自观测器。事件环依次出现
       TRANSITION_STARTED / TRANSITION_COMPLETED；
       超时则 TRANSITION_TIMEOUT + MOTOR_FAULT_TRANSITION_TIMEOUT。 */
    demo_Stop();
}
```

调度与 7.0 相同。本场景必须绑定 `motor_time_if_t`（超时判定），`motor_Start()` 会校验；目标为 SPEED/POSITION 模式时还要求外环控制器支持 `fnTrack` 预置，以便角度接管完成后无扰启用外环。

## 8. 统一位置源适配说明

### 8.1 接口与有效位

Hall、AB/ABZ、光栅、磁编码器和无感观测器统一实现：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

输入 `foc_position_input_t` 包含 α/β 电流、α/β 电压、采样周期（`qSamplePeriod`，秒）和毫秒时间戳；物理传感器可忽略电气量，观测器按需使用。输出 `foc_position_output_t` 的能力通过有效位声明：

| 有效位 | 含义 |
|---|---|
| `FOC_POSITION_VALID_ELECTRICAL_ANGLE` | `tElectricalAngle` 有效（环绕电角 turn） |
| `FOC_POSITION_VALID_ELECTRICAL_SPEED` | `qElectricalSpeed` 有效（电角 turn/s） |
| `FOC_POSITION_VALID_MECHANICAL_ANGLE` | `tMechanicalAngle` 有效（环绕机械 turn） |
| `FOC_POSITION_VALID_MECHANICAL_SPEED` | `qMechanicalSpeed` 有效（机械 turn/s） |
| `FOC_POSITION_VALID_MULTI_TURN` | `nMultiTurn` 多圈计数有效 |

输出还包含 `qConfidence`（`[0, 1]`，切换资格判定用，不是硬件安全状态）、`wFaults`（`FOC_POSITION_FAULT_*`，如 Hall 无效码、非法跳变）和 `wTimestamp`（毫秒，新鲜度判定用）。

### 8.2 各类源的有效位填法

| 源类型 | 应置有效位 | 说明 |
|---|---|---|
| Hall | `ELECTRICAL_ANGLE` + `ELECTRICAL_SPEED` | 天然电角（六步扇区）；无效码清有效位并报故障 |
| AB/ABZ 编码器 | `MECHANICAL_ANGLE` + `MECHANICAL_SPEED` | 只报机械量，电角由 motor 换算；Z 脉冲校准后可加 `MULTI_TURN` |
| 光栅 | 同 ABZ | 绝对多圈器件置 `MULTI_TURN` |
| 磁编码器 | `MECHANICAL_ANGLE`（+ `MECHANICAL_SPEED`） | 单圈器件不置 `MULTI_TURN` |
| 观测器（SMO/NLFO） | `ELECTRICAL_ANGLE` + `ELECTRICAL_SPEED` | 天然电角，附 `qConfidence` |

Hall 无效码 0/7 时清有效位并在 `wFaults` 置 `FOC_POSITION_FAULT_INVALID_DATA`。库内适配器：Hall 用 `foc_hall_source_Init()` + `foc_hall_PositionSourceInterface()`；SMO/NLFO 用 `foc_smo_PositionSourceInterface()` /`foc_nlfo_PositionSourceInterface()`。

### 8.3 机械角到电角转换的归属

极对数、方向、机械零位和电角偏移由 motor 统一管理，配置在 `motor_config_t.tPosition`：

```c
typedef struct {
    foc_angle_t tMechanicalZero;
    foc_angle_t tElectricalOffset;
    uint8_t chPolePairs;
    int8_t chDirection;      /* 仅 +1 或 -1 */
} foc_position_config_t;
```

规则：提供机械角的源（AB/ABZ、光栅、磁编码器）只填机械量，不要自行换算电角——motor 会对机械量应用方向、机械零位、极对数和电角偏移后生成电角并置电位标志；天然电角源（Hall、观测器）直接填电角并通过有效位标识，motor 不重复转换。SPEED/POSITION 外环消费的机械角/机械速度同样来自该统一换算。

### 8.4 故障与新鲜度

源每次 `fnStep` 必须更新 `wTimestamp`；motor 用毫秒时钟判定样本新鲜度。候选源资格不足时继续判定，超时急停；活动源失效（有效位丢失或 `wFaults` 非零）第一版直接以 `MOTOR_FAULT_POSITION_SOURCE` 急停，不自动后备。

### 8.5 MDI/HAL 边界

位置源适配器读取硬件必须经 MDI（或产品等价抽象），`foc/` 通用库不包含任何厂商头文件；芯片相关代码放 `peripheral/<chip>/` 或 `target/<chip>/`。适配器对象（如 `foc_hall_source_adapter_t`）和其 `pContext` 由产品静态持有，生命周期必须覆盖整个运行期。

## 9. 控制器和辅助控制算法

### 9.1 统一控制器接口

`foc_controller_if_t` 包含上下文、复位和单步函数：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);
} foc_controller_if_t;
```

现成适配器：

```c
foc_controller_if_t tPidIf = foc_controller_FromPid(&tPid);
foc_controller_if_t tLadrcIf =
    foc_ladrc_ControllerInterface(&tLadrc);
```

PID 和 LADRC 可直接绑定到 `motor_control_config_t`。SMC、STA 当前提供独立的 `Init/Reset/Step`，如需运行时绑定，应由产品写一个小型上下文适配器，不要修改电机控制核心。开环到闭环切换且目标为 SPEED/POSITION 时，外环控制器还需要 `fnTrack` 预置能力（`foc_controller_CanTrack()` 可查），以便角度接管后无扰启用外环。

### 9.2 模块概览

| 模块 | 主要接口 | 典型用途 |
|---|---|---|
| PID | `foc_pid_Init/Reset/Step` | 电流、速度、位置环 |
| LADRC | `foc_ladrc_Init/Reset/Step` | 带扩张状态观测的鲁棒控制 |
| SMC | `foc_smc_Init/Reset/Step` | 一阶滑模控制 |
| STA | `foc_sta_Init/Reset/Step` | 超螺旋二阶滑模控制 |
| LPF | `foc_lpf1_Init/Reset/Step` | 一阶低通 |
| Biquad | `foc_biquad_LowPassInit` | Butterworth、Chebyshev、Bessel |
| PLL | `foc_pll_Init/Reset/Step` | 角度跟踪、速度估计和锁定判断 |
| LTD | `foc_ltd_Init/Reset/Step` | 参考轨迹限速、限加速度 |
| DOB | `foc_dob_Init/Reset/Step` | 速度和负载扰动估计 |
| Feedforward | `foc_feedforward_Pmsm` | D/Q 交叉耦合与反电势前馈 |

所有包含运行状态的对象都应由对应电机实例单独持有。PID 参数已经离散化：`tKiTs` 包含采样周期，`tKdOverTs` 包含采样周期的倒数；改变控制周期后必须重新计算。`foc_gain_from_float()` 适合初始化阶段，定点产品可把最终参数固化为预构造增益，避免启动期浮点依赖。

## 10. 调制

通过 `motor_control_config_t.eModulation` 选择：

```c
MOTOR_MODULATION_SVPWM
MOTOR_MODULATION_SPWM
MOTOR_MODULATION_THIRD_HARMONIC
```

也可以直接调用：

```c
foc_ab_t tVoltage = { .qAlpha = FOC_SCALAR(0.2f),
                      .qBeta = FOC_SCALAR(0.1f) };
foc_duty_abc_t tDuty;

foc_svpwm(&tVoltage, &tDuty);
```

输出占空比会限制在 `[0, 1]`。板级 PWM 适配器仍需负责同步寄存器、互补输出和硬件死区。正常控制流程中调制由 motor 高频步骤内部调用，应用不直接调用调制函数。

## 11. 观测器与位置源实现

所有位置源统一实现第 8 章的 `foc_position_source_if_t`，由 motor 在高频步骤中调度；应用不再手动调用观测器 Step 或回写角度。

### 11.1 Hall

```c
foc_hall_params_t tParams;
foc_hall_t tHall;
foc_hall_source_adapter_t tAdapter;
foc_position_source_if_t tSource;

foc_hall_DefaultParams(&tParams);
foc_hall_Init(&tHall, &tParams);
foc_hall_source_Init(&tAdapter, &tHall, pHwContext, board_ReadHallCode);
tSource = foc_hall_PositionSourceInterface(&tAdapter);
```

Hall 码 0 和 7 无效。需要按实际接线修改 `achSectorByCode`，并验证正反方向的合法跳变序列。完整用法见 7.3。

### 11.2 SMO 与 NLFO

- `foc_smo_*`：基于静止坐标系电流模型和反电势的滑模观测器。
- `foc_nlfo_*`：非线性磁链观测器。

二者都需要归一化的电阻、电感、磁链、采样周期相关增益和有效性阈值，参数必须来自实际电机和控制周期。初始化后转换为统一位置源接口：

```c
tSource = foc_smo_PositionSourceInterface(&tSmo);
/* 或 */
tSource = foc_nlfo_PositionSourceInterface(&tNlfo);
```

完整用法（开环拖动后接管）见 7.4。

### 11.3 HFI

HFI 模块产生 D 轴高频注入并同步解调上一周期电流响应：

```c
foc_hfi_output_t tOutput;

foc_hfi_Step(&tHfi, qMeasuredCurrentD, &tOutput);
tVoltageReference.qD = foc_add_sat(tVoltageReference.qD,
                                   tOutput.qInjectionD);
```

`qPositionError` 可送入 PLL 或产品定义的角度校正环。只有 `bValid` 为真时才能使用响应。应用必须保证 ADC 样本确实对应上一次返回的注入载波。HFI 当前以算法模块形式提供，包装成 `foc_position_source_if_t` 由产品自行完成。

### 11.4 观测器间切换

`foc_observer_selector_t` 是两个统一位置源之间的资格检查和平滑切换器（适用于闭环运行中更换观测器的场景；motor 启动期的开环到闭环接管由 motor 内部完成，见 7.4）：

```c
foc_observer_selector_Init(&tSelector, &tParams, &tInitialSource);
foc_observer_selector_Request(&tSelector, &tTargetSource);
foc_observer_selector_Step(&tSelector, &tInput, &tOutput);
foc_observer_selector_Cancel(&tSelector);
```

切换阈值需要在真实转速变化和噪声条件下验证。

## 12. 优化和补偿算法

### 12.1 MTPA

```c
foc_scalar_t qIdReference;

foc_mtpa_Calculate(qFlux, qLd, qLq, qIqReference,
                    &qIdReference);
```

适用于具有凸极差异的电机。当 `Lq <= Ld` 时函数返回 0 D 轴参考。输入应使用一致的归一化基准。

### 12.2 弱磁

`foc_field_weakening_t` 内部持有一个电压 PID。速度未达到 `qBaseSpeed` 时返回 0；超过基速后，根据 D/Q 电压幅值生成受 `qMinimumId` 限制的负 D 轴电流。

```c
foc_scalar_t qWeakeningId = foc_field_weakening_Step(
    &tWeakening, qElectricalSpeed, &tVoltageDq);
```

产品需要决定如何与 MTPA 的 D 轴参考合成，并保证最终电流矢量不超过限流值。

### 12.3 死区和相位延迟

```c
foc_deadtime_Compensate(&tDeadtimeParams, &tPhaseCurrent, &tDuty);
foc_phase_delay_Compensate(tAngle, qSpeed, &tDelayGain,
                           qDirectionOffset, &tCompensatedAngle);
```

死区补偿按相电流方向修正占空比，并在零电流阈值内不动作。补偿量必须由实际 PWM 周期、死区和功率器件测试确定。

### 12.4 齿槽补偿

表格存储由产品负责，库不分配全局 1800 点数组：

```c
static const foc_scalar_t s_aqCogging[] = {
    FOC_ZERO, FOC_SCALAR(0.02f), FOC_ZERO, FOC_SCALAR(-0.02f),
};
foc_cogging_t tCogging;
foc_scalar_t qCompensation;

foc_cogging_Init(&tCogging, s_aqCogging,
                 (uint16_t)(sizeof(s_aqCogging) /
                            sizeof(s_aqCogging[0])));
foc_cogging_Get(&tCogging, tMechanicalAngle, &qCompensation);
```

查询使用周期线性插值。输入是机械角度，不是电角度。

## 13. 实验功能与硬件诊断

NSD 和参数辨识会主动给停止的电机施加电压，默认关闭：

```c
#define FOC_ENABLE_EXPERIMENTAL_NSD       0
#define FOC_ENABLE_EXPERIMENTAL_IDENTIFY  0
```

通过 Makefile 显式启用：

```powershell
.\make.bat TARGET_CHIP=at32f413 FOC_NUMERIC=fixed `
    FOC_EXPERIMENTAL_NSD=1 FOC_EXPERIMENTAL_IDENTIFY=1
```

启用并不代表允许直接上电。每个实验必须配置：

```c
foc_experiment_safety_t tSafety = {
    .qMaximumCurrent = FOC_SCALAR(0.2f),
    .qMaximumSpeed = FOC_SCALAR(0.02f),
    .qMinimumBusVoltage = FOC_SCALAR(0.2f),
    .qMaximumBusVoltage = FOC_ONE,
    .wTimeoutSamples = 1000U,
    .pContext = &s_tMotor,
    .fnEmergencyStop = app_ExperimentEmergencyStop,
};
```

其中急停函数必须直接关闭 PWM：

```c
static void app_ExperimentEmergencyStop(void *pContext)
{
    motor_EmergencyStop((motor_handle_t *)pContext,
                        MOTOR_FAULT_INVALID_COMMAND);
}
```

每次调用 `Start` 和 `Step` 都要提供最新 `foc_experiment_guard_t`。发生故障、过流、超速、母线越界或超时后，状态机返回 `FOC_RESULT_SAFETY`，输出清零并调用急停函数。

### 13.1 NSD

NSD 状态机依次经历稳定、正 D 轴偏置、零偏置、负 D 轴偏置和完成状态。应用把 HFI 解调响应传入 `foc_nsd_Step()`，并把输出的 `qVoltageD` 合入受限的 D 轴测试电压。

### 13.2 参数辨识

支持：

- `FOC_IDENTIFY_RS_LD_LQ`：电阻、D 轴电感、Q 轴电感。
- `FOC_IDENTIFY_FLUX`：在满足最小速度条件时辨识磁链。

状态机是非阻塞的，产品必须每个采样周期调用 `foc_identify_Step()`，应用其输出电压，并把实际电流和速度反馈给下一步。

`qInductanceTimeStep` 应预先包含采样周期与算法 `1/ln(20)` 的换算。辨识完成后先校验结果范围，再决定是否保存；不得自动覆盖量产参数。

### 13.3 硬件诊断输出

固定占空比直测、相序检查等危险测试放在可选的 `foc/diagnostic/motor_diagnostic.c`，默认不进入量产构建（`FOC_DIAGNOSTIC=1` 时编译，对应 `FOC_ENABLE_DIAGNOSTIC`）。诊断模块只通过门控 API `motor_DiagnosticSetOutput()` /`motor_DiagnosticStopOutput()` 操作，每次调用都强制校验 IDLE、无故障、占空比上限和累计输出时长；它不访问 motor 私有成员，正常生命周期 FSM 不含诊断专用状态。

## 14. 多电机

每个电机必须拥有独立的：

- `motor_handle_t`。
- PWM/ADC `pContext`。
- Id、Iq、速度、位置控制器对象。
- Hall/SMO/NLFO/HFI 位置源、PLL 和选择器对象。
- 弱磁、DOB、实验状态机等可变对象。

示意：

```c
static board_motor_hw_t s_tHwA;
static board_motor_hw_t s_tHwB;
static motor_handle_t s_tMotorA;
static motor_handle_t s_tMotorB;
static foc_pid_t s_tPidIdA;
static foc_pid_t s_tPidIdB;
```

可以共享只读参数和齿槽表，但不能让两个电机共享同一个 PID、观测器或 `foc_mdi_motor_context_t`。两个控制中断应各自调用自己实例的 `motor_HighFrequencyStep()`。`motor_handle_t` 之间没有共享可变状态，也不使用动态内存。

## 15. 参数整定顺序

建议按以下顺序进行，每一步都在低母线电压和硬件限流条件下验证：

1. 验证急停、硬件过流、PWM 极性和安全占空比。
2. 验证 ADC 触发点、三相零偏、电流方向和 Clarke 结果。
3. 验证编码器/Hall 电角度方向、极对数和零位。
4. 使用很小 D/Q 电压验证开环相序和 Park/反 Park 方向（场景 7.1）。
5. 在开环角度下整定 Id、Iq 电流环（场景 7.2）。
6. 整定速度环，再整定位置环（场景 7.3）。
7. 验证 SMO/NLFO 置信度与开环到闭环接管门限（场景 7.4）。
8. 最后启用前馈、MTPA、弱磁、死区和齿槽补偿。
9. NSD 和参数辨识只在专门测试流程中启用。

高级算法不能修复错误的电流方向、相序、角度方向或 ADC 时序。

## 16. 移植检查表

- [ ] 选择且只选择一个数值后端。
- [ ] `foc/` 未包含新平台的厂商头文件。
- [ ] `foc_pwm_if_t` 三回调齐全，占空比范围 `[0, 1]`。
- [ ] 正常使能和硬件急停是两条独立路径。
- [ ] `fnReconstruct` 采样点避开开关噪声和不可重构区。
- [ ] `fnOffsetCalib` 在无相电流条件下执行并置 `bIsCalibrated`。
- [ ] 电流重构输出使用统一 pu 基准和正确方向。
- [ ] `motor_sync_if_t` 成对绑定且中断安全（推荐 perfc_port 守卫）。
- [ ] `motor_time_if_t` 已绑定（有启动延时或开环到闭环切换时必需）。
- [ ] 高频 ISR 频率与 `qHighFrequencyPeriod` 一致，低频调度与 `qLowFrequencyPeriod` 一致。
- [ ] 位置源有效位、时间戳和故障位填写符合第 8 章约定。
- [ ] 机械角源不自行换算电角，`tPosition` 配置正确。
- [ ] 高频和低频控制器增益按各自周期离散化。
- [ ] 切换门限（置信度、最小速度、角误差、超时）经真实噪声验证。
- [ ] 所有故障路径最终调用功率级急停。
- [ ] 多电机没有共享可变算法对象或硬件上下文。
- [ ] 实验功能与硬件诊断在量产配置中保持关闭。

## 17. 构建和测试

项目根目录的 `Makefile` 和 `make.bat` 是权威入口。

AT32F413 浮点构建：

```powershell
.\make.bat clean TARGET_CHIP=at32f413
.\make.bat BUILD=debug TARGET_CHIP=at32f413 FOC_NUMERIC=float
```

AT32F413 定点构建：

```powershell
.\make.bat clean TARGET_CHIP=at32f413
.\make.bat BUILD=debug TARGET_CHIP=at32f413 FOC_NUMERIC=fixed
```

主机算法测试：

```powershell
Set-Location tests\foc
mingw32-make clean
mingw32-make all
```

`all` 依次执行：封装负面编译检查（证明 `motor_handle_t` 成员不可直接访问）、浮点测试套件、定点测试套件。测试环境可通过 `CC=<compiler>` 显式指定可用的 C11 编译器。


### 17.1 常规应用层控制指令集 (`motor`)

在应用层 (`foc_app.c`) 中，注册了标准的业务控制命令集 `motor`，用于正常运行、在线参数更新与快照查询：

#### 指令指南
- `motor start`：按应用层既定 `motor_run_config_t` 配置启动电机（进入 `STARTING` 自适应零偏校准与平滑启动）。
- `motor stop`：常规平滑软停机，使电机回归 `IDLE` 状态。
- `motor vq <volts-pu>`：在线实时更新开环 Vq 参考电压（例如 `motor vq 0.05`）。
- `motor pid <kp> <ki>`：在线动态整定 D/Q 轴电流环 PID 控制器的 $K_p$ 与 $K_i T_s$ 增益（例如 `motor pid 0.1 0.01`）。
- `motor status`：读取电机原生快照（`motor_GetSnapshot`），打印当前的运行状态、启动相位、控制模式、使能状态、故障字、实时电角度、三相 PWM 占空比、三相采样电流及 ADC 自适应偏置。
- `motor clear`：清除挂起的错误故障状态。


### 17.2 电机初步验证与调试模块 (`foc_verify`)

`experimental/foc_verify.h` / `foc_verify.c` 模块专为新硬件上电初期的相序定位、静态锁角、开环旋转提速与电流闭环测试而设计。

#### 1. 架构与设计规范 (高度可移植)
- **无状态与显式句柄 API**：模块内部**不保存任何侵入式 `static` 状态变量**，零反向依赖应用层句柄。所有 C 函数显式接受 `motor_handle_t *ptMotor` 电机句柄，保持绝对的高度可移植与模块化特性。
- **原生生命周期 FSM 兼容**：内部完全基于 Universal-FOC 原生的 `MOTOR_CONTROL_VOLTAGE_OPEN_LOOP` / `MOTOR_CONTROL_CURRENT` 配置与 `motor_Start()` 实现，由框架自动管理 25ms (512步) 自适应零偏校准与安全使能，零侵入高频 ISR，`HF_cycles` 保持极致低耗。
- **BUSY 状态防重入保护**：当电机已在运行状态中时，下发新验证命令会自动执行平滑 `motor_Stop()` 切换，避免返回 `FOC_RESULT_BUSY` (code 2)。

#### 2. 零代码侵入使能
- **只需要使能宏**：在 `foc_config.h` 中将 `FOC_ENABLE_MOTOR_VERIFY` 置为 `1` 即可：
  ```c
  #define FOC_ENABLE_MOTOR_VERIFY 1
  ```
- **无需修改 `foc_app.c`**：该模块使用 MODUS 框架的 `MODUS_SHELL_CMD` 宏自动注册命令。在 `foc_app.c` 中**无需添加任何初始化或调用代码**，应用层保持 100% 干净。

#### 3. C 语言 API

```c
/* 静态电角度锁定：指定电角度 turns (0.0~1.0) 与安全电压 Vq (pu, 限制 <= 0.05) */
foc_result_t foc_verify_StaticLock(motor_handle_t *ptMotor, float fTurns, float fVq);

/* 开环旋转强拖：指定旋转频率 (Hz) 与开环电压 Vq (pu, 限制 <= 0.05) */
foc_result_t foc_verify_OpenLoopRun(motor_handle_t *ptMotor, float fVoltageQ, float fSpeedHz);

/* 电流闭环测试：指定目标 Iq 电流 (pu, 限制 <= 0.05) 与开环拖动频率 (Hz) */
foc_result_t foc_verify_CurrentLoopRun(motor_handle_t *ptMotor, float fIqRef, float fSpeedHz);

/* 停止测试并平滑回归 IDLE 状态 */
foc_result_t foc_verify_Stop(motor_handle_t *ptMotor);
```

#### 4. MSHELL 控制台调试指令使用指南

在 RTT Shell 终端中可直接输入 `motor_verify` 及其子命令：

- **静态锁定指定电角度**：
  ```bash
  motor_verify static <turns> [vq=0.05]
  ```
  *说明*：`<turns>` 为电角度圈数（0.0 ~ 1.0），`[vq]` 为给定开环 Vq 电压 (pu，默认 0.05，最大钳位 0.05)。
  *示例*：`motor_verify static 0.667 0.03`（在 0.667 圈注入 0.03 pu 电压锁定）。

- **开环提速旋转强拖**：
  ```bash
  motor_verify run <speed-hz> [vq=0.05]
  ```
  *说明*：`<speed-hz>` 为开环电角度旋转频率 (Hz)，`[vq]` 为给定 Vq 电压。
  *示例*：`motor_verify run 2.0 0.05`（以 2.0 Hz 开环旋转）。

- **电流闭环测试**：
  ```bash
  motor_verify current <iq-pu> [speed-hz=2.0]
  ```
  *说明*：`<iq-pu>` 为目标 Iq 闭环参考电流 (pu，限制最大 0.05)，`[speed-hz]` 为拖动电频率 (Hz，默认 2.0)。
  *示例*：`motor_verify current 0.03 2.0`（电流环闭环运行在 0.03 pu Iq，开环 2 Hz 拖动）。

- **平滑停止测试**：
  ```bash
  motor_verify stop
  ```
  *说明*：使电机安全停机回归 `IDLE` 状态。

## 18. 常见问题

### 启动后立即过流

优先检查相序、电流方向、电角度方向、零位和 PWM 极性，不要先调 PID。

### 电流环输出正常但电机抖动

检查位置源给的是否为电角度、`tPosition` 极对数与方向是否正确，以及 ADC 样本与 PWM 周期是否对应。

### `motor_Start()` 返回 `FOC_RESULT_INVALID_ARGUMENT`

依次检查：当前是否在 IDLE（快照 `eRunState`）；故障位是否未清除；位置源组合是否合法（6.6）；所选模式要求的控制器是否都已绑定；开环到闭环切换是否缺毫秒时钟绑定。

### `motor_Start()` 返回 `FOC_RESULT_BUSY`

已有挂起命令未消化。等 `motor_RunFSM()` 处理后再下发，或用 `motor_Stop()` 改判。

### 切换一直停在资格判定相位

观察快照 `eCandidateSourceValidFlags`、`qAngleError` 和 `qTransitionMinimumConfidence/MinimumSpeed` 门限；`wTransitionTimeoutMs` 超时后默认急停是第一版的既定行为。

### 快照中某个源字段看起来是零

先检查对应的 `eActiveSourceValidFlags`/`eCandidateSourceValidFlags`；有效位未置位时源字段无意义。

### 定点结果和浮点略有差异

Q16.15 存在量化和饱和。检查基准值、增益范围和中间量是否合理，比较时使用与控制精度相符的容差。

### M0 上是否可以使用

高频核心路径支持不带 FPU 的 Cortex-M0。应选择 `FOC_NUMERIC=fixed`，避免在中断中调用 `foc_from_float()`，并使用目标编译器检查最终对象是否引入不希望的运行库 helper。

## 19. 延后扩展项

以下条目是有条件的扩展路线，不属于当前版本验收范围。这里只记录触发条件与实现方向；在条件满足前，不得为它们增加休眠枚举、状态或 API。

### 19.1 飞车启动

触发条件：产品需要在转子已旋转时接管。实现方向：在 STARTING 中增加速度和方向检测子流程，先资格判定位置源，再以当前估算速度预置角度发生器与控制器，最后执行无扰接管；不得复用静止启动的强制对齐流程。

### 19.2 多位置源管理器

触发条件：同一电机需要三个及以上位置源，或需要自动后备。实现方向：新增独立的 position manager，负责固定容量注册、优先级、健康度、交叉诊断和切换请求；motor 仍只消费 manager 输出，不在生命周期 FSM 中堆叠源类型分支。

### 19.3 闭环故障降级

触发条件：具体产品完成高速回切安全验证。实现方向：先同步开环发生器到当前闭环角度和速度，再反向执行角度融合，并对电流、速度和母线电压设置更严格限制。默认策略继续保持急停。

### 19.4 运行中控制模式切换

触发条件：产品确实需要 CURRENT/SPEED/POSITION 在线切换。实现方向：新增独立 handover 模块，为每一级外环提供输出跟踪或积分预置，按从内到外启用、从外到内退出；禁止直接修改模式枚举。

### 19.5 在线参数更新

触发条件：需要运行中整定。实现方向：按参数组提供版本化双缓冲，在明确的周期边界原子接管；拓扑、接口绑定和存储布局类参数仍只允许 IDLE 更新。

### 19.6 扩展诊断 FSM

触发条件：独立诊断模块出现三种以上共享流程。实现方向：复用安全门控、超时和限幅基础设施，将校准、固定 D/Q、电角度扫描等操作建成单独 FSM；不得让正常 motor FSM 增加诊断专用状态。

## 20. 公共接口索引

| 头文件 | 职责 |
|---|---|
| `math/foc_numeric.h` | 标量、饱和运算、除法、增益和 `foc_result_t` |
| `math/foc_angle.h` | 归一化角度、环绕、三角函数 |
| `middleware/foc_core.h` | Clarke、Park、反 Park |
| `hal/foc_hal.h` | PWM/ADC 统一包装与校验 |
| `motor/motor.h` | 电机对象公共 API：生命周期、命令、引用、实时步骤、快照、事件 |
| `motor/motor_types.h` | 句柄存储、配置、快照、事件、故障与状态枚举 |
| `motor/motor_position.h` | 统一位置源接口、有效位与转换配置 |
| `control/foc_controller.h` | 可绑定的标量控制器接口 |
| `control/foc_pid.h` | PID |
| `control/foc_ladrc.h` | LADRC |
| `control/foc_smc.h` | 一阶滑模控制 |
| `control/foc_sta.h` | 超螺旋滑模控制 |
| `control/foc_filter.h` | 一阶、二阶滤波 |
| `control/foc_pll.h` | 归一化角度 PLL |
| `control/foc_ltd.h` | 跟踪微分器 |
| `control/foc_dob.h` | 扰动观测器 |
| `control/foc_feedforward.h` | PMSM 解耦前馈 |
| `modulation/foc_modulation.h` | 三种调制算法 |
| `observer/foc_hall.h` | Hall 位置源与适配器 |
| `observer/foc_smo.h` | 滑模反电势观测器与适配器 |
| `observer/foc_nlfo.h` | 非线性磁链观测器与适配器 |
| `observer/foc_hfi.h` | 高频注入和同步解调 |
| `observer/foc_observer_selector.h` | 位置源间资格检查和平滑切换 |
| `optimization/foc_optimization.h` | MTPA、弱磁、死区、相位补偿 |
| `optimization/foc_cogging.h` | 外部齿槽补偿表 |
| `experimental/foc_experiment.h` | 实验安全契约 |
| `experimental/foc_nsd.h` | 转子极性检测状态机 |
| `experimental/foc_identify.h` | 电阻、电感、磁链辨识状态机 |
| `experimental/foc_verify.h` | 上电初试相序与开环调试验证例程 |
| `diagnostic/motor_diagnostic.h` | 硬件诊断输出（默认不进构建） |

`motor/motor_control.h`、`motor/motor_private.h` 和 `motor/motor_control_types.h` 是 motor 内部实现头，应用不应直接包含；公共声明以 `motor/motor.h` 为准。旧的 `middleware/observer_lib.h` 并行传感器抽象已删除，由 `motor/motor_position.h` 统一取代。

应用通常只需：

```c
#include "foc/foc.h"
```

移植层建议只包含所需的 HAL/MDI 头文件，减少不必要的依赖。
