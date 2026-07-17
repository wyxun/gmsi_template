# 通用 FOC 电机控制库

该目录提供一个与 MCU 架构无关、支持多电机实例的 PMSM/BLDC 矢量控制库。
核心算法同时支持浮点和 Q16.15 定点运算，业务代码只通过 FOC HAL 接口访问
PWM 与电流采样硬件。

本文以“把一个电机实例运行起来”为主线，说明库的分层、接口、调用周期、
算法选择和移植方法。本文中的数值仅用于说明 API，不能直接作为未知电机的
上电参数。

## 1. 能力与边界

当前库包含：

- Clarke、Park、反 Park 坐标变换。
- SVPWM、SPWM、三次谐波注入 SPWM。
- 电压开环、电流、速度和位置级联控制。
- PID、LADRC、SMC、STA、PLL、LTD、DOB、滤波和 PMSM 解耦前馈。
- Hall、SMO、NLFO、HFI 以及观测器平滑切换。
- MTPA、弱磁、死区、相位延迟和齿槽转矩补偿。
- 默认关闭的 NSD 转子极性检测和电机参数辨识。
- 浮点与 Q16.15 定点后端。
- 每个对象独立保存运行状态，可创建多个 `motor_handle_t`。

库不会替产品完成以下工作：

- 配置 PWM 定时器、互补输出、死区、刹车输入和 ADC 触发点。
- 决定电流采样电阻、运放增益、ADC 偏置和物理量归一化比例。
- 自动调度位置观测器。应用必须周期调用观测器并更新电机角度、速度。
- 自动整定电机参数或保护阈值。
- 替代硬件过流、过压和功率级急停保护。

主机测试和交叉编译只能证明接口与数值路径可构建，不能替代真实电机的限流
上电和波形验证。

## 2. 目录和依赖关系

```text
foc/
├── math/          数值后端、归一化角度、三角函数和平方根
├── middleware/    Clarke/Park 变换及兼容抽象
├── modulation/    SVPWM、SPWM、三次谐波调制
├── control/       控制器、滤波、PLL、DOB 和前馈
├── observer/      Hall、SMO、NLFO、HFI 和观测器选择器
├── optimization/  MTPA、弱磁、死区、相位和齿槽补偿
├── experimental/  NSD 和参数辨识安全状态机
├── hal/           平台无关的 PWM/ADC 接口
├── motor/         电机对象及高频、低频控制编排
├── app/           当前项目的应用示例，不属于通用移植必需层
├── foc_config.h   编译期配置
└── foc.h          对外统一头文件
```

推荐依赖方向：

```text
产品应用 -> motor / observer / optimization
         -> control / modulation / middleware
         -> math

产品应用 -> FOC HAL -> peripheral/<chip>/ MDI 适配 -> 芯片驱动
```

业务代码不应包含厂商 HAL 头文件。芯片相关代码放在
`peripheral/<chip>/` 或 `target/<chip>/`。

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

在代码中统一使用 `foc_scalar_t`。不要根据后端直接声明 `float` 或 `int32_t`
保存控制量。

### 3.2 Q16.15

定点后端使用带符号 32 位存储、15 位小数：

- `FOC_ONE` 表示 1.0。
- `FOC_HALF` 表示 0.5。
- `FOC_NEG_ONE` 表示 -1.0。
- `FOC_SCALAR(0.25f)` 生成后端对应的常量。
- `foc_from_float()` 和 `foc_to_float()` 用于非实时边界转换。

高频中断中应使用预先计算的 `foc_scalar_t` 和 `foc_gain_t`，避免反复进行
浮点转换。

### 3.3 归一化量

控制器和调制器使用归一化值。通常约定：

- 相电流、D/Q 电流：额定或允许峰值电流为 1 pu。
- D/Q 电压：可用母线电压或最大调制电压为 1 pu。
- 占空比：范围 `[0, 1]`。
- 速度：由产品定义基准速度，正负号表示方向。
- 置信度：范围 `[0, 1]`。

`motor_params_t` 中的毫欧、微亨等字段是电机元数据。当前基础控制编排不会
自动把这些物理量转换为控制器增益。

### 3.4 角度

`foc_angle_t` 使用“圈”而不是弧度：

- 0.0 表示 0°。
- 0.25 表示 90°。
- 0.5 表示 180°。
- 1.0 会环绕到 0.0。

常用接口：

```c
foc_angle_t angle = foc_angle_from_turns(0.25f);
foc_scalar_t sine = foc_angle_sin(angle);
foc_scalar_t delta = foc_angle_diff(target, actual);
angle = foc_angle_wrap(angle);
```

`motor_state_t.tThetaE` 是电角度。机械角转换为电角度时，需要乘以极对数并
环绕到一圈内。

### 3.5 增益

`foc_gain_t` 把整数和小数部分分开，可表达大于 1 的增益，同时保持统一的
定点实现。推荐只通过构造函数创建：

```c
foc_gain_t gain;

if (foc_gain_from_float(2.5f, &gain) != FOC_RESULT_OK) {
    /* 参数错误，禁止启动电机。 */
}
```

初始化函数会拒绝非法的增益小数部分。不要手工写入超出 `[-1, 1]` 的
`qFraction`。

## 4. 运行一个电机所需的对象

一个电机实例至少包含：

```text
motor_handle_t
├── motor_params_t          电机元数据
├── motor_state_t           角度、速度、电流、故障和运行状态
├── foc_hal_t
│   ├── foc_pwm_if_t        设置占空比、使能、紧急关闭
│   └── foc_adc_if_t        校准、原始采样、电流重构
├── motor_control_t         模式、参考值、变换结果和占空比
└── phase_current_handle_t  三相电流和 ADC 零偏
```

PID、LADRC、观测器等算法对象不放在 `motor_handle_t` 内自动创建。产品代码
持有这些对象，并通过接口结构绑定给电机。这样可以按产品需求选择算法，也
能保证多个电机之间没有共享的可变运行状态。

## 5. 实现 FOC HAL

### 5.1 PWM 接口

```c
typedef struct {
    void *pContext;
    foc_result_t (*fnSetDuty)(void *pContext,
                              foc_scalar_t qDutyU,
                              foc_scalar_t qDutyV,
                              foc_scalar_t qDutyW);
    foc_result_t (*fnEnable)(void *pContext, bool bEnable);
    void (*fnEmergencyStop)(void *pContext);
} foc_pwm_if_t;
```

约束：

- `fnSetDuty` 接收 `[0, 1]` 的三相占空比。
- `fnEnable(false)` 用于正常停机。
- `fnEmergencyStop` 必须立即关闭功率输出，不能只设置软件标志。
- 每个回调都接收自己的 `pContext`，禁止依赖全局“当前电机”。

### 5.2 ADC 接口

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
                                  phase_current_handle_t *ptCurrent);
} foc_adc_if_t;
```

其中 `fnReconstruct` 是电机初始化的必需回调，负责：

1. 读取本周期对应的 ADC/DMA 结果。
2. 减去 `ptCurrent->tCalib` 中的零偏。
3. 根据采样电阻和运放增益换算并归一化。
4. 按 1P、2P 或 3P 拓扑重构 `qIu/qIv/qIw`。
5. 检查结果有效性，异常时返回错误。

`fnStartConversion` 是否使用取决于板级时序。典型中心对齐 PWM 由定时器事件
触发 ADC，控制中断直接消费 DMA 结果，不需要软件启动转换。

### 5.3 最小板级接口示例

以下 `board_*` 函数由移植层实现，不是 FOC 库 API：

```c
#include "foc/foc.h"

typedef struct {
    void *pPwmDevice;
    void *pAdcDevice;
    uint32_t wPwmPeriod;
} board_motor_hw_t;

static foc_result_t board_SetDuty(void *pContext,
                                  foc_scalar_t qU,
                                  foc_scalar_t qV,
                                  foc_scalar_t qW)
{
    board_motor_hw_t *ptHw = (board_motor_hw_t *)pContext;

    /* 将 0..1 转成三个比较寄存器值并同步更新。 */
    return board_pwm_write(ptHw->pPwmDevice, ptHw->wPwmPeriod,
                           qU, qV, qW);
}

static foc_result_t board_Enable(void *pContext, bool bEnable)
{
    board_motor_hw_t *ptHw = (board_motor_hw_t *)pContext;
    return board_pwm_enable_outputs(ptHw->pPwmDevice, bEnable);
}

static void board_EmergencyStop(void *pContext)
{
    board_motor_hw_t *ptHw = (board_motor_hw_t *)pContext;
    board_pwm_force_off(ptHw->pPwmDevice);
}

static foc_result_t board_ReconstructCurrent(
    void *pContext,
    phase_current_handle_t *ptCurrent)
{
    board_motor_hw_t *ptHw = (board_motor_hw_t *)pContext;
    uint32_t wRawU;
    uint32_t wRawV;
    uint32_t wRawW;

    if (!board_adc_read_dma(ptHw->pAdcDevice,
                            &wRawU, &wRawV, &wRawW)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptCurrent->qIu = board_current_to_pu(
        wRawU, ptCurrent->tCalib.wOffsetU);
    ptCurrent->qIv = board_current_to_pu(
        wRawV, ptCurrent->tCalib.wOffsetV);
    ptCurrent->qIw = board_current_to_pu(
        wRawW, ptCurrent->tCalib.wOffsetW);
    return FOC_RESULT_OK;
}
```

当前仓库中的参考适配器：

- `peripheral/at32f413/foc_hal_mdi_adapter.c`
- `peripheral/stm32g431/foc_hal_mdi_adapter.c`

它们通过 MDI 访问硬件，不应把其中的厂商实现复制到通用业务层。

## 6. 初始化电机和控制器

### 6.1 创建 PID

PID 参数已经离散化：`tKiTs` 包含采样周期，`tKdOverTs` 包含采样周期的倒数。
改变控制周期后必须重新计算。

```c
static foc_pid_t s_tIdPid;
static foc_pid_t s_tIqPid;
static foc_pid_t s_tSpeedPid;

static foc_result_t app_InitPid(foc_pid_t *ptPid,
                                float fKp,
                                float fKiTs,
                                float fOutputLimit)
{
    foc_pid_params_t tParams = {
        .qOutputMinimum = foc_from_float(-fOutputLimit),
        .qOutputMaximum = foc_from_float(fOutputLimit),
        .qIntegratorMinimum = foc_from_float(-fOutputLimit),
        .qIntegratorMaximum = foc_from_float(fOutputLimit),
    };

    if (foc_gain_from_float(fKp, &tParams.tKp) != FOC_RESULT_OK ||
        foc_gain_from_float(fKiTs, &tParams.tKiTs) != FOC_RESULT_OK ||
        foc_gain_from_float(0.0f, &tParams.tKdOverTs) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return foc_pid_Init(ptPid, &tParams);
}
```

`foc_from_float()` 适合初始化阶段。定点产品可把最终参数固化为
`FOC_SCALAR()` 和预构造的增益，避免启动期浮点依赖。

### 6.2 构造电机配置

```c
static board_motor_hw_t s_tMotorHw;
static motor_handle_t s_tMotor;

static foc_result_t app_InitMotor(void)
{
    motor_config_t tConfig = {0};
    foc_result_t eResult;

    eResult = app_InitPid(&s_tIdPid, 0.4f, 0.02f, 0.8f);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = app_InitPid(&s_tIqPid, 0.4f, 0.02f, 0.8f);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = app_InitPid(&s_tSpeedPid, 0.2f, 0.01f, 0.8f);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }

    tConfig.tParams.chPolePairs = 4U;
    tConfig.eTopology = SENSING_TOPOLOGY_3P;

    tConfig.tHal.tPwm.pContext = &s_tMotorHw;
    tConfig.tHal.tPwm.fnSetDuty = board_SetDuty;
    tConfig.tHal.tPwm.fnEnable = board_Enable;
    tConfig.tHal.tPwm.fnEmergencyStop = board_EmergencyStop;

    tConfig.tHal.tAdc.pContext = &s_tMotorHw;
    tConfig.tHal.tAdc.fnReconstruct = board_ReconstructCurrent;
    /* 若需要 motor_CalibrateCurrent()，还必须设置 fnOffsetCalib。 */

    tConfig.tControl.tIdController =
        foc_controller_FromPid(&s_tIdPid);
    tConfig.tControl.tIqController =
        foc_controller_FromPid(&s_tIqPid);
    tConfig.tControl.tSpeedController =
        foc_controller_FromPid(&s_tSpeedPid);
    tConfig.tControl.eModulation = MOTOR_MODULATION_SVPWM;

    eResult = motor_Init(&s_tMotor, &tConfig);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }

    /* 必须由传感器或观测器提供有效电角度后再启动闭环。 */
    s_tMotor.tRt.tThetaE = foc_angle_from_scalar(FOC_ZERO);
    s_tMotor.tRt.qOmegaE = FOC_ZERO;
    return FOC_RESULT_OK;
}
```

`motor_Init()` 会验证：

- 极对数不为 0。
- 电流采样拓扑有效。
- PWM 的设置占空比、使能、急停回调存在。
- ADC 电流重构回调存在。

它不会自动校准 ADC，也不会自动启动 PWM。

### 6.3 电流零偏校准

如果适配器实现了 `fnOffsetCalib`：

```c
if (motor_CalibrateCurrent(&s_tMotor) != FOC_RESULT_OK ||
    !s_tMotor.tCurrent.tCalib.bIsCalibrated) {
    motor_EmergencyStop(&s_tMotor, MOTOR_FAULT_CURRENT_SAMPLE);
}
```

校准时必须保证 PWM 关闭、电机无相电流。若产品在生产阶段保存零偏，也必须
在启动前把有效值写入 `tCurrent.tCalib` 并设置 `bIsCalibrated`。

## 7. 控制模式

| 模式 | 启动枚举 | 必需控制器 | 高频环 | 低频环 |
|---|---|---|---|---|
| 电压开环 | `MOTOR_CONTROL_VOLTAGE_OPEN_LOOP` | 无 | 直接使用 D/Q 电压参考 | 无 |
| 电流闭环 | `MOTOR_CONTROL_CURRENT` | Id、Iq | D/Q 电流 | 无 |
| 速度闭环 | `MOTOR_CONTROL_SPEED` | Id、Iq、速度 | D/Q 电流 | 速度生成 Iq |
| 位置闭环 | `MOTOR_CONTROL_POSITION` | Id、Iq、速度、位置 | D/Q 电流 | 位置生成速度，再生成 Iq |

启动示例：

```c
motor_ControlSetCurrentReference(&s_tMotor, FOC_ZERO, FOC_ZERO);
motor_ControlSetSpeedReference(&s_tMotor, FOC_SCALAR(0.2f));

if (motor_ControlStart(&s_tMotor, MOTOR_CONTROL_SPEED) != FOC_RESULT_OK) {
    motor_EmergencyStop(&s_tMotor, MOTOR_FAULT_INVALID_COMMAND);
}
```

`motor_ControlStart()` 会复位绑定的控制器并使能 PWM。启动前应先把参考值设为
安全值，确认角度、采样和保护均有效。

正常停机：

```c
motor_ControlSetSpeedReference(&s_tMotor, FOC_ZERO);
/* 等待产品定义的减速条件满足。 */
motor_ControlStop(&s_tMotor);
```

故障停机：

```c
motor_EmergencyStop(&s_tMotor, MOTOR_FAULT_HARDWARE);
```

`motor_Reset()` 只清除运行状态，不应作为未经故障诊断的自动重新启动手段。

## 8. 周期调用关系

### 8.1 高频控制周期

高频步骤通常放在 PWM 更新或 ADC 注入转换完成中断中：

```c
void board_FocHighFrequencyInterrupt(void)
{
    if (s_tMotor.tRt.eRunState == MOTOR_STATE_OPEN_LOOP ||
        s_tMotor.tRt.eRunState == MOTOR_STATE_CLOSE_LOOP) {
        if (motor_ControlHighFrequencyStep(&s_tMotor) != FOC_RESULT_OK) {
            motor_EmergencyStop(&s_tMotor,
                                MOTOR_FAULT_INVALID_COMMAND);
        }
    }
}
```

该函数内部依次执行：

```text
电流重构 -> Clarke -> Park -> 电压/电流控制
         -> 反 Park -> 调制 -> 三相占空比写入
```

高频中断应固定周期、低抖动，并在下一次 PWM 更新前完成。日志、阻塞延时、
参数保存和复杂命令处理不得放入该中断。

### 8.2 角度和速度更新

`motor_ControlHighFrequencyStep()` 不会自动调用 `ptSensor` 或 `ptObserver`。
应用必须保证每次 Park 变换前，`tRt.tThetaE` 已包含可用电角度。

以通用观测器接口为例，可以在每个高频步骤结束后计算下一周期角度：

```c
static foc_observer_if_t s_tObserverIf;

static void app_UpdateObserver(void)
{
    foc_observer_input_t tInput = {
        .tCurrent = s_tMotor.tControl.tCurrentAlphaBeta,
        .tVoltage = s_tMotor.tControl.tVoltageAlphaBeta,
        .qEstimatedSpeed = s_tMotor.tRt.qOmegaE,
    };
    foc_observer_output_t tOutput;

    if (foc_observer_Step(&s_tObserverIf, &tInput, &tOutput) ==
            FOC_RESULT_OK &&
        tOutput.bValid) {
        s_tMotor.tRt.tThetaE = tOutput.tAngle;
        s_tMotor.tRt.qOmegaE = tOutput.qSpeed;
    }
}
```

实际 ISR 中可采用：

```c
motor_ControlHighFrequencyStep(&s_tMotor);
app_UpdateObserver(); /* 为下一周期提供角度。 */
```

首次闭环启动前仍需要已知初始角度、对齐流程、Hall 角度或开环切换策略。
不要在观测器尚未有效时直接依赖随机角度启动电流闭环。

`motor_AttachSensor()` 和 `motor_AttachObserver()` 只记录互斥的接口指针，不会
自动调度对应算法。

### 8.3 低频级联周期

速度和位置控制器不必按 PWM 频率运行：

```c
void board_FocLowFrequencyTick(void)
{
    if (s_tMotor.tRt.eRunState == MOTOR_STATE_CLOSE_LOOP) {
        if (motor_ControlLowFrequencyStep(&s_tMotor) != FOC_RESULT_OK) {
            motor_EmergencyStop(&s_tMotor,
                                MOTOR_FAULT_INVALID_COMMAND);
        }
    }
}
```

必须按低频周期重新离散化速度、位置控制器的积分和微分增益。

### 8.4 主循环

主循环适合：

- 接收命令并做范围检查。
- 更新速度、位置或转矩参考。
- 处理状态机和故障上报。
- 保存参数和执行非实时诊断。
- 在明确停机条件下启动实验算法。

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
foc_controller_if_t pidIf = foc_controller_FromPid(&pid);
foc_controller_if_t ladrcIf =
    foc_ladrc_ControllerInterface(&ladrc);
```

PID 和 LADRC 可直接绑定到 `motor_control_config_t`。SMC、STA 当前提供独立的
`Init/Reset/Step`，如需运行时绑定，应由产品写一个小型上下文适配器，不要
修改电机控制核心。

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

所有包含运行状态的对象都应由对应电机实例单独持有。

## 10. 调制

通过 `motor_control_config_t.eModulation` 选择：

```c
MOTOR_MODULATION_SVPWM
MOTOR_MODULATION_SPWM
MOTOR_MODULATION_THIRD_HARMONIC
```

也可以直接调用：

```c
foc_ab_t voltage = { .qAlpha = FOC_SCALAR(0.2f),
                     .qBeta = FOC_SCALAR(0.1f) };
foc_duty_abc_t duty;

foc_svpwm(&voltage, &duty);
```

输出占空比会限制在 `[0, 1]`。板级 PWM 适配器仍需负责同步寄存器、互补输出
和硬件死区。

## 11. 位置和速度观测器

### 11.1 通用接口

`foc_observer_if_t` 让上层在不依赖具体算法的情况下执行和切换观测器：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_observer_input_t *ptInput,
                           foc_observer_output_t *ptOutput);
} foc_observer_if_t;
```

输出只有在 `bValid == true` 时才可接管闭环角度。`qConfidence` 用于切换资格
判断，不是硬件安全状态。

### 11.2 Hall

```c
foc_hall_params_t params;
foc_hall_t hall;
foc_observer_output_t output;

foc_hall_DefaultParams(&params);
foc_hall_Init(&hall, &params);
foc_hall_Step(&hall, board_read_hall_code(), &output);
```

Hall 码 0 和 7 无效。需要按实际接线修改 `achSectorByCode`，并验证正反方向的
合法跳变序列。

### 11.3 SMO 与 NLFO

- `foc_smo_*`：基于静止坐标系电流模型和反电势的滑模观测器。
- `foc_nlfo_*`：非线性磁链观测器。

二者都需要归一化的电阻、电感、磁链、采样周期相关增益和有效性阈值。参数
必须来自实际电机和控制周期。

初始化后转换为通用接口：

```c
s_tObserverIf = foc_smo_ObserverInterface(&s_tSmo);
/* 或 */
s_tObserverIf = foc_nlfo_ObserverInterface(&s_tNlfo);
```

### 11.4 HFI

HFI 模块产生 D 轴高频注入并同步解调上一周期电流响应：

```c
foc_hfi_output_t output;

foc_hfi_Step(&hfi, measuredCurrentD, &output);
voltageReference.qD = foc_add_sat(voltageReference.qD,
                                  output.qInjectionD);
```

`qPositionError` 可送入 PLL 或产品定义的角度校正环。只有 `bValid` 为真时才能
使用响应。应用必须保证 ADC 样本确实对应上一次返回的注入载波。

### 11.5 平滑切换

`foc_observer_selector_t` 支持：

1. 保持当前观测器运行。
2. 请求目标观测器。
3. 检查目标置信度、速度和角度误差。
4. 连续稳定若干样本后开始角度混合。
5. 混合完成后切换活动接口。

主要接口：

```c
foc_observer_selector_Init(&selector, &params, &initialIf);
foc_observer_selector_Request(&selector, &targetIf);
foc_observer_selector_Step(&selector, &input, &output);
foc_observer_selector_Cancel(&selector);
```

切换阈值需要在真实转速变化和噪声条件下验证。

## 12. 优化和补偿算法

### 12.1 MTPA

```c
foc_scalar_t qIdReference;

foc_mtpa_Calculate(qFlux, qLd, qLq, qIqReference,
                    &qIdReference);
```

适用于具有凸极差异的电机。当 `Lq <= Ld` 时函数返回 0 D 轴参考。输入应使用
一致的归一化基准。

### 12.2 弱磁

`foc_field_weakening_t` 内部持有一个电压 PID。速度未达到 `qBaseSpeed` 时返回
0；超过基速后，根据 D/Q 电压幅值生成受 `qMinimumId` 限制的负 D 轴电流。

```c
foc_scalar_t qWeakeningId = foc_field_weakening_Step(
    &weakening, electricalSpeed, &voltageDq);
```

产品需要决定如何与 MTPA 的 D 轴参考合成，并保证最终电流矢量不超过限流值。

### 12.3 死区和相位延迟

```c
foc_deadtime_Compensate(&deadtimeParams, &phaseCurrent, &duty);
foc_phase_delay_Compensate(angle, speed, &delayGain,
                           directionOffset, &compensatedAngle);
```

死区补偿按相电流方向修正占空比，并在零电流阈值内不动作。补偿量必须由实际
PWM 周期、死区和功率器件测试确定。

### 12.4 齿槽补偿

表格存储由产品负责，库不分配全局 1800 点数组：

```c
static const foc_scalar_t s_aqCogging[] = {
    FOC_ZERO, FOC_SCALAR(0.02f), FOC_ZERO, FOC_SCALAR(-0.02f),
};
foc_cogging_t cogging;
foc_scalar_t compensation;

foc_cogging_Init(&cogging, s_aqCogging,
                 (uint16_t)(sizeof(s_aqCogging) /
                            sizeof(s_aqCogging[0])));
foc_cogging_Get(&cogging, mechanicalAngle, &compensation);
```

查询使用周期线性插值。输入是机械角度，不是电角度。

## 13. 实验功能与强制安全要求

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
foc_experiment_safety_t safety = {
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

每次调用 `Start` 和 `Step` 都要提供最新 `foc_experiment_guard_t`。发生故障、
过流、超速、母线越界或超时后，状态机返回 `FOC_RESULT_SAFETY`，输出清零并
调用急停函数。

### 13.1 NSD

NSD 状态机依次经历稳定、正 D 轴偏置、零偏置、负 D 轴偏置和完成状态。
应用把 HFI 解调响应传入 `foc_nsd_Step()`，并把输出的 `qVoltageD` 合入受限的
D 轴测试电压。

### 13.2 参数辨识

支持：

- `FOC_IDENTIFY_RS_LD_LQ`：电阻、D 轴电感、Q 轴电感。
- `FOC_IDENTIFY_FLUX`：在满足最小速度条件时辨识磁链。

状态机是非阻塞的，产品必须每个采样周期调用 `foc_identify_Step()`，应用其
输出电压，并把实际电流和速度反馈给下一步。

`qInductanceTimeStep` 应预先包含采样周期与算法 `1/ln(20)` 的换算。辨识完成
后先校验结果范围，再决定是否保存；不得自动覆盖量产参数。

## 14. 多电机

每个电机必须拥有独立的：

- `motor_handle_t`。
- PWM/ADC `pContext`。
- Id、Iq、速度、位置控制器对象。
- Hall/SMO/NLFO/HFI、PLL 和选择器对象。
- 弱磁、DOB、实验状态机等可变对象。

示意：

```c
static board_motor_hw_t s_tHwA;
static board_motor_hw_t s_tHwB;
static motor_handle_t s_tMotorA;
static motor_handle_t s_tMotorB;
static foc_pid_t s_tIdPidA;
static foc_pid_t s_tIdPidB;
```

可以共享只读参数和齿槽表，但不能让两个电机共享同一个 PID、观测器或
`foc_mdi_motor_context_t`。两个控制中断应调用各自的电机实例。

## 15. 错误处理

`foc_result_t` 包含：

| 返回值 | 含义 |
|---|---|
| `FOC_RESULT_OK` | 成功 |
| `FOC_RESULT_NULL` | 必需指针为空 |
| `FOC_RESULT_INVALID_ARGUMENT` | 配置、状态或枚举非法 |
| `FOC_RESULT_OUT_OF_RANGE` | 数值超出允许范围 |
| `FOC_RESULT_DIVIDE_BY_ZERO` | 除数为零 |
| `FOC_RESULT_DISABLED` | 实验功能未编译启用 |
| `FOC_RESULT_SAFETY` | 实验安全条件触发 |

原则：

- 所有 `Init` 返回值必须检查，任一失败都禁止启动 PWM。
- 高频采样、变换、调制或占空比写入失败时立即急停。
- `MOTOR_STATE_FAULT` 下不能再次使能，必须先诊断原因。
- 不要在中断中打印错误；记录故障码，在主循环中上报。

## 16. 参数整定顺序

建议按以下顺序进行，每一步都在低母线电压和硬件限流条件下验证：

1. 验证急停、硬件过流、PWM 极性和安全占空比。
2. 验证 ADC 触发点、三相零偏、电流方向和 Clarke 结果。
3. 验证编码器/Hall 电角度方向、极对数和零位。
4. 使用很小 D/Q 电压验证开环相序和 Park/反 Park 方向。
5. 整定 Id、Iq 电流环。
6. 整定速度环，再整定位置环。
7. 验证 SMO/NLFO/HFI 有效性与开环到闭环切换。
8. 最后启用前馈、MTPA、弱磁、死区和齿槽补偿。
9. NSD 和参数辨识只在专门测试流程中启用。

高级算法不能修复错误的电流方向、相序、角度方向或 ADC 时序。

## 17. 移植检查表

- [ ] 选择且只选择一个数值后端。
- [ ] `foc/` 未包含新平台的厂商头文件。
- [ ] PWM 三相占空比范围是 `[0, 1]`。
- [ ] 正常使能和硬件急停是两条独立路径。
- [ ] ADC 采样点避开开关噪声和不可重构区。
- [ ] 零偏校准在无相电流条件下执行。
- [ ] 电流重构输出使用统一 pu 基准和正确方向。
- [ ] 电角度使用归一化圈数并按极对数转换。
- [ ] 高频和低频控制器增益按各自周期离散化。
- [ ] 观测器在接管闭环前满足有效性和置信度要求。
- [ ] 所有故障路径最终调用功率级急停。
- [ ] 多电机没有共享可变算法对象或硬件上下文。
- [ ] 实验功能在量产配置中保持关闭。

## 18. 构建和测试

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

测试同时构建浮点和定点版本。测试环境可通过 `CC=<compiler>` 显式指定可用的
C11 编译器。

烧录和硬件调试必须使用项目规定入口。AI 调试统一使用 `tools/aitrace.exe`；
CPU halt、reset 或 GDB 操作前需要明确确认。

## 19. 常见问题

### 启动后立即过流

优先检查相序、电流方向、电角度方向、零位和 PWM 极性，不要先调 PID。

### 电流环输出正常但电机抖动

检查 Park 使用的是否为电角度、极对数是否正确，以及 ADC 样本与 PWM 周期
是否对应。

### `motor_ControlStart()` 返回参数错误

检查当前故障位，以及所选模式要求的 Id、Iq、速度、位置控制器是否都已绑定。

### 观测器对象已经 Attach，但角度没有变化

Attach 只保存接口指针。应用仍需调用 `foc_observer_Step()` 或具体观测器的
`Step()`，并更新 `motor.tRt.tThetaE/qOmegaE`。

### 定点结果和浮点略有差异

Q16.15 存在量化和饱和。检查基准值、增益范围和中间量是否合理，比较时使用
与控制精度相符的容差。

### M0 上是否可以使用

高频核心路径支持不带 FPU 的 Cortex-M0。应选择 `FOC_NUMERIC=fixed`，避免在
中断中调用 `foc_from_float()`，并使用目标编译器检查最终对象是否引入不希望
的运行库 helper。

## 20. 公共接口索引

| 头文件 | 职责 |
|---|---|
| `math/foc_numeric.h` | 标量、饱和运算、除法和增益 |
| `math/foc_angle.h` | 归一化角度、环绕、三角函数 |
| `middleware/foc_core.h` | Clarke、Park、反 Park |
| `hal/foc_hal.h` | PWM/ADC 统一包装与校验 |
| `motor/motor.h` | 电机初始化、采样、使能、急停 |
| `motor/motor_control.h` | 控制模式、参考值和周期步骤 |
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
| `observer/foc_observer.h` | 通用观测器接口 |
| `observer/foc_hall.h` | Hall 角度和速度 |
| `observer/foc_smo.h` | 滑模反电势观测器 |
| `observer/foc_nlfo.h` | 非线性磁链观测器 |
| `observer/foc_hfi.h` | 高频注入和同步解调 |
| `observer/foc_observer_selector.h` | 观测器资格检查和平滑切换 |
| `optimization/foc_optimization.h` | MTPA、弱磁、死区、相位补偿 |
| `optimization/foc_cogging.h` | 外部齿槽补偿表 |
| `experimental/foc_experiment.h` | 实验安全契约 |
| `experimental/foc_nsd.h` | 转子极性检测状态机 |
| `experimental/foc_identify.h` | 电阻、电感、磁链辨识状态机 |

应用通常只需：

```c
#include "foc/foc.h"
```

移植层建议只包含所需的 HAL/MDI 头文件，减少不必要的依赖。
