# Modus FOC 架构原理

本文档面向需要深入理解 FOC 库内部设计或对其进行二次开发的人员。阅读前提：已熟悉 [`foc/README.md`](README.md) 中的使用接口和移植流程。

## 1. 总体架构与模块分层

```
┌──────────────────────────────────────────────────┐
│  产品应用 (foc/app)                               │
│  按钮、Shell、日志、产品策略和调度                   │
├──────────────────────────────────────────────────┤
│  motor 公共 API (motor/motor.h)                   │
│  生命周期、命令、实时步骤、快照、事件环               │
├────────────┬────────────┬────────────┬───────────┤
│ observer   │ control    │ optimization│ modulation│
│ Hall/SMO/  │ PID/LADRC/ │ MTPA/弱磁/  │ SVPWM/    │
│ NLFO/HFI   │ SMC/STA/   │ 死区/齿槽   │ SPWM      │
│            │ PLL/DOB    │            │           │
├────────────┴────────────┴────────────┴───────────┤
│  middleware (Clarke/Park/IPark)                    │
├──────────────────────────────────────────────────┤
│  math (foc_scalar_t, BAM32 角度, 三角后端)         │
├──────────────────────────────────────────────────┤
│  HAL 抽象 (foc_pwm_if_t / foc_adc_if_t)           │
├──────────────────────────────────────────────────┤
│  MDI 适配 + 芯片驱动 (peripheral/<chip>/)          │
└──────────────────────────────────────────────────┘
```

职责划分原则：

- **上层不直接操作底层**：产品应用通过 `motor.h` 的公共 API 操作电机，不得访问 `tRt`、`tControl`、`tCurrent` 等私有成员，也不得直接调用 `foc_ipark()`、`foc_svpwm()` 或写入 PWM 占空比。
- **算法层不依赖硬件**：`foc/math`、`foc/middleware`、`foc/motor` 不包含任何厂商头文件、寄存器定义、芯片专用 DMA 或加速器配置。
- **芯片代码只在适配层**：所有芯片相关代码放在 `peripheral/<chip>/` 或 `target/<chip>/`，通过 MDI（Motor Driver Interface）绑定到 HAL 抽象。

### 内部头 vs 公共头

应用只需 `#include "foc/foc.h"`。以下头文件是 motor 内部实现，应用不应直接包含：

- `motor/motor_private.h` — 私有对象布局和内部 helper；
- `motor/motor_control.h` — 高低频控制内部入口；
- `motor/motor_control_types.h` — 控制模式枚举和运行时结构。

编译期有负面编译检查（`tests/foc` 的 encapsulation 目标）确保应用无法访问 `motor_handle_t` 的私有成员。

## 2. 数值后端：浮点 / 定点双轨设计

### 2.1 类型系统

整个库通过 `foc_scalar_t`（兼容别名 `q_type`）抽象数值类型：

```c
#if defined(FOC_NUMERIC_FLOAT)
typedef float foc_scalar_t;     /* 硬件 FPU 或软浮点 */
#elif defined(FOC_NUMERIC_FIXED)
typedef int32_t foc_scalar_t;   /* Q16.15 定点 */
#endif
```

所有控制算法（PID、LADRC、SMO、Park/Clarke、调制）都基于 `foc_scalar_t` 编写唯一一份代码。编译期通过 `FOC_NUMERIC_FLOAT` / `FOC_NUMERIC_FIXED` 宏选择后端，构建时二者必须且只能选一个（`foc_config.h` 有编译期断言）。

浮点后端直接映射为硬件 `float` 运算；定点后端利用 `foc_mul_pu()`、`foc_mul_wide()` 等内联函数处理 Q16.15 乘法和饱和算术，关键路径中不使用 `foc_from_float()` / `foc_to_float()` 以避免运行时转换。

### 2.2 增益抽象

`foc_gain_t` 将增益拆分为整数部分（`int16_t`）和小数部分（`foc_scalar_t`）：

- 可表达大于 1 的增益（浮点天然支持，定点通过整数部分实现）；
- 浮点和定点后端的控制器代码完全一致；
- 初始化阶段通过 `foc_gain_from_float()` 构造，运行阶段通过 `foc_gain_apply()` 使用，避免运行期浮点依赖。

### 2.3 主机测试守卫

`tests/foc/Makefile` 同时构建 float 和 fixed 两个目标。任何算法改动必须通过双后端测试矩阵（encapsulation / float / fixed / strict-alias），确保数值后端之间的行为一致性。定点与浮点之间存在 Q16.15 固有的量化和舍入差异，测试容差根据后端分别设置。

## 3. 角度模型：BAM32

### 3.1 设计动机

第一版角度用 `float`（圈数）存储，每次 wrap 需要 `floorf()` 调用，每次 sin/cos 还需要范围归约。V2.1 引入 BAM32（Binary Angle Measurement，32-bit），将电角度存储为 `uint32_t`，0x00000000 到 0xFFFFFFFF 表示 0.0 到 1.0 圈（即将溢出前为 1.0-ε 圈）。

### 3.2 关键性质

```c
typedef struct {
    uint32_t wBam32; /* 0 到 2^32-1 表示 0.0 到接近 1.0 圈 */
} foc_angle_t;
```

- **零开销 wrap**：`foc_angle_wrap()` 直接返回原值——32-bit 无符号整数加法自然溢出即等价于模 2^32 圈环绕，无需任何比较或分支。
- **角度差**：`target - actual` 的 32-bit 无符号差，高位判断正负方向，自然给出最短路径环绕误差。
- **三角函数索引**：BAM32 的高位直接用作 LUT 索引和 CORDIC 参数，无需浮点乘法换算。
- **跨平台可移植**：模 2^32 加法、有符号差解释和位移索引全部是标准 C 操作，不依赖特定 MCU 或编译器特性。

### 3.3 速度积分边界

速度（turn/s）乘以采样周期（s）得到增量圈数，在积分时通过 `foc_angle_from_scalar()` 转换为 BAM32 增量，然后用 `foc_angle_add()` 自然溢出累加。浮点速度 × 采样周期的乘积只需在积分边界进行一次 BAM32 转换，高频路径中的 wrap 操作变为整数加法。

## 4. 三角后端三层架构

### 4.1 问题背景

float 后端的 `sinf()` / `cosf()` / `atan2f()` 是高频路径最大的单一开销来源。20 kHz 下每步 4 次 `sinf()` 仅此一项就超过 1100 cycles（M4F）。fixed 后端从一开始就是自研实现——这证明自研三角后端是本库的既定路线。V1/V2 优化将 float 后端补齐，形成三层可切换架构。

### 4.2 后端分派

```c
#define FOC_TRIG_BACKEND_LIBM    0  /* 参考实现，仅用于精度对照 */
#define FOC_TRIG_BACKEND_LUT     1  /* 通用软件后端（默认） */
#define FOC_TRIG_BACKEND_CORDIC  2  /* STM32G4 硬件后端 */
```

- **LIBM**：调用标准 `sinf`/`cosf`/`atan2f`，仅用于 host 测试精度对照，固件不启用。
- **LUT**（默认）：513 项 1/4 波正弦查找表 + 线性插值，最大绝对误差 < 1.2e-6（满足 1e-5 测试容差并留 5 倍裕量）。象限对称性同时提供 sin/cos，无需第二张表。host（x64）、AT32 和所有无硬件三角加速器的芯片通用。
- **CORDIC**：STM32G431 的 CORDIC 协处理器寄存器级驱动。一次写寄存器同时输出 sin 和 cos，~20–40 cycles。驱动代码仅在 `peripheral/stm32g431/halcordic.c` 中，FOC 内核零 vendor 依赖。

固定后端 `foc_trig_backend_t` 通过 vtable 在编译期静态绑定，无运行时分支开销。LUT 后端的 `fnSinCosBam32` 利用 BAM32 高位直接索引 + 象限分解，一次调用同时输出 sin 和 cos，比两次独立调用省一次 wrap 和象限分解。

### 4.3 Park/IPark 单次计算编排

高频步对同一个电角度做 Park 和反 Park 需要 2×(sin+cos)。L0 优化引入 `foc_park_cached()` / `foc_ipark_cached()`，接收已计算的 sin/cos 值。motor 高频步预先调用一次 `foc_angle_sincos()`，然后复用结果到 cached 变换。此编排与后端无关——即使 libm 后端也能省掉 2 次三角调用。

## 5. 不透明对象与封装

### 5.1 静态句柄

```c
typedef union motor_handle_u {
    max_align_t tAlignment;
    uint8_t achPrivate[MOTOR_HANDLE_STORAGE_SIZE];
} motor_handle_t;
```

`motor_handle_t` 对外只暴露固定大小的字节数组。内部实现类型 `motor_impl_t` 仅存在于 `motor_private.h`，应用代码无法包含它。通过 `__attribute__((__may_alias__))` 实现私有别名访问，在保持标准 C 严格别名规则的同时允许 motor 内部以 lvalue 访问成员。

编译期断言保证：
- `sizeof(motor_impl_t)` ≤ `MOTOR_HANDLE_STORAGE_SIZE`（当前 768/832 字节，取决于是否启用 Profile）；
- `motor_handle_t` 的对齐满足 `motor_impl_t` 的对齐要求。

不使用 `malloc`。每个电机实例都是栈上或静态存储区的 `motor_handle_t`。

### 5.2 私有成员布局

`motor_impl_t` 按对齐要求排序字段：先放指针和含指针的结构（8 字节对齐）、再放 4 字节成员、2 字节成员、1 字节成员和位域。枚举状态压缩到 `uint8_t`；不取地址的标志位用位域打包。此布局在 32-bit MCU 和 64-bit host 上均保持一致。

### 5.3 多实例隔离

每个电机实例独立持有：

- HAL 绑定（PWM/ADC 的 `pContext`）；
- 控制器对象（Id/Iq/速度/位置 PID 或 LADRC）；
- 位置源（Hall/SMO/NLFO 适配器）；
- 弱磁、DOB、实验状态机等可变对象。

`motor_handle_t` 之间没有共享可变状态。两个控制中断各自调用自己实例的 `motor_HighFrequencyStep()`。只读参数和齿槽表可以共享，但 PID、观测器和 MDI 上下文不可共享。

## 6. 生命周期 FSM 与控制模型正交分离

### 6.1 三个正交维度

不能用一个"开环/闭环"枚举同时表示生命周期状态、控制模式和角度来源：

- **生命周期**（`motor_state_e`）：IDLE → STARTING → RUNNING / STOPPING → IDLE，以及从任意状态 → FAULT；
- **控制模式**（`motor_control_mode_e`）：VOLTAGE_OPEN_LOOP / CURRENT / SPEED / POSITION；
- **角度来源**：零个（内部开环发生器）、一个（直接闭环）、两个（开环拖动后切入目标源）。

三者独立变化。例如"开环角度 + D/Q 电流闭环"是合法的运行组合：角度来自内部发生器，但电流仍在闭环。

### 6.2 命令邮箱模型

普通 API（`motor_Start` / `motor_Stop`）不直接修改状态，只把校验后的命令放入内部邮箱并立即返回。状态迁移由 `motor_RunFSM()` 在主循环中执行。`motor_EmergencyStop()` 是唯一绕过 FSM、立即关闭功率输出的公共接口。

设计约束：
- 不提供 `motor_SetState()`；
- 命令被拒绝只产生事件，不进入故障；
- 故障只由真实危险路径设置。

### 6.3 启动子相位

启动过程被分解为多个子相位（`motor_startup_phase_e`），由 `motor_RunFSM()` 驱动：

```text
IDLE → CALIBRATE → WAIT_DELAY → ENABLE
→ QUALIFY_SOURCE → BLEND_ANGLE → COMPLETE (→ RUNNING)
```

每个相位有明确的进入条件、超时和失败路径。例如校准失败直接进入 FAULT，资格判定超时默认急停（第一版不自动退回开环）。

## 7. 高频 / 低频双频控制架构

### 7.1 频率分离

| 步骤 | 调用频率 | 调用上下文 | 职责 |
|---|---|---|---|
| `motor_HighFrequencyStep` | PWM 载波频率（如 20 kHz） | ADC/PWM 中断 | 电流采样、位置源更新、角度融合、Clarke/Park、电流环、反 Park、调制、PWM 输出 |
| `motor_LowFrequencyStep` | ms 级（如 1 kHz） | 系统调度 | 位置环 → 速度引用，速度环 → Iq 引用 |
| `motor_RunFSM` | 每轮主循环 | 线程上下文 | 生命周期迁移、事件处理、故障日志 |

控制周期必须与配置中的 `qHighFrequencyPeriod` / `qLowFrequencyPeriod` 一致。周期失配会导致转速和控制周期错误。

### 7.2 高频步骤数据流与 Fast Path 内核 (V4)

```text
+--------------------------------------------------------------------+
| 阶段 0: 前置参数校验与局部只读快照提取 (Entry Check & Snapshot)       |
|   - 校验初始化状态与重入标志 bHighFrequencyStepInProgress            |
|   - 校验运行状态 (STARTING/RUNNING) 与 PWM 使能位                    |
|   - 临界区提取 command/plan 只读快照至栈帧临时变量，快速解锁             |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 1: 批量相电流采样 (plan.tIo.fnSampleCurrent)                    |
|   - 直调底层 ADC/DMA 回调读取相电流，完成零偏扣除与归一化 -> (Iu,Iv,Iw)  |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 2: Clarke 变换与位置源步进 (foc_clarke & plan.fnSourceStep)     |
|   - Clarke 变换: (Iu, Iv, Iw) -> α-β 静止坐标系电流 (Iα, Iβ)         |
|   - 调度位置源 fnStep 计算当前电角度/电速度并应用极对数与零位平移       |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 3: 源切换管理与角度混合过渡 (Qualification & Blending)          |
|   - 资格判定: 检查观察器置信度、转速与角度误差是否持续 N 拍合格       |
|   - 角度混合: foc_position_Blend 在开环角与闭环角间按比例加权平滑接管    |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 4: 算法核心 (Park -> PI 电流调节 -> IPark -> 调制)              |
|   - foc_angle_sincos: 基于 BAM32 计算成对正余弦 (LUT / CORDIC)     |
|   - foc_park_cached: 复用正余弦，将 (Iα, Iβ) 转为 (Id, Iq) 反馈       |
|   - D/Q 电流闭环: 调度 plan.tId.fnStep / plan.tIq.fnStep 生成 (Vd, Vq)|
|   - foc_ipark_cached: 将 (Vd, Vq) 解旋还原为 (Vα, Vβ) 静止轴电压     |
|   - plan.fnModulate: 调度 SVPWM/SPWM 调制算法计算三相占空比          |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 5: 软停机与并发拦截点 (Stop Interlock Check)                    |
|   - 检查是否有主循环 MOTOR_COMMAND_STOP 挂起，若有则放弃本拍提交    |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 6: 占空比硬件提交 (plan.tIo.fnCommitDuty)                       |
|   - 调度直写回调一次性写入 TIMx 预装载寄存器                         |
+--------------------------------------------------------------------+
                                  │
                                  ▼
+--------------------------------------------------------------------+
| 阶段 7: 状态发布、延迟事件入槽与 Profile 写回                         |
|   - 将本拍最终电角度、速度、dq 电压电流写回 state                   |
|   - 调用 motor_hf_post_event 写入 4 槽无锁延迟队列 aPendingEvents[4] |
|   - 记录 Profile 快照，解开重入标记，退出                            |
+--------------------------------------------------------------------+
```

#### V4 Fast Path 核心设计原则：

1. **慢路径预解析 (`motor_hf_plan_t`)**：
   在 `motor_Init()` 与 `motor_Start()` 绑定阶段预先完成所有接口有效性、ABI 版本、回调非空校验，填充为只读的 `motor_hf_plan_t`。高频 20 kHz ISR 直接调度缓存的回调指针，去除了二次服务层转发与重复校验。
2. **锁隔离与栈帧缓存 (`motor_hf_frame_t`)**：
   ISR 仅在入口与出口极短暂加锁提取/写回快照，核心算法计算全过程运行在栈帧 `frame` 上，消除了中间锁竞争与内存总线等待。
3. **无锁延迟事件队列 (`aPendingEvents[4]`)**：
   观察性状态变更（有效性改变、启动相位过渡）暂存至 ISR 4 槽无锁延迟队列，由低频调度 `motor_LowFrequencyStep()` 统一出列写入事件环，彻底解决了 ISR 竞争事件环锁导致的周期性耗时抖动。
4. **单出口硬件急停 (`motor_hf_fault()`)**：
   硬件采样、计算或提交异常统一走唯一故障出口，在保护锁下写故障字并立即触发底层 `fnEmergencyStop` 切断 PWM 驱动。

### 7.3 引用值原子性

`motor_Set*Reference()` 四个 setter 在临界区内原子写入，下一控制周期生效。避免了主循环与 ISR 之间的撕裂读取，无需锁或双重缓冲。

## 8. 统一位置源接口

### 8.1 接口设计

所有位置源——物理传感器（Hall、AB/ABZ 编码器、光栅、磁编码器）和无感观测器（SMO、NLFO、HFI）——统一实现 `foc_position_source_if_t`：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(void *pContext,
                           const foc_position_input_t *ptInput,
                           foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

统一输入（`foc_position_input_t`）包含 α/β 电流、α/β 电压、采样周期和毫秒时间戳。物理传感器可忽略电气量，观测器按需使用。

### 8.2 有效位与能力声明

统一输出（`foc_position_output_t`）通过有效位声明本周期哪些字段有意义：电角度、电角速度、机械角、机械速度、多圈计数、置信度和故障位。motor 根据有效位决定控制角度来源和故障响应，而不关心源的具体类型。

### 8.3 机械 ↔ 电转换的归属

极对数、方向、机械零位和电角偏移由 motor 统一管理（`foc_position_config_t`），不由位置源自行换算：

- 提供机械角的源（AB/ABZ、光栅、磁编码器）只填机械量。motor 对机械量应用方向、机械零位、极对数和电角偏移后生成电角并置电位标志。
- 天然电角源（Hall、观测器）直接填电角并通过有效位标识，motor 不重复转换。

### 8.4 资格判定与最短路径融合

开环到闭环切换时，候选源必须在以下条件连续满足 `hwTransitionQualificationSamples` 个样本后才允许接管：

- 输出有效且时间戳未过期；
- 置信度 ≥ `qTransitionMinimumConfidence`；
- 速度 ≥ `qTransitionMinimumSpeed` 且方向一致；
- 与开环角的最短环绕误差 ≤ `qTransitionMaximumAngleError`。

融合期间角度按最短路径从开环角平滑过渡到候选源角度（`qBlendFactor` 从 0 到 1），保持 D/Q 引用连续。融合完成后可选启用外环（速度/位置）。

## 9. 跨架构兼容策略

### 9.1 零厂商头文件原则

`foc/` 通用库目录中不包含任何 MCU 厂商头文件。这确保了：

- 库可以在任意 Cortex-M / RISC-V 芯片上编译；
- host（x64）测试可以直接编译同一份算法代码，无需模拟 MCU 外设；
- 新平台移植只需实现 HAL 接口，无需修改算法代码。

芯片相关代码严格放在 `peripheral/<chip>/` 或 `target/<chip>/`。

### 9.2 临界区多架构适配

`motor_sync_if_t` 在不同架构上通过 perf_counter 的架构守卫实现：

- **Cortex-M**：`mrs primask` / `cpsid i` / `msr primask`；
- **RISC-V**：`csrrc/csrrs mstatus, MIE`；
- **其他架构**：空实现（支持纯单线程主机测试）。

推荐使用 `perfc_port_disable_global_interrupt()` / `perfc_port_resume_global_interrupt()`，这些函数在 `modus/src/arch/perfc_port.h` 中根据架构宏编译时选择，不含厂商头文件。

### 9.3 三角后端跨平台

- **host（x64）测试**：默认 LUT，与固件运行的同一条数值路径；TRIG=libm 对照目标用于逐点精度验证；
- **STM32G431**：CORDIC 硬件后端（`target/stm32g431/target.mk` 设 `FOC_TRIG_BACKEND=CORDIC`）；
- **AT32F413 / AT32F407 / CH592 等**：LUT 软件后端，不引入任何新代码；
- **所有平台**：CORDIC 失效时回退到 LUT 零成本（编译期切换），FOC 内核零改动。

### 9.4 Cortex-M0 支持

高频核心路径支持不带 FPU 的 Cortex-M0。应选择 `FOC_NUMERIC=fixed`，避免在中断中调用 `foc_from_float()`。使用目标编译器检查最终对象是否引入不希望的浮点运行库 helper。

### 9.5 RISC-V 支持

RISC-V（如 CH592）运行在定点后端 + LUT 三角后端 + RISC-V 临界区守卫上。部分 RISC-V 芯片的 `mcycle` CSR 未实现硬件周期计数器，此时 perf_counter 使用 SysTick + 软件溢出计数器的 64 位方案（`perfc_port.c` 中 `get_riscv_systick_elapsed64()`）。

## 10. 实时性能与 Profile 系统

### 10.1 性能基线

STM32G431（170 MHz，20 kHz 控制周期，8500 cycles 预算），实测数据：

| 阶段 | V0（优化前） | V1（LUT + BAM32 + cached Park/IPark） | V2.1（零开销 Profile） | **V4 Fast Path（重构后实测）** |
|---|---|---|---|---|
| 开环总 cycles | ~7,442 (87.5%) | ~5,241 (61.7%) | 3,588 (42.2%) | **2,315–2,322 (27.2%)** |
| 电流闭环总 cycles | 未测（libm 溢出风险） | ~5,200 (61.2%) | 4,306–4,378 (51.5%) | **3,117–3,128 (36.6%)** |
| 主要手段 | libm sinf×4 + floorf | 513 项 LUT + BAM32 角度 + Park/IPark 单次计算 | 对象化 Profile、DWT 架构地址直读 | 硬件直写契约、绑定期预解析计划、延迟事件槽、栈帧锁隔离 |

V2.1 电流闭环分阶段实测（4306 cycles）：

| 阶段 | cycles |
|---|---|
| sample（三相电流采样/校准） | 687 |
| clarke（Clarke 变换） | 128 |
| park（Park 变换） | 172 |
| ipark（反 Park 变换） | 165 |
| modulate（SVPWM 调制） | 430 |
| commit（状态更新与 PWM 提交） | 879 |
| 其他（控制器 + 编排逻辑） | ~1,845 |

V2.1 的实现不改变控制周期和硬件安全行为：BAM32 角度模型让 wrap 变成 uint32 自然溢出，LUT 后端复用同一索引路径同时出 sin/cos，DWT 直读用单条 `LDR` 替代 noinline 函数调用。

### 10.2 零开销 Profile（V2.1 P0）

Profile 系统（`foc_hf_profile.h`）使用三级编译期门控：

- `FOC_HF_PROFILE_LEVEL = 0`：所有 profile 宏编译为空，零 DWT 读取，零全局变量，量产构建使用；
- `LEVEL = 1`：仅测量整环总 cycles；
- `LEVEL = 2`：测量各阶段分段 cycles（sample / clarke / park / controller / ipark / modulate / commit）。

Profile 数据存储在每个 motor 实例的私有字段中（`motor_hf_profile_snapshot_t`），不再是全局变量。高频步内部使用栈上局部变量记录各阶段 cycles，在步骤末尾一次性写回私有快照，消除了中间多次写全局 `volatile` 的总线开销。关闭 Profile 时 `MOTOR_HANDLE_STORAGE_SIZE` 自动缩减，无 RAM 浪费。

### 10.3 DWT 直读 vs get_system_ticks

`foc_hf_profile.h:13-26` 直接通过 CoreSight 架构地址 `0xE0001004` 读取 `DWT->CYCCNT`，而不是通过 CMSIS `DWT` 符号。原因：业务代码不包含 CMSIS 设备头文件，`DWT` 符号不可见时编译器会静默回退到 `get_system_ticks()`——一个 noinline 函数调用，开销数百 cycles。在 `FOC_HF_PROFILE_LEVEL=2` 下该回退每步执行 14 次，足以把 20 kHz ISR 推到 PWM 周期之外。单条 `LDR` 从固定架构地址读取，开销可忽略。

## 11. 模块间依赖与关键文件

### 11.1 核心实现文件

| 文件 | 职责 |
|---|---|
| `motor/motor.c` | Init/Reset、引用值原子写入、快照、事件环读写、故障清除 |
| `motor/motor_control.c` | `motor_HighFrequencyStep` / `motor_LowFrequencyStep` 完整编排、调制分派、角度控制 |
| `motor/motor_fsm.c` | `motor_RunFSM` 生命周期状态机、命令消费、启动子相位 |
| `motor/motor_position.c` | 位置源校验、机械→电转换、资格判定、最短路径角度融合 |
| `motor/motor_private.h` | `motor_impl_t` 布局、内部 helper 宏、事件记录格式 |
| `motor/motor_profile.h` | Profile 快照结构和读取 API |

### 11.2 事件环内部格式

事件环内部使用紧凑的 8 字节记录（`motor_event_record_t`）：

```c
typedef struct {
    uint32_t wSequence;    /* 独立单调序号 */
    uint16_t hwPayload;    /* 事件专用数据（故障位/命令码/新旧标志） */
    uint8_t chType;        /* 事件类型枚举 */
    uint8_t chMeta;        /* 打包的 from-state(3b) | to-state(3b) | detail(2b) */
} motor_event_record_t;
```

公共 API 读出时展开为 `motor_event_t`。高频路径永远不对事件做字符串格式化。

### 11.3 MIDI/HAL 边界

HAL 抽象（`foc_pwm_if_t`、`foc_adc_if_t`）不包含任何厂商类型。每个回调通过自己的 `pContext` 获得平台上下文，不依赖全局变量。MDI 适配器（`foc_hal_mdi_adapter.c`）负责把 HAL 回调桥接到芯片驱动。位置源的硬件读取同样通过 MDI。

## 12. 设计决策记录

### 为什么不允许直接访问 motor 私有成员

第一版重构前，应用代码直接读写 `tRt`、`tControl`、`tCurrent` 等字段。这导致：
- 任何字段布局变化都破坏应用代码；
- 无法保证多实例隔离；
- ISR 和主循环之间的数据竞争无防护。

当前设计通过不透明句柄 + 私有头 + 负面编译检查三层防护：应用代码包含 `motor.h`，布局只对 motor 内部文件可见，external code 无法通过编译访问任何成员。

### 为什么命令不直接修改状态

`motor_Start()` 只把命令放入邮箱，由 `motor_RunFSM()` 执行。这确保：
- 命令校验（配置合法性、状态前置条件）与执行（PWM 使能、角度融合）在正确的上下文分离；
- 校准、延时、资格判定等长时间流程不会被 `Start` 调用者阻塞；
- 故障清除的时序约束（PWM 必须先关闭）可被 FSM 强制执行。

### 为什么用 BAM32 而不是 float 角度

BAM32 的核心收益是零开销 wrap（无符号整数自然溢出）和直接 LUT/CORDIC 索引（高位提取，无需浮点乘除）。速度 × 周期的浮点乘法只在积分边界做一次，高频路径只剩下整数加法。此改动同时让 float 和 fixed 后端的角度存储格式统一，两种后端共享完全相同的 LUT 索引路径和角度差计算。

### 为什么 LUT 后端是默认而不是 libm

LUT 后端的 513 项 1/4 波表占 2 KB flash，最大误差 1.2e-6——比 libm 的 sinf（通常 1-2 ULP ≈ 1e-7）大一个数量级但仍远小于 1e-5 测试容差。host 测试同时以 LUT 和 libm 后端运行（TRIG=libm 目标做逐点对照），确保精度始终可验证。关键优势是确定性：LUT 在所有平台上的数值行为完全一致，不受不同 libm 实现的精度或异常处理差异影响。

### 为什么 `foc/` 不包含 DWT/CORDIC 厂商头文件

一方面是为了跨芯片可移植性，另一方面是为了支持 host 测试——host 上不存在这些外设寄存器。所有架构相关代码通过编译期后端选择（三角）或 HAL 接口（PWM/ADC）注入，FOC 内核只看到抽象。

## 13. 延后扩展项

以下是有条件的扩展路线，不属于当前版本范围。每个条目记录了触发条件和实现方向，在条件满足前不为其增加休眠枚举、状态或 API。

### 飞车启动

触发条件：产品需要在转子已旋转时接管。方向：在 STARTING 中增加速度和方向检测子流程，先资格判定位置源，再以当前估算速度预置角度发生器与控制器，最后执行无扰接管。不得复用静止启动的强制对齐流程。

### 多位置源管理器

触发条件：同一电机需要三个及以上位置源，或需要自动后备。方向：新增独立的 position manager，负责固定容量注册、优先级、健康度、交叉诊断和切换请求。motor 仍只消费 manager 输出，不在生命周期 FSM 中堆叠源类型分支。

### 闭环故障降级

触发条件：具体产品完成高速回切安全验证。方向：先同步开环发生器到当前闭环角度和速度，再反向执行角度融合，并对电流、速度和母线电压设置更严格限制。默认策略继续保持急停。

### 运行中控制模式切换

触发条件：产品确实需要 CURRENT/SPEED/POSITION 在线切换。方向：新增独立 handover 模块，为每一级外环提供输出跟踪或积分预置，按从内到外启用、从外到内退出。禁止直接修改模式枚举。

### 在线参数更新

触发条件：需要运行中整定。方向：按参数组提供版本化双缓冲，在明确的周期边界原子接管。拓扑、接口绑定和存储布局类参数仍只允许 IDLE 更新。

### 扩展诊断 FSM

触发条件：独立诊断模块出现三种以上共享流程。方向：复用安全门控、超时和限幅基础设施，将校准、固定 D/Q、电角度扫描等操作建成单独 FSM。不得让正常 motor FSM 增加诊断专用状态。
