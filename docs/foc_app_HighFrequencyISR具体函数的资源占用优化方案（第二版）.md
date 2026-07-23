# FOC 高频控制环路资源占用优化方案（第二版 / V2.1 修订版）

## 1. 目的与适用范围

本文是对《foc_app_HighFrequencyISR具体函数的资源占用优化方案》的 V2.1 修订版。目标是在不改变 FOC 控制周期、不削弱硬件保护、且不破坏 `float` 与定点数值后端一致性的前提下，系统性降低 `motor_HighFrequencyStep()` 及其 ADC/PWM 中断入口的 CPU 占用与抖动。

本方案面向多芯片工程（如 STM32G431、AT32F413/407 等），遵循以下设计原则：

- `foc/`、`foc/motor/`、`foc/math/` 和 `foc/middleware/` 为纯算法与调度内核，绝不包含厂商头文件、寄存器定义或芯片专用 DMA/FMAC 配置。
- `peripheral/<chip>/` 与 `target/<chip>/` 只负责 ADC 采样、PWM 更新、可选数学/滤波加速器、计时器和中断入口的适配。
- 硬件加速器（如 CORDIC、FMAC）仅作为可选平台适配后端；无加速器的芯片必须以通用软件后端获得相同的控制语义和严密可验证的数值误差。
- 本方案禁止将“降低 20 kHz 控制频率”作为首选优化手段。高频控制中断是采样、电流调节和 PWM 预装载更新的实时边界。

---

## 2. 核心结论与整改目标

### 2.1 参考基线 (STM32G431 170MHz, 20kHz, 8500-cycle 预算)

| 指标 | 优化前 (V0) | V1 已实现成果 | V2.1 目标预算 |
| --- | ---: | ---: | ---: |
| 高频步总周期 (含 Profile) | 7442 cycles (87.4%) | 5241 cycles (61.7%) | ≤ 3400 cycles (≤ 40%) |
| 纯 FOC 步 cycles (无 Profile) | ~7442 cycles | 4608 cycles (54.2%) | ≤ 2975 cycles (≤ 35% P99) |
| 电流闭环 P99 | 未测 | 约 5200 cycles | ≤ 4250 cycles (≤ 50% P99) |

*注：以上数值为参考目标板基线，跨芯片验收统一使用“周期预算占比、P99 与最大抖动 (max)”。*

---

## 3. V2.1 核心优化项定位与规则

根据对芯片微架构与指令集的深度审计，确定 5 项新增/精细化优化项的定位规则：

| 优化项目 | 最终定位与结论 | V2.1 方案处理规则 |
| --- | --- | --- |
| **Profile 零开销闭包** | **强烈推荐 (P0 强制准入)** | 修复 `stm32g4xx_it.c` 等处残留的 Profile 变量与 DWT 计数；关闭时宏不能引用参数/定义局部变量，nm 校验 0 符号残留。 |
| **BAM32 角度数据模型** | **强烈推荐 (P1.0 独立前置阶段)** | 定义 `foc_angle_t { uint32_t wBam32; }`；消除 Fixed 模式下 `% FOC_ONE` 整数除法；每周期 speed×Ts 入口仅转换一次，非实时边界保留 Turns API。 |
| **热数据布局 / LDRD** | **P3 汇编与 DWT 证据驱动项** | 热字段按访问关系连续排布 + 4 字节自然对齐；不预设“访存减半”承诺；仅在 `-O2` 反汇编证明更优且 DWT P99 获益时才施加 `_Alignas(8)`。 |
| **无分支 Clamp / Sat** | **P1/P3 比较路径汇编审计项** | 纠正“VMAXNM 指令”在 CM4F VFPv4-SP 上的误设；Fixed 审计 `SSAT`/`USAT` 结合；Float 对比 `if` 与条件指令生成；绝不破坏 SVPWM 保护钳位。 |
| **FMAC 硬件加速** | **P5 目标层可选扩展** | 不进入 `foc/` 内核；仅作为 `target/` 适配层中允许 1 周期延迟的滤波接口 (`foc_filter_if_t`)，需评估 DMA/FIFO 同步成本。 |

---

## 4. 分阶段实施路线 (Phase-by-Phase)

### P0：Profile 编译期零开销闭包与基线审计 ( mandatory )

**目标：** 彻底剔除量产/非 Profile 构建下的所有性能测量残留开销，建立无污染的基线。

**整改内容：**
1. 统一封装 Profile 宏闭包：
   ```c
   #if FOC_HF_PROFILE
     #define FOC_HF_PROFILE_BEGIN(name) uint32_t name = foc_hf_cycles()
     #define FOC_HF_PROFILE_END(name, dst) do { (dst) = foc_hf_cycles() - (name); } while (0)
   #else
     #define FOC_HF_PROFILE_BEGIN(name)    do { } while (0)
     #define FOC_HF_PROFILE_END(name, dst) do { } while (0)
   #endif
   ```
2. 彻底清理 `target/stm32g431/stm32g4xx_it.c:76` 等入口处默认保留的 `g_wHFStepCycles`、DWT 读取与统计全局变量。
3. **多 motor 对象化：** 禁止 `g_wSampleCurrentCycles` 一类文件级单例统计量。Profile 构建中，每个 `motor_impl_t` 持有独立的私有 `motor_hf_profile_t`；每份记录只代表该 motor 最近一次已完成的高频步。Profile 关闭时，该字段、所有计时局部变量与发布逻辑全部不参与编译。
4. **一次发布：** ISR 仅把阶段计时保存在局部变量中；在高频步结束时发布一份完整的 `motor_hf_profile_snapshot_t`，包含样本序号、总 cycles、六个阶段 cycles、有效位和本周期执行结果。禁止阶段中间多次写全局 `volatile` 变量。
5. **一致性读取：** `motor_GetHighFrequencyProfileSnapshot()` 按该 motor 已绑定的 `motor_sync_if_t` 在短临界区内复制记录。应用层按自己的 motor 句柄读取和标识记录，不在 FOC 内核维护全局 motor ID 或全局 profile 表。
6. **分级开销：** `FOC_HF_PROFILE=0` 为量产零开销；`FOC_HF_PROFILE_LEVEL=1` 只记录总 cycles；`FOC_HF_PROFILE_LEVEL=2` 才记录阶段 cycles。Level 2 的读计数器成本必须单独校准，不能与无 Profile 性能基线混用。
7. **准入标准：** 使用 `llvm-nm` 或 `arm-none-eabi-nm` 对 Release / 非 Profile 构建的 ELF 进行符号校验，断言不存在 Profile 相关的全局/静态变量符号；Profile 构建的双 motor 交错调用测试必须证明两个句柄得到独立、完整且不串扰的记录。

---

### P1.0：BAM32 角度数据模型重构 ( 独立前置阶段 )

**目标：** 将电角度统一抽象为 BAM32（32 位二进制角度），彻底消除角度 wrap 运算与 Fixed 模式下的高频除法。

**整改内容：**
1. 重构 `foc_angle_t` 定义：
   ```c
   typedef struct {
       uint32_t wBam32; /* 0 ~ 2^32-1 对应 0 ~ 360 度 */
   } foc_angle_t;
   ```
2. **零开销回绕 (Wrap)：** 角度累加 `angle.wBam32 += step_bam32` 自然靠 32 位无符号整数溢出完成；`foc_angle_wrap()` 只保留为内联恒等 API 以兼容调用方，不得保留取模或边界判断。
3. **定点除法裁撤：** 彻底消除 `foc_angle.c` 中 Fixed 模式下 `% FOC_ONE` 的长除法指令。
4. **单次标度转换：** 对仍以 float/fixed speed 表示的控制环，`speed × Ts` 到 BAM32 的转换只允许在积分入口每周期一次；LUT 查表直接取 `wBam32` 高位作为索引。支持 BAM32 的硬件 trig 适配器可直接消费该值，但该行为仅属于 `peripheral/<chip>/`，不进入 `foc/` 通用内核。
5. 保留 `foc_angle_from_turns()` 和 `foc_angle_to_turns()` 作为低频/初始化边界 API。

---

### P1：通用数学内核与成对 SinCos 优化

**目标：** 一次角度输入同时生成 $\sin$ 与 $\cos$，消除重复查表/角度处理。

**整改内容：**
1. 新增成对数学接口：
   ```c
   void foc_angle_sincos(foc_angle_t tAngle, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
   ```
2. LUT 后端（`foc_trig_lut.c`）：利用 BAM32 高 9 位查表，共享象限分解与对称变换，一次返回 sin 和 cos。
3. CORDIC 后端（`halcordic.c`）：向 CORDIC CSR 写入一次，直接并行读取 X (cos) 与 Y (sin) RESULT 寄存器。
4. 在 `motor_control.c` 高频步中替换为单次 `foc_angle_sincos()` 调用，传给 `foc_park_cached()` 与 `foc_ipark_cached()`。

---

### P2：采样重构与 PWM 提交瘦身

**目标：** 把不随样本变化的比例与拓扑逻辑留在初始化/标定期。

**整改内容：**
1. **倒数预计算：** 电流采样校准完成后，预先计算相电流归一化倒数乘数。每周期使用 `raw * rcp` 乘法替代除法。
2. **PWM 三相原子提交：** `foc_hal_SetDuty(&impl->tHal.tPwm, u, v, w)` 仍是通用内核接口；目标适配器将其实现为一次三相预装载提交或等价硬件事务。通用文档与 `foc/` 不出现具体定时器寄存器名称。
3. **错误显示传递：** ADC overrun 或采样未就绪显式返回错误，促使电机安全急停。

---

### P3：高频状态与并发边界重构 ( 布局与汇编审计 )

**目标：** 缩短临界区，优化内存访存布局。

**整改内容：**
1. 拆分高频控制状态 `motor_hf_runtime_t`（高频实时更新）与遥测镜像 `motor_telemetry_t`（按 1kHz 发布）。
2. 使用版本号邮箱（`version - data - version`）传递低频写入的命令与引用值。
3. **热数据布局与汇编审计：**
   - 将连续调用的字段（Alpha/Beta、D/Q、Duty U/V/W）按 4 字节自然对齐紧凑排布。
   - 反汇编审计 `-O2` 下生成的指令，仅在 DWT 实测证明 8 字节对齐（`_Alignas(8)`）能带来 P99 改善时才局部开启。
   - 审计 `foc_sat()` 汇编，Fixed 模式下核查 `SSAT`/`USAT` 指令生成；Float 模式下比对条件指令与 `if` 的生成质量，不强制假设无分支。

---

### P4：高频调用闭包 `-O2` 编译

**目标：** 确保高频链路上的所有源文件以一致的 `-O2` 编译。

**整改内容：**
1. 根 `Makefile` 中定义 `FOC_HF_SOURCES`，覆盖 ISR 薄封装、`foc_app.c`、`motor_control.c`、FOC HAL 包装、数学/调制模块及当前目标的 ADC、PWM、trig 适配器；清单按目标声明，不能假设某个芯片文件名存在。
2. 全量施加 `-O2`，禁止使用 `-ffast-math`。

---

### P5：可选平台加速后端 (CORDIC / FMAC)

**目标：** 在不污染 `foc/` 通用内核的前提下提供平台级加速能力。

**整改内容：**
1. 硬件加速实现位于 `peripheral/<chip>/`，通过 `foc_trig_if_t` / `foc_filter_if_t` 向上提供能力。
2. FMAC 仅作为 target 适配层可选接口，面向允许 1 周期延迟的 BEMF 或低频滤波路径。

---

## 5. 验收标准与验证矩阵

1. **Host 单测矩阵：** `tests/foc` 下 `float`、`fixed` 与 `strict-aliasing` 三个组合全量 Pass，全角度 sin/cos 误差 $< 2\times 10^{-6}$。
2. **Profile 残留校验：** 量产构建 ELF 中经 `llvm-nm` 校验 0 Profile 符号。
3. **目标板性能验收：** 每个支持 FOC 的目标均使用本芯片可用的周期计数器或 GPIO 时序测量，分别报告 `mean / P99 / max / overrun_count`。STM32G431 170MHz 下采用以下参考门槛：
   - 纯 FOC 开环 P99 ≤ 35% (2975 cycles)；
   - 电流闭环 P99 ≤ 50% (4250 cycles)；
   - 连续 100,000 周期无 overrun、无 ADC/PWM 提交错误。
4. **跨芯片准入：** 不支持 CORDIC、FMAC、DWT 或三相定时器预装载的目标仍必须可用软件 trig、目标本地计时器和既有 PWM 适配器完成 float/fixed 矩阵；不得因缺少某 STM32G4 外设而降低通用功能或放宽数值容差。
