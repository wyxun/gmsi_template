# FOC 高频控制环路资源占用优化方案（第二版 / V2.1）代码落实与上机实测报告

## 1. 概述与优化目标

本报告总结了《FOC 高频控制环路资源占用优化方案（第二版 / V2.1）》的代码落实成果与 STM32G431 真实硬件板卡（170 MHz 主频，20 kHz 控制频率）的上机实测数据。

本轮重构旨在不降低 20 kHz 控制频率、不削弱硬件安全保护的前提下，实现 FOC 高频环路的零开销多 Motor Profile 统计、BAM32 角度模型引入、架构解耦直读 DWT CYCCNT 以及高频链路指令级瘦身。

---

## 2. 核心架构与代码变更明细

### 2.1 P0: Profile 编译期零开销闭包与多 Motor 对象化 (Mandatory)

1. **私有快照与多 Motor 隔离 (`foc/motor/motor_profile.h`, `foc/motor/motor_private.h`)**：
   - 彻底移除了 `g_wSampleCurrentCycles`、`g_wClarkeCycles` 等全局单例变量。
   - 在 `motor_impl_t` 中新增 `motor_hf_profile_snapshot_t tProfileSnapshot` 字段，且仅在 `FOC_HF_PROFILE` 启用时分配。
   - 引入动态分配控制：开启 Profile 时 `MOTOR_HANDLE_STORAGE_SIZE` 为 576 字节；关闭 Profile 时恢复为 512 字节，无任何 RAM 浪费。

2. **高频步单点发布 (`foc/motor/motor_control.c`)**：
   - 在 `motor_HighFrequencyStep()` 内部仅使用栈上局部变量记录各阶段 cycles 与有效标记位。
   - 控制步统一汇聚至 `publish_exit` 标号，一次性写回 `ptImpl->tProfileSnapshot` 并更新样本序号，彻底消除高频步中间多次写全局 `volatile` 变量的总线开销。

3. **短临界区安全读取 API (`foc/motor/motor.c`)**：
   - 实现 `motor_GetHighFrequencyProfileSnapshot(ptMotor, ptSnapshot)`，在 `motor_sync_if_t` 短临界区内完成快照复制。
   - 当 `FOC_HF_PROFILE = 0` 时，该 API 编译为返回 `FOC_RESULT_DISABLED` 的恒等内联/空操作，零运行开销。

4. **分级测量边界与 DWT 硬件直读解耦 (`foc/motor/foc_hf_profile.h`)**：
   - `FOC_HF_PROFILE_LEVEL = 0`：零测量开销、零 DWT 读取、零全局符号残留。
   - `FOC_HF_PROFILE_LEVEL = 1`：仅测量并记录整环总 cycles（Level 1）。
   - `FOC_HF_PROFILE_LEVEL = 2`：记录细粒度 6 阶段分段 cycles。
   - **架构解耦设计**：按项目规范，FOC 业务内核严禁包含芯片厂商/CMSIS 设备头文件。为了防止 `defined(DWT)` 判断失败隐式退化为带关中断锁的 `get_system_ticks()`，改用 ARM CoreSight 架构固定地址（`0xE0001004`）直读 `DWT->CYCCNT`，单次读取仅需 1 条 `LDR` 指令；在 `motor_Init()` 时调用 `foc_hf_profile_InitCycles()` 完成 TRCENA 与 CYCCNTENA 初始化。

---

### 2.2 P1.0: BAM32 角度数据模型重构

1. **角度类型统一 (`foc/math/foc_angle.h`)**：
   - 定义 `foc_angle_t { uint32_t wBam32; }`，其中 `0 ~ 2^32-1` 对应 `0.0 ~ 1.0` 圈（$0^\circ \sim 360^\circ$）。

2. **零开销回绕与裁撤除法 (`foc/math/foc_angle.c`)**：
   - 角度累加直接靠 32 位无符号整数自然溢出完成，`foc_angle_wrap()` 保持为零开销内联 API。
   - 彻底移除了定点模式下 `% FOC_ONE` 的长除法指令。
   - 修正了定点 Q15 模式下的 BAM32 缩放映射（$1.0 \text{ turn} = 32768 \times 131072 = 2^{32}$，移位统一为 `>> 17`）。

---

### 2.3 P1: 通用成对 SinCos 接口

1. **成对接口定义 (`foc/math/foc_angle.h`)**：
   - 提供 `foc_angle_sincos(tAngle, &qSin, &qCos)`。
   - LUT 后端 (`foc_trig_lut.c`) 直接取 `wBam32` 高 9 位作为查表索引，一次性计算成对三角函数。
   - CORDIC 后端 (`halcordic.c`) 仅写入一次寄存器并并行读取 Sin/Cos 结果。

---

### 2.4 P2 ~ P4: 采样重构与高频编译优化

1. **采样与 PWM 原子提交规范**：
   - 保证通用内核不依赖特定芯片定时器寄存器，由 `peripheral/<chip>/` 提供 PWM 三相预装载原子更新。

2. **全量高频链路 `-O2` 编译 (`make.bat` / `Makefile`)**：
   - 高频控制源文件统一施加 `-O2` 优化，且禁用 `-ffast-math` 以保证数值严密性。

---

## 3. 代码静态审计与 Host 仿真验证结果

### 3.1 符号残留校验 (Release 目标 ELF)

使用 `llvm-nm` 对 Release 构建产物 `build/template.elf` 进行全局/静态符号检查：
- **Profile 符号搜索结果**：`0` 个 Profile 相关变量符号残留。
- **结论**：完全达成了 `FOC_HF_PROFILE=0` 时的编译期零开销准入标准。

### 3.2 目标镜像尺寸 (STM32G431 Cortex-M4)

```text
   text    data     bss     dec     hex filename
  82632     924    9752   93308   16c7c build/template.elf
```

### 3.3 Host 单元测试矩阵 (`tests/foc`)

通过 MinGW GCC 在 Host 环境运行全量单元测试矩阵：

| 测试项 | 编译条件 / 模式 | 测试结果 | 备注 |
| --- | --- | --- | --- |
| **Encapsulation Probe** | `motor_handle_t` 封装断言 | **PASS** | 验证拒绝直接访问 `tRt` 等私有成员 |
| **foc_test_float** | `FOC_NUMERIC_FLOAT=1` | **PASS (0 failures)** | 全角度 LUT 误差 $< 1.26\times 10^{-6}$ |
| **foc_test_fixed** | `FOC_NUMERIC_FIXED=1` (Q15) | **PASS (0 failures)** | BAM32 定点变换与 Cogging 补偿全量通过 |
| **foc_test_profile** | `FOC_HF_PROFILE=1, LEVEL=2` | **PASS (0 failures)** | 验证多 Motor 快照读取与 Level 2 阶段记录 |

---

## 4. 上机实测结果分析 (STM32G431 @ 170MHz, 20kHz PWM)

在 170 MHz 主频、20 kHz 控制频率下，单次中断周期上限预算为：
$$\text{Max Budget} = \frac{170\,\text{MHz}}{20\,\text{kHz}} = 8,500\,\text{cycles} \quad (50.0\,\mu\text{s})$$

接入硬件板卡，在 `FOC_HF_PROFILE_LEVEL = 2`（DWT 物理硬件直读口径）下实测 RTT 心跳数据如下：

### 4.1 开环对齐 / IDLE 阶段耗时拆解

- **总 cycles (`total`)**：**3,588 cycles** (占用率 **42.2%**，耗时约 **21.1 μs**)
- **分阶段耗时明细**：
  - `sample`（三相电流采样/校准）：**687 cycles**
  - `clarke`（Clarke 变换）：**128 cycles**
  - `park`（Park 变换）：**172 cycles**
  - `ipark`（反 Park 变换）：**161 cycles**
  - `modulate`（SVPWM 矢量调制）：**450 cycles**
  - `commit`（高频步状态更新与 PWM 寄存器提交）：**879 cycles** (IDLE 止步阶段为 205 cycles)

### 4.2 电流闭环运行阶段耗时拆解 (Current-Loop Closed)

- **总 cycles (`total`)**：**4,306 ~ 4,378 cycles** (占用率 **50.7% ~ 51.5%**，耗时约 **25.3 ~ 25.7 μs**)
- **分阶段耗时明细**：
  - `sample`：**687 ~ 694 cycles**
  - `clarke`：**128 cycles**
  - `park`：**172 cycles**
  - `ipark`：**165 cycles**
  - `modulate`：**430 cycles**
  - `commit`：**879 cycles**

### 4.3 性能对比与口径说明

| 评估指标 / 阶段 | 原始基线 (V0) | 第一版重构 (V1.0) | **第二版实测 (V2.1, LEVEL=2 DWT)** | **性能提升幅度 / 结论** |
| :--- | :--- | :--- | :--- | :--- |
| **开环总 Cycles** | ~7,442 cycles (87.5%) | ~5,241 cycles (61.7%) | **3,588 cycles (42.2%)** | **↓ 3,854 cycles (-51.8%)** |
| **电流闭环总 Cycles** | > 9,000 cycles (溢出风险) | ~5,200 cycles (61.2%) | **4,306 ~ 4,378 cycles (51.5%)** | **彻底消除 ISR Overrun 风险** |
| **单周期耗时 (μs)** | ~43.7 μs | ~30.6 μs | **21.1 μs (开环) / 25.7 μs (闭环)** | **CPU 空间余量高达 48.5%** |

> **注（口径区分）**：LEVEL=1 阶段日志显示的 total (3,768~4,617) 采用 `get_system_ticks()` 计算，包含了 `perf_counter` 的关中断锁与非内联函数开销；LEVEL=2 直读 `DWT->CYCCNT` 为真正的物理硬件 cycle 口径，实测闭环仅需 4,300 cycles 左右，性能远超预期。
