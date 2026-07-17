# FOC 通用使用手册 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建 `foc/README.md`，使新用户能够完成通用 MCU 移植、单电机或
多电机初始化、控制周期接入及算法配置。

**Architecture:** README 采用任务导向结构，先解释分层和数值约定，再给出从
HAL 到电机控制的完整使用流程，最后提供模块接口索引。所有示例只引用当前
源码中的公共类型和函数，不依赖厂商 HAL。

**Tech Stack:** Markdown、C11、项目 Makefile、PowerShell/rg 静态核验。

---

### Task 1: 审计公共接口和实际调用顺序

**Files:**
- Read: `foc/**/*.h`
- Read: `foc/motor/motor.c`
- Read: `foc/motor/motor_control.c`
- Read: `foc/app/foc_app.c`
- Read: `peripheral/at32f413/foc_hal_mdi_adapter.c`
- Read: `tests/foc/*.c`

- [ ] **Step 1: 提取公共函数、结构体和枚举**

运行：

```powershell
rg -n "^(typedef|foc_result_t|void|bool|foc_scalar_t|foc_.*_if_t)" `
    foc -g "*.h"
```

预期：得到数值、HAL、电机、控制器、观测器、优化和实验模块的公共符号。

- [ ] **Step 2: 核对初始化和周期调用顺序**

读取 `motor_Init()`、`motor_control_Init()`、`motor_control_HighFrequencyStep()`、
`motor_control_LowFrequencyStep()`、`foc_app_Init()` 以及 AT32F413 MDI 适配器，
记录调用者必须提供的对象和错误处理路径。

- [ ] **Step 3: 核对示例所需测试用法**

从 `tests/foc/test_motor.c`、`test_observer.c`、`test_optimization.c` 和
`test_experimental.c` 提取已编译验证的初始化样式，禁止根据函数名猜测字段。

### Task 2: 编写任务导向 README

**Files:**
- Create: `foc/README.md`

- [ ] **Step 1: 编写定位、架构和数值约定**

写清目录职责、依赖方向、`FOC_NUMERIC_FLOAT`/`FOC_NUMERIC_FIXED`、Q16.15、
归一化标量、归一化圈数角度和 `foc_gain_t` 的使用方法。

- [ ] **Step 2: 编写最小集成流程**

按以下顺序组织正文：包含 `foc/foc.h`、实现 MDI/HAL 适配、创建 HAL、初始化
电机对象、初始化控制器、绑定 `motor_control_config_t`、校准电流、使能 PWM、
执行高频/低频步骤、故障停机。

- [ ] **Step 3: 编写硬件无关示例**

示例必须使用真实接口，并明确标注板级代码需要实现 ADC 原始采样、PWM 更新、
输出使能和紧急关闭。示例不得包含 AT32 或 STM32 头文件。

- [ ] **Step 4: 编写算法模块使用章节**

覆盖 PID/LADRC/SMC/STA、滤波/PLL/LTD/DOB、三种调制、Hall/SMO/NLFO/HFI、
观测器切换、MTPA/弱磁/死区/相位/齿槽补偿，并说明对象所有权和周期层级。

- [ ] **Step 5: 编写实验安全和多电机章节**

明确 NSD/辨识默认关闭，给出 Make 变量、强制安全回调和守卫条件；展示每个
电机分别持有 HAL、控制器、观测器和运行状态，禁止共享可变算法对象。

- [ ] **Step 6: 编写整定、移植、构建和接口索引**

提供参数整定顺序、常见错误、移植检查表、AT32F413 双后端构建命令、主机测试
命令，以及按目录列出的主要头文件和职责。

### Task 3: 核验 README 与源码一致

**Files:**
- Verify: `foc/README.md`
- Verify against: `foc/**/*.h`

- [ ] **Step 1: 扫描占位符和厂商依赖**

运行：

```powershell
rg -n "TBD|TODO|待定|待补充|HAL_AT32|HAL_STM32" foc/README.md
```

预期：无匹配。

- [ ] **Step 2: 核对 README 中的 FOC 标识符**

提取 README 代码块中的 `foc_*`、`motor_*` 类型和函数，并用 `rg` 确认均存在
于 `foc/` 公共头文件。对只用于讲解的板级伪函数统一使用 `board_` 前缀并明确
标注“由移植层实现”。

- [ ] **Step 3: 检查 Markdown 和工作区差异**

运行：

```powershell
git diff --check -- foc/README.md
```

预期：退出码 0。确认未执行 `git add` 或 `git commit`。
