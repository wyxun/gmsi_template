# FOC Motor 重构 Task 8 交接

日期：2026-07-19

## 1. 恢复位置

- 仓库：`E:\Project\modus_template`；
- 分支：`master`；
- 基线：`aae829b task 6` + Task 7/8 工作区改动（未提交，未 stage）。

前置交接：`docs/superpowers/handoffs/2026-07-19-foc-motor-task7-handoff.md`。

## 2. 已完成范围（Task 8：迁移 FOC 应用层并恢复目标固件构建）

计划 6 个步骤全部完成，复选框已在计划文档勾选。

## 3. 文档化 review 检查（计划 Step 1）

```powershell
rg -n "->tRt|->tControl|->tCurrent|->tParams|foc_ipark|foc_svpwm|motor_Enable|motor_SetDuty|motor_SampleCurrent" foc/app
```

- 迁移前：41 处匹配（foc_app.c 32 处、phase_test.c 9 处），覆盖直接成员
  访问、旧状态名、低层公开 API 和算法直调；
- 迁移后：仅剩 1 处匹配 `phase_test.c:33 foc_ipark`，位于
  `#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC` 门控内
  （phase_testA 坐标变换数学自检），属计划允许的显式门控诊断代码。

## 4. 迁移要点

### 4.1 app FSM 编排化（foc/app/foc_app.c，重写）

`foc_app_RunFSM()` 缩减为纯编排：参数检查后直接 `return motor_RunFSM(...)`。
开环角推进、校准、PWM 使能、占空比写入、电流重构、Park/SVPWM、motor 状态
赋值全部删除（这些已由 motor 内部启动 FSM 与高频步接管）。

### 4.2 按钮与 Shell

- 按钮按下沿：先取 `motor_GetSnapshot()`——有故障位则打印故障并尝试
  `motor_ClearFault()`；IDLE 则 `foc_app_Start()`；其余状态
  `foc_app_Stop()`；
- `foc_app_Start()` 用产品持有的单一静态 `s_tMotorRunConfig`
  （`MOTOR_CONTROL_VOLTAGE_OPEN_LOOP`、无位置源 = 内部开环角度发生器、
  1 e-turn/s、5 turn/s²、Vq 0.05 pu）调 `motor_Start()`；故障激活与命令
  被拒绝分开打印；`foc_app_Stop()` 调 `motor_Stop()` 并区分接受/拒绝；
- Shell `motor` 命令扩展为 `start|stop|clear|vq <x>|status`：status 全部
  从快照打印（状态/启动相位/模式/故障/事件序号/角度/占空比/电流/校准），
  `vq` 同时更新 run config 并调 `motor_SetVoltageReference()`；
- 心跳与波形监控（phase_test.c）改从快照取数；
- `motor_DebugReadEvent()` 仅在主循环 drain 做日志，控制路径不依赖事件环。

### 4.3 实时调度（ISR 绑定）

- 高频：`target/at32f413/at32f413_it.c` 的 `ADC1_2_IRQHandler`（TMR1 CH4
  触发的抢占转换完成，20 kHz 载波一次）调 `foc_app_HighFrequencyISR()` →
  `motor_HighFrequencyStep()`；这是唯一高频调用点。为此在
  `peripheral/at32f413/haladc.c` 补使能 `ADC_PCCE_INT`（此前 NVIC 开了但
  ADC 中断源未使能，handler 从不会触发）。target 文件改动仅限该 ISR；
- 低频：`foc_app_Clock()`（MODUS 1 ms 系统时钟，SysTick → modus_Clock）
  调 `motor_LowFrequencyStep()`，已在代码注释中文档化为唯一低频调度点；
- 主循环 FSM 不再跑任何高频算法。

### 4.4 sync/time 绑定与 config 填写

- `motor_sync_if_t`：perf_counter 多架构全局中断守卫
  `perfc_port_disable_global_interrupt()` /
  `perfc_port_resume_global_interrupt()`（`modus/src/arch/perfc_port.h`，
  Cortex-M 用 PRIMASK、RISC-V 用 mstatus.MIE、其他架构空实现；中断安全、
  允许嵌套，无 vendor 头）。2026-07-19 应用户要求由最初的 CMSIS
  `__get_PRIMASK`/`__set_PRIMASK` 直写改为此可移植方案
  （`global_define.h` 只对 RISC-V 重映射 `__disable/enable_irq`，不覆盖
  `__get/__set_PRIMASK`，直写在 RISC-V 上无法编译）；
- `motor_time_if_t`：包装 `get_system_ms()` 为 uint32_t；
- `motor_config_t` 补齐：`qHighFrequencyPeriod = 50us`（20 kHz，中心对齐
  TWO_WAY_1 模式 CH4 仅向下计数比较一次，已在注释说明）、
  `qLowFrequencyPeriod = 1ms`、tPosition（4 极对、方向 +1）、切换门限
  （置信度 0.8 / 最小速度 0.05 / 最大角误差 0.25 圈 / 超时 1000 ms /
  资格 3 样本 / 融合 8 样本）、`wStartupDelayMs = 200`；
- tControl 绑定 4 个真实 PID（foc_pid + foc_controller_FromPid），电压开环
  模式不使用，为后续闭环模式预置（motor_Start 对闭环模式强制校验）。

### 4.5 phase_test 处置

- 正常初始化（FOC_ENABLE_DIAGNOSTIC=0）：phase_testA/B 为空实现，不再
  有任何固定占空比旁路；phase_testC 仅打印；
- 新增 `foc/diagnostic/motor_diagnostic.c/.h`（默认不进构建）：固定占空比
  直测先校验快照 IDLE + 无故障，占空比限 0.1 pu、保持 100 ms、过流限
  （原始计数偏移 > 校准偏移 1/2）检查，超时 2000 ms 由 motor 侧强制；
- motor.h/motor.c 新增门控狭窄诊断 API `motor_DiagnosticSetOutput()` /
  `motor_DiagnosticStopOutput()`（`FOC_ENABLE_DIAGNOSTIC=1` 才编译），
  强制 IDLE/无故障/占空比上限/累计时长检查；诊断模块本身不接触 motor
  私有成员。私有布局新增 `wDiagnosticStartMs`（4B）+ `bDiagnosticActive`
  （位域剩余位），`sizeof(motor_impl_t)` 496 → 500，仍在 512 以内；
- Makefile 新增 `FOC_DIAGNOSTIC ?= 0` → `-DFOC_ENABLE_DIAGNOSTIC=`，
  `FOC_DIAGNOSTIC=1` 时才把 `foc/diagnostic/*.c` 加入 FOC_SOURCES。

### 4.6 顺带修复（恢复链接必需）

- `src/main.c`：`grbl_enter()` 无条件调用导致无 grblHAL 的目标链接失败
  （f351319 引入的既有问题）。改为 `#if defined(GRBLHAL_ENABLE)` 走
  grblHAL，`#elif MODUS_ENABLE` 走 `while(1) modus_Run()`。

## 5. 验证结果（2026-07-19，全部通过）

host 测试矩阵（GCC，含 O2 strict-alias 变体）：

- encapsulation：`PASS: motor_handle_t rejects direct member access`；
- float：`FOC tests: PASS (0 failures)`；
- fixed：`FOC tests: PASS (0 failures)`；
- O2 strict-alias float/fixed：PASS；
- 注：`make.bat` 硬编码 `D:\0_software`，本机实际工具链在 `D:\software`，
  用 `mingw32-make SW_ROOT=D:/software` 等价执行；host 测试用
  `mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all`。

目标固件（Makefile 默认 target = at32f413，AGENTS.md 中 at32f407 的描述
已过时）：

- 默认 debug 构建（at32f413, float）：编译链接成功，
  `text 67012 / data 2296 / bss 4904`；
- `FOC_DIAGNOSTIC=1` 构建：成功（门控诊断代码可编译）；
- `TARGET_CHIP=at32f407`：成功，`text 279496 / data 2316 / bss 48440`；
- 迁移后 Step 1 搜索：见第 3 节，仅剩 1 处门控诊断匹配。

## 6. 已知事项与遗留

- `foc/README.md` 仍含旧 API 示例，属 Task 9 范围，未动；
- stm32g431 目标的 FOC ISR 尚未接 `foc_app_HighFrequencyISR()`
  （其 it.c 仍是 TODO 注释），at32f421/ch592 无 FOC；多目标构建验证属
  Task 10 范围；
- 高频周期 50 us 基于 TMR1 中心对齐 TWO_WAY_1 下 CH4 每载波比较一次的
  推断（未在示波器上实测），已文档化在 foc_app.c 注释；
- snapshot 中 `tElectricalAngle/qElectricalSpeed` 与 active 字段同源的
  冗余仍留待后续任务（Task 7 交接已声明）；
- `tests/foc/foc_test_float.exe`（tracked 生成物）显示 modified 属正常；
  `foc_test_fixed.exe` untracked。未 stage/commit；
- modus 子模块与 grblhal/vendor 子模块的既有改动未触碰。

## 7. 下一步：Task 9

更新 `foc/README.md` 与 `foc/foc.h`：替换直接成员示例、补四个完整用法
示例、统一位置源适配说明、延后扩展项备注、公共头一致性核查。
