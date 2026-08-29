# FOC 极简核心重构 — 交接文档

> 交接日期：2026-08-11 阶段（本轮工作区状态）
> 配套计划：`docs/superpowers/plans/2026-08-11-foc-core-minimal-refactor.md`
> 本文档供提交后在新环境继续开发使用，包含当前进度、构建方法、验证结果与后续任务。

---

## 1. 重构目标

把通用多实例、运行时配置的旧 FOC 框架（motor_handle_t / foc_hal 函数表 /
plan resolver / 位置源接口）重构为面向 STM32G431 单电机的极简实现：
**编译期链接 foc_port → 纯数学 foc_core → 唯一 foc_runtime_t**，
保留 float/fixed 双数值后端。详见计划文档 §1-§3。

## 2. 当前进度（提交时点）

| 任务 | 状态 | 说明 |
|---|---|---|
| Task 0 基线 | ✅ 完成 | 旧基线已记录于计划文档 §11 |
| Task 1 纯数学核心 | ✅ 完成 | foc_core + test_foc_minimal_core |
| Task 2 编译期端口 | ✅ 完成 | foc_port.h + stm32g431/foc_port.c |
| Task 3 编码器直接路径 | ✅ 完成 | foc_encoder 去 adapter，API ≤4 参数 |
| Task 4 四态生命周期+ISR | ✅ 完成（代码与验证） | foc_app 重写、ISR 已切换；计划文档勾选待补 |
| Task 5 波形+Shell 状态 | ⏳ 未开始 | 9 路波形注册、foc_app_GetStatus 已有雏形 |
| Task 6 停止构建旧框架 | ⏳ 未开始 | 收敛 FOC_SOURCES、删除旧文件 |
| Task 7 真机安全验证 | ⏳ 未开始 | 需硬件 |
| Task 8 文档更新 | ⏳ 未开始 | foc/README、foc-architecture.md |

**Task 4 已完成的实质内容**（代码层面已切换，注意计划文档中 Task 4
勾选框尚未更新，交接后第一件事可补勾选与实施记录）：

- `foc/app/foc_app.h` 重写为 §4.3 最小 API（Start/Stop/ClearFault/
  SetVoltageReference/SetCurrentReference/SetSpeedReference/GetStatus/
  HighFrequencyISR），`foc_status_t` 定义于此，仅依赖 `foc_types.h`，
  **不再 include modus.h**（host 测试可编译）。
- `foc/app/foc_app.c` 重写：唯一 `static foc_runtime_t s_tFoc`、
  IDLE→CALIBRATING→RUNNING→FAULT 四态、命令邮箱（chPendingCommand +
  perfc 中断守卫）、20 kHz ISR 链路
  （foc_port_CurrentSample → foc_encoder_Step → foc_core_step →
  foc_port_DutyCommit）、1 kHz 速度 PI（SPEED 模式）、校准 512 拍 /
  100 ms 超时 / 偏移非法进 FAULT、所有故障先 EmergencyStop、
  校准完成先提交中性 duty 再使能 PWM、MODUS 对象注册 + 最小
  `motor` Shell 命令（start/stop/clear/status/vq）。
- `foc/foc_types.h` 补充 `foc_run_state_e`、`foc_command_e`（计划 §2.1）。
- `foc/foc.mk` 从构建排除 `phase_test.c`、`foc_verify.c`（引用旧 API）。
- 新增 `tests/foc/test_foc_minimal_lifecycle.c`（fake foc_port +
  as5600 stub + `perfc_port_stub.h`）覆盖 §7 十条安全不变量中的
  可测部分（PWM 关闭、校准时序、先急停再 FAULT、STOP 立即急停、
  CLEAR_FAULT 条件、模式固定、Start 校验）。
- ISR 接线未改文件：`target/stm32g431/stm32g4xx_it.c` 的
  `ADC1_2_IRQHandler → foc_app_HighFrequencyISR()` 现在调用新生命周期。

## 3. 构建与测试命令（重要）

### 3.1 环境事实

- 仓库根：`D:\2_xundoc\project\modus_template`（默认 `TARGET_CHIP=stm32g431`）。
- **工具链不在默认路径**：`makefile` 里 `SW_ROOT ?= D:/software`，
  但本机实际在 `D:/0_software`。所有固件构建必须临时覆盖：
  `SW_ROOT=D:/0_software`（计划文档 §11 已记录此约定）。
- mingw32-make 完整路径：`D:\0_software\msys64\mingw64\bin\mingw32-make.exe`
  （不在 PATH；宿主 clang 也在该 bin 目录，测试编译需把它加入 PATH
  或显式指定 `CC=...\clang.exe`）。
- 固件编译器：`D:/0_software/llvm_for_arm/bin/clang`。
- host 测试默认 `CC ?= clang`，依赖 PATH 里的 clang。

### 3.2 Host 极简测试（核心+编码器+生命周期，float/fixed 双后端）

```powershell
$env:PATH = "D:\0_software\msys64\mingw64\bin;" + $env:PATH
& "D:\0_software\msys64\mingw64\bin\mingw32-make.exe" -C tests/foc minimal
```

单测：`minimal-core` / `minimal-encoder` / `minimal-lifecycle`。
预期输出：

```
minimal core: PASS (0 failures)        # ×2 (float/fixed)
minimal encoder: PASS (0 failures)     # ×2
minimal lifecycle: PASS (0 failures)   # ×2
```

旧框架回归（验证未破坏，Task 6 后会删除）：

```powershell
& "...\mingw32-make.exe" -C tests/foc float
& "...\mingw32-make.exe" -C tests/foc fixed
```

### 3.3 固件构建

```powershell
& "...\mingw32-make.exe" SW_ROOT=D:/0_software TARGET_CHIP=stm32g431 BUILD=debug-rel
& "...\mingw32-make.exe" SW_ROOT=D:/0_software TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
& "...\mingw32-make.exe" SW_ROOT=D:/0_software TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed CPU_FLAGS="-mcpu=cortex-m4 -mthumb -mfloat-abi=soft"
```

注意：连续切换 float/fixed/soft 构建之间需 `clean`（或加 `clean` 参数），
否则旧 .o 残留。

## 4. 本轮验证结果（提交时点实测）

| 项目 | 结果 |
|---|---|
| minimal core float/fixed | PASS (0 failures) ×2 |
| minimal encoder float/fixed | PASS (0 failures) ×2 |
| minimal lifecycle float/fixed | PASS (0 failures) ×2 |
| 旧 FOC 回归 float/fixed | PASS (0 failures) ×2 |
| STM32G431 float 固件 | 构建通过 text=54124 data=516 bss=19208 |
| STM32G431 fixed 固件 | 构建通过 text=53980 data=516 bss=19200 |
| fixed + -mfloat-abi=soft | 构建通过 text=55144 data=516 bss=19208 |
| 旧框架符号（motor_/foc_hal_/foc_controller_/foc_position_source 等） | 链接产物中已不存在（gc-sections 剔除） |
| 新符号 | foc_app_Init/Start/Stop/GetStatus/HighFrequencyISR、init_info_FocApp、shell_cmd_motor 均在 |
| foc_app/core/encoder/port 软浮点引用（fixed） | 无（仅 modus 调试框架 mwaveform/trace 有，属 1kHz 慢路径，符合 §0.3） |
| 新增代码行宽 | ≤78 字符 |

注意：Task 3 之后的 text 增长（75788→54124 后又加 Shell 到 54124）
是因为 foc_app.c 重写后旧 motor 框架被链接器剔除——这是重构预期收益；
Shell 命令引入少量 stdio 开销（sscanf/MLOGF）。

## 5. 关键文件清单

### 新增（未跟踪）

- `foc/foc_types.h` — 公共组合类型 + run_state/command 枚举
- `foc/hal/foc_port.h` — 编译期端口 API（无函数指针）
- `peripheral/stm32g431/foc_port.c` — ADC 采样/校准（512 拍、偏移
  [20000,60000]）/PWM 提交/使能/急停
- `tests/foc/test_foc_minimal_core.c` / `test_foc_minimal_encoder.c` /
  `test_foc_minimal_lifecycle.c`
- `tests/foc/perfc_port_stub.h` — host 测试中断守卫桩

### 修改（已跟踪）

- `foc/middleware/foc_core.h/.c` — 极简核心（Task 1）
- `foc/observer/foc_encoder.h/.c` — 去 adapter、新三参 API（Task 3）
- `foc/app/foc_app.h/.c` — 极简应用层（Task 4，重写）
- `foc/foc.mk` — 排除 phase_test.c / foc_verify.c
- `foc/hal/foc_hal_types.h`、`foc/math/foc_angle.c`、
  `foc/modulation/foc_modulation.h`、`peripheral/stm32g431/halcordic.c` —
  Task 1/2 相关清理
- `tests/foc/Makefile` — minimal 三套 target + include 路径
- `docs/superpowers/plans/2026-08-11-foc-core-minimal-refactor.md` —
  Task 3 勾选与实施记录（Task 4 勾选待补）
- `.agents/skills/embedded-coding/SKILL.md` — 用户侧规则更新（勿覆盖）
- `modus` — 子模块指针（勿动，保持原样）

## 6. 下一步（Task 5 起）

### Task 5：波形与 Shell 状态收敛

- `src/userconfig.h`：`MWAVEFORM_SNAPSHOT_ENABLE` 改 0（当前仍为 1），
  保持 `MWAVEFORM_MAX_CHANNELS 9`。
- `foc_app.c`：按计划 §6 表格注册固定 9 路变量（Iu/Iv/Iw/Id/Iq/Angle/
  Speed/Vd/Vq，float 版；Angle 用 `s_tFoc.fElectricalAngleTurns` 展示
  字段，ISR 已写该字段），检查每个 AddVariable 返回值；删除
  AddChannel/Push/SnapshotFeed/SnapshotStart 路径；fixed 版不注册。
- `foc_app_CmdMotor` status 分支：改用一次临界区 `foc_app_GetStatus()`
  输出完整字段（§6 foc_status_t 已具备）。

### Task 6：停止构建并删除旧框架

- 收敛 `foc/foc.mk`：只构建 math、foc_core.c、foc_pid.c、SVPWM、
  foc_encoder、foc_app、STM32G431 port；`foc.h` 只导出极简类型。
- 用 `rg` 确认无引用后删除 §8 列出的旧文件（motor/ 目录、
  foc_hal*.c/h、foc_hal_mdi_adapter、phase_test.c、旧 motor 测试）。
- 注意 at32f413 目前本身构建失败（既有问题，计划 §0.4 明确 AT32
  不再构建极简 FOC，不影响本计划验收）。

### Task 7：真机安全验证（需硬件，务必遵守 aitrace 被动优先）

按计划 §7 执行：校准时序（PWM 关闭约 25.6ms → 512 拍 → RUNNING）、
急停/故障、低电流编码器闭环（Iq≤0.015 pu）、20kHz 周期预算
（<5950 cycles @170MHz）、记录最终 text/data/bss/栈对比。

### Task 8：文档更新

- foc/README.md、foc/doc/foc-architecture.md 更新为极简架构；
- 计划文档 Task 4 勾选 + §11 实施记录补 Task 4 实测数据。

## 7. 风险与注意事项

1. **at32f413 构建已坏**（历史遗留，port_mdi 链接错误），非本次改动
   引入；Task 6 收敛 FOC_SOURCES 时建议把 at32f413 排除 FOC 或同步修复。
2. **modus 是子模块**：工作区有 `M modus`，提交时确认不误提交子模块
   指针变化（本次未改 modus 内容）。
3. **host 测试依赖 PATH 中的 clang**（msys64 mingw64 bin），换环境后
   需确认该路径或显式 `CC=`。
4. **计划文档 §11 的尺寸表格**记录到 Task 3；Task 4 后的实测值
   （text≈54k、bss≈19.2k）待交接后补录。
5. **foc_app_Init 签名**保留 MODUS 的 `(uintptr_t, uintptr_t)`，但
   runtime 是文件内静态 `s_tFoc`，对象地址实际未使用——后续如有多
   实例需求需重新设计，当前产品约束单实例。
6. **编码器样本超时默认 100 tick（5ms）**：真机若 AS5600 1kHz 更新
   正常无影响；若 I2C 抖动需在 foc_app_Init 调整
   `tEncoderParams.hwInvalidTimeout`。
7. 当前 `foc_app_CmdMotor` 的 start 固定 VOLTAGE 模式 Vq=0.03；
   Task 5 若加 enc/speed 命令需按 foc_control_mode_e 组装 command。

## 8. 提交建议

- 建议分两个提交：① Task 1-3（core/port/encoder + 测试）；② Task 4
  （foc_app 重写 + lifecycle 测试 + foc.mk 排除 + 计划文档勾选）。
- 提交前可先补计划文档 Task 4 勾选与实施记录，保持文档与代码同步。
- 不要 stage `modus` 子模块变更（除非确认）。
