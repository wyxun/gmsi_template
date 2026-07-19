# FOC Motor 重构 Task 10 交接（收官）

日期：2026-07-19

## 1. 恢复位置

- 仓库：`E:\Project\modus_template`；
- 分支：`master`；
- 基线：`aae829b task 6` + Task 7–10 工作区改动（未提交，未 stage）。

前置交接：同目录 task7 / task8 / task9 三份文档。

## 2. 已完成范围

**Task 1–10 全部完成**，计划文档全部 54 个步骤复选框已勾选
（Task 1–5 的框为本次补勾——工作早已完成验证，框从未勾选）。

## 3. 本任务变更

### 3.1 stm32g431：剔除 grblHAL、保留 FOC（用户决定）

- `target/stm32g431/target.mk`：删除 `GRBLHAL_ENABLE = 1` 及条件块，无条
  件 `-DFOC_SUPPORT=1`；`FOC_SOURCES` 补齐与 at32f413 一致的完整模块集
  （原列表缺 control/modulation/observer/optimization/experimental 五个
  目录，曾导致 `foc_pid_Init` 等 undefined symbol）；删除已失效的
  `CLASS_SOURCES += class/grblhal.c`（该文件早已在 b0d55f7 删除）和
  grblHAL 专属 settings.o 编译标志；
- `foc/app/foc_app.c`：按钮处理用 `#if defined(MDI_HW_HAS_BUTTON_START)`
  守卫（G431 的 `mdi_hardware_t` 无 `ptButtonStart` 字段，无按钮芯片
  编译为空操作）；`peripheral/at32f413/mdi_hw.h` 新增该宏，at32f413
  行为不变；
- `target/stm32g431/stm32g4xx_it.c` **未动**——FOC 高频 ISR 未接线，
  固件可编译链接，但 `motor_HighFrequencyStep()` 运行时不会被调度，
  接线留待后续（target.mk 顶部注释已记录）。

### 3.2 遗留修复

- `make.bat`：工具链路径改为自动探测——`MAKE_EXE` 环境变量优先 →
  `D:\0_software\...` 存在则用 → 回退 `D:\software\...` → 都没有则报错
  退出；参数透传与 auto/rttv 流程不变；
- `AGENTS.md`：默认 target 订正 `at32f407` → `at32f413`；
- 临界区多架构化（本日早些时候完成）：`foc_app.c` 的 sync 绑定由 CMSIS
  `__get/__set_PRIMASK` 改为 perf_counter 多架构守卫
  `perfc_port_disable_global_interrupt()` /
  `perfc_port_resume_global_interrupt()`，RISC-V 可编译；
- `grblhal_adapt/grblhal_stubs.c` 中途的临时改动已完整还原（与 HEAD
  一致）。

## 4. Task 10 验证结果

### 4.1 格式与陈旧符号

- `git diff --check`：foc/tests 范围干净；
- 旧公共名（`motor_control_mode_t`、`sensor_interface_t`、
  `observer_interface_t`、`motor_Control(Start|Stop|Set|High|Low)`）：
  零匹配；
- 可接受的保留匹配：私有实现经 `ptImpl` 访问的私有字段（约 90 处）；
  `motor_state_t` 运行时状态结构体类型名；观测器公共输入
  `foc_position_input_t.tCurrent`；负向封装测试夹具本体。

### 4.2 host 完整矩阵（GCC）

默认 + `STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2'`：
encapsulation PASS、float PASS 0 failures、fixed PASS 0 failures（两轮同）。

### 4.3 目标构建（`mingw32-make SW_ROOT=D:/software`）

| 目标 | 配置 | 结果 | text / data / bss |
|---|---|---|---|
| at32f413（默认） | debug float | OK | 67036 / 2296 / 4904 |
| at32f407 | debug | OK（无 FOC） | 279496 / 2316 / 48440 |
| stm32g431 | debug float | OK（含 FOC、无 grblhal） | 80692 / 764 / 9272 |

G431 验证：编译命令行含 `-DFOC_SUPPORT=1`，0 次 grblhal 引用，28 个
`foc/*.o` 参与编译，ELF 含 63 个 `foc_/motor_` 符号。

### 4.4 对照设计终审

公共操作全 API 化 ✓；状态迁移 FSM 独占（仅 Init/Reset/EmergencyStop
三处规范特许写点）✓；实时路径确定性（高频无日志/动态内存/阻塞）✓；
双源限制启动期强制 ✓；切换失败默认急停不退回 ✓；诊断可读且门控 ✓；
延后特性仅文档备注 ✓。已知小偏差：4 个 `motor_Set*Reference` 返回
`void`（设计草案为 `foc_result_t`），属可靠的暂存写入，如实记录。

## 5. 遗留问题

1. stm32g431 FOC ISR 未接线（用户决定），运行时调度留待后续；
2. stm32g431 无启停按钮（MDI 无 `ptButtonStart`），按钮功能编译为空
   操作，shell `motor` 命令不受影响；需要时在 `peripheral/stm32g431/`
   定义引脚并定义 `MDI_HW_HAS_BUTTON_START`；
3. `motor_snapshot_t.qVbus` 保留字段恒 0（HAL 无母线电压回调）；
4. 计划文档 Task 10 Step 3 正文仍写"默认目标 at32f407"（与事实
   at32f413 不符），未改正文；
5. 未 stage/commit；`third_party/grblhal/core` 与
   `vendor/cortex-m/AT32F403A_407_Firmware_Library` 子模块既有改动保持
   原样；`tests/foc/*.exe` 生成物改动未 restore。

## 6. 后续建议

全部计划任务已收官。若需固化成果，下一步是用户授权后的提交（建议按
Task 7 / Task 8+临界区 / Task 9 / Task 10+G431 分批或单次提交）；硬件
实机验证（at32f413 电机板开环拖动 → Hall/SMO 闭环接管）尚未进行。
