# FOC Motor 重构 Task 9 交接

日期：2026-07-19

## 1. 恢复位置

- 仓库：`E:\Project\modus_template`；
- 分支：`master`；
- 基线：`aae829b task 6` + Task 7/8/9 工作区改动（未提交，未 stage）。

前置交接：

- `docs/superpowers/handoffs/2026-07-19-foc-motor-task7-handoff.md`；
- `docs/superpowers/handoffs/2026-07-19-foc-motor-task8-handoff.md`。

## 2. 已完成范围（Task 9：公共文档与扩展说明）

计划 5 个步骤全部完成，复选框已勾选。Task 1–9 全部完成，剩余 Task 10
（全验证矩阵）。

## 3. 变更内容

### 3.1 `foc/README.md` 全面重写（993 → 1651 行）

用户核心要求：清晰抽离两条接口边界，让使用者能简单移植 FOC 库并在其上
实现电机控制。新结构：

1. 能力与边界；
2. 目录依赖；
3. 数值/单位/角度；
4. **移植总览：两条接口边界**——输入（平台要实现什么）/输出（应用能
   调用什么）两侧总览表 + 9 步最小移植清单；
5. **硬件层接口（输入侧）**——`foc_pwm_if_t`、`foc_adc_if_t`、
   `motor_sync_if_t`、`motor_time_if_t`、位置源硬件回调；每项含逐字
   签名、调用上下文（ISR/主循环/临界区）、时序要求、调用频率；附频率
   总表和 MDI 参考适配器说明；
6. **应用层接口（输出侧）**——每个 motor API 的输入/输出/语义/单位；
   快照 24 字段表；事件环解码表；`motor_config_t`/`motor_run_config_t`
   字段表与组合规则；故障与返回值表；
7. 四种场景示例（公共骨架 + 电压开环 / 开环角电流闭环 / Hall 直接闭环 /
   开环→SMO 接管），所有签名逐字核对头文件与 `foc_app.c`；
8. 统一位置源适配（有效位填法表、机械→电转换归属、MDI/HAL 边界）；
9–12. 控制器/调制/观测器/优化（更新为新适配器）；
13. 实验特性与硬件诊断（`FOC_DIAGNOSTIC` 门控）；
14–18. 多电机/整定/检查表/构建/FAQ；
19. 延后扩展项（flying start、position manager、闭环降级、运行中模式
    切换、在线整定、扩展诊断 FSM——仅触发条件与实现方向，与设计文档
    §15 对齐，未新增休眠枚举/状态/API）；
20. 接口索引。

### 3.2 `foc/foc.h`

删除冗余的 `#include "motor/motor_control.h"`（该头为内部实现头，公共
声明已由 `motor.h` 覆盖），加注释说明；其余导出不变。原本就不含
`observer_lib`/旧 `motor_Control*` 符号。

## 4. 验证结果

- README 旧 API 搜索：29 处 → 9 处，剩余全部为**公共类型字段**
  （`motor_config_t.tControl/.tParams`、`motor_run_config_t
  .tCurrentReference`、`motor_snapshot_t.tCurrent`），与 `foc_app.c`
  真实写法一致，零私有布局访问；该正则无法区分公共/私有字段，字面
  "为空"在文档化公共 API 的前提下不可达；
- `rg "observer_lib|motor_Control" foc/foc.h`：前后均为空；
- 因改动 `foc.h` 重跑 host 矩阵：encapsulation / float / fixed 全 PASS，
  0 failures。

## 5. 遗留问题

1. README 示例中的 `board_BindFocHal()`、`board_ReadHallCode()` 等为明确
   标注的平台/产品占位符（参考实现指向 `foc_hal_mdi_BindDefault`），
   示例是"可编译风格"而非开箱即编译；
2. `motor_snapshot_t.qVbus` 从未被写入（HAL 无母线电压回调），README
   已如实标注保留恒 0；
3. `docs/` 下其余历史文档未检查旧 API 表述，超出本任务范围。

## 6. 下一步：Task 10（全验证）

1. `git diff --check` + 陈旧符号搜索（计划 Step 1 命令）；
2. host 完整矩阵（encapsulation / float / fixed / strict-alias）；
3. 默认目标构建（注意：Makefile 实际默认 target 是 at32f413，
   AGENTS.md 写的 at32f407 已过时；`make.bat` 硬编码 `D:\0_software`
   不存在，本机需 `mingw32-make SW_ROOT=D:/software`）；
4. at32f413 / stm32g431 目标构建（stm32g431 的 FOC ISR 接线仍是
   Task 8 遗留 TODO，构建若失败需先补 `stm32g431_it.c` 高频入口）；
5. 对照设计文档终审最终 diff。
