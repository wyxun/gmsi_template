# FOC Motor 重构 Task 7 交接

日期：2026-07-19

## 1. 恢复位置

- 仓库：`E:\Project\modus_template`；
- 分支：`master`；
- 基线提交：`aae829b task 6`（Task 7 改动在工作区，未提交）。

仓库内交接资料：

- `docs/superpowers/specs/2026-07-18-foc-motor-object-design.md`；
- `docs/superpowers/plans/2026-07-18-foc-motor-object.md`；
- `docs/superpowers/handoffs/2026-07-18-foc-motor-task5-handoff.md`；
- `docs/superpowers/handoffs/2026-07-19-foc-motor-task7-handoff.md`（本文档）。

## 2. 已完成范围

1. Task 1：封装测试护栏；
2. Task 2：不透明 `motor_handle_t` 和 snapshot；
3. Task 3：统一位置源；
4. Task 4：命令邮箱和生命周期 FSM；
5. Task 5：开环、高频电流控制与低频级联控制；
6. Task 6：target-only 开环到闭环位置源接管；
7. Task 7：稳定诊断快照和固定容量数字事件环。

尚未开始：Task 8–10。

## 3. 重要发现：Task 6 基线实际已损坏

`aae829b task 6` 中已混入部分 Task 7 骨架（事件环、扩展快照、
`motor_DebugReadEvent`），但处于损坏状态：

- `sizeof(motor_impl_t)` 实测 536 > 512；`motor_Init` 只 `memset` 512 字节
  却按 536 字节布局写字段，栈破坏导致 float 套件以 0xC0000005 崩溃；
- production `_Static_assert` 被错写成 `<= 1024U`，失去保护。

Task 7 的实际工作量是：压缩私有布局回到 512 以内、修正断言、修正测试
中的事件读取顺序错误、补齐事件/环形缓冲测试覆盖。前一份交接中
"`motor_impl_t == 512`" 的描述与实际提交不符，以此文档为准。

## 4. 私有布局压缩（536 → 496，余量 16 字节）

1. 事件记录 12 → 8 字节（环 48 → 32）：
   - `uint32_t wSequence`：独立单调事件序号（不是毫秒、不是高频样本索引）；
   - `uint16_t hwPayload`：FAULT=故障位低 16 位；命令=`cmd | result<<8`；
     有效性/切换=`old | new<<8`；timeout=`phase | IDLE<<8`；
   - `uint8_t chType` + `uint8_t chMeta`（from 状态 3bit | to 状态 3bit |
     detail/role 2bit，`MOTOR_EVENT_META*` 宏打包/解包）。
2. 枚举收窄为 `uint8_t`：`chStartupPhase`、`chPendingCommand`、
   `chMechanicalValidFlags`。
3. bool 位域化：5 个不取地址的标志打包进 1 字节；
   `bHigh/LowFrequencyStepInProgress` 因被取地址保留 `bool`。
4. 按对齐重排字段（指针 → 4 字节 → 2 字节 → 1 字节），消除内部填充。

最终 `sizeof(motor_impl_t)`：float 496 / 512，fixed 496 / 512
（host 64 位为最劣情形，32 位目标只会更小）。公共 handle 保持 512 字节
ABI 不变。`_Static_assert` 已修正回 `MOTOR_HANDLE_STORAGE_SIZE`，另有
`motor_TestGetImplementationSize()` 运行时断言双保险。

命名约定：前缀与位宽严格对应——`w`=uint32_t、`hw`=uint16_t、
`ch`=uint8_t、`b`=bool。骨架中的 `uint16_t wPayload` 已改名 `hwPayload`。

## 5. 诊断快照（motor_snapshot_t）

字段单位与 source-role 语义已冻结在 `motor_types.h` 注释中：

- active = 当前控制使用的角度；candidate = 资格判定/融合期间的目标源；
  open-loop = 独立跟踪的启动角；
- 生命周期状态、启动子状态、控制模式、pending command；
- active/candidate 源 validity flags（决定源字段是否有意义）；
- 开环/active/candidate 角、角误差 `qAngleError`、融合系数 `qBlendFactor`；
- D/Q 引用与反馈、D/Q 电压、相电流、占空比、`qVbus`；
- 校准状态与校准值 `tCurrentCalibration`、`bPwmEnabled`、故障位 `wFaults`；
- 事件序号与覆盖计数。

快照在 `motor_sync_if_t` 临界区内一次性相干复制，不返回任何私有指针；
host 无 sync 绑定继续可用。

已知冗余：`tElectricalAngle/qElectricalSpeed` 与
`tActiveAngle/qActiveSpeed` 目前同源，是否精简留给后续任务。

## 6. 事件环

- 容量 4，overwrite-oldest，记录覆盖计数；
- 类型 `motor_event_type_e`：`COMMAND_ACCEPTED`、`COMMAND_REJECTED`、
  `STATE_CHANGED`、`SOURCE_VALIDITY_CHANGED`、`TRANSITION_STARTED`、
  `TRANSITION_COMPLETED`、`TRANSITION_TIMEOUT`、`FAULT`；
- 读取 API：`bool motor_DebugReadEvent(motor_handle_t *, motor_event_t *)`，
  FIFO 顺序、sequence 单调；
- 实时路径只写紧凑记录，无字符串格式化/日志；控制路径不依赖消费事件环；
- FAULT payload 为 `wFaults` 低 16 位（当前 5 个故障位足够），超过 16 个
  故障位时需扩展记录（已在 `motor_private.h` 注释声明）。

## 7. 测试覆盖

- `tests/foc/test_motor.c`：事件环 FIFO / overwrite-oldest / 序号单调
  （6 次 fault 事件覆盖 2 条）；
- `tests/foc/test_motor_fsm.c`：BUSY 拒绝事件、序号 1→4 单调、
  TRANSITION_TIMEOUT 事件（drain 后末条）；
- `tests/foc/test_motor_control_runtime.c`：`test_transition_events`
  （VA/VC/TS/TC 四事件顺序、role、payload、序号连续）；timeout 分支
  TIMEOUT 事件断言。

切换开始/完成与源有效性事件由高频控制路径产生，测试放在
`test_motor_control_runtime.c`（只有该 harness 能驱动真实 transition），
而非计划字面指定的两个文件。

## 8. 最后验证命令与结果

```powershell
$env:Path = 'D:\software\msys64\mingw64\bin;' +
            'D:\software\msys64\usr\bin;' + $env:Path

& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    -C tests/foc SHELL=cmd.exe CC=gcc clean all

& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' `
    -C tests/foc SHELL=cmd.exe CC=gcc `
    STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2' clean all
```

2026-07-19 结果：

- encapsulation：PASS；
- float：PASS，0 failures；
- fixed：PASS，0 failures；
- O2 strict-alias float/fixed：PASS；
- `git diff --check -- foc tests`：PASS（仅 LF/CRLF warning）。

不要使用 clang；统一使用 GCC。

## 9. 工作区注意事项

- `tests/foc/foc_test_float.exe` 是 tracked 生成物，测试后显示 modified；
  `foc_test_fixed.exe` 实际是 untracked（与前一份交接描述不符）。未得到
  用户授权前不要 `git restore/checkout`。
- 不要提交 modus 子模块内容变化。
- 没有 stage 或 commit。

## 10. 当前已知断点（同前，Task 8 范围）

完整目标固件暂时不能编译：

- `foc_app.c` 仍直接访问旧 `tRt/tCurrent` 成员；
- `phase_test.c` 仍调用已撤除的低层公开 API；
- app 仍使用旧状态名和旧控制 API；
- app config 尚未填写控制周期、sync/time 和 run config。

不要为临时编译重新暴露 motor 成员或恢复 `motor_Enable/SetDuty` 公共接口。
README 仍含旧 API 示例，留给 Task 9。

## 11. 下一步：Task 8

迁移 `foc_app`、Shell、button、ISR 和 phase diagnostic，恢复目标构建。
先跑 `rg -n "->tRt|->tControl|->tCurrent|->tParams|foc_ipark|foc_svpwm|motor_Enable|motor_SetDuty|motor_SampleCurrent" foc/app`
记录现状，迁移后该命令应无匹配（显式门控的诊断代码除外）。
