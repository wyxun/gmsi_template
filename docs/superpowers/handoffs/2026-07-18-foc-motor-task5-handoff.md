# FOC Motor 重构 Task 5 交接

日期：2026-07-18

## 1. 恢复位置

本交接已合并回本地 `master`。后续工作的唯一恢复位置是：

- 仓库：`D:\2_xundoc\project\modus_template`；
- 分支：`master`；
- 功能分支基线：`8fea180 update foc`。

`codex/foc-motor-refactor` 和它的临时 worktree 只用于 Task 1–5 的隔离
开发；合并验证完成后可以清理，不应作为后续恢复位置。

仓库内交接资料：

- `docs/superpowers/specs/2026-07-18-foc-motor-object-design.md`；
- `docs/superpowers/plans/2026-07-18-foc-motor-object.md`；
- `docs/superpowers/handoffs/2026-07-18-foc-motor-task5-handoff.md`。

## 2. 已完成范围

已完成并通过规格、质量双重审查：

1. Task 1：封装测试护栏；
2. Task 2：不透明 `motor_handle_t` 和 snapshot；
3. Task 3：统一位置源；
4. Task 4：命令邮箱和生命周期 FSM；
5. Task 5：开环、高频电流控制与低频级联控制。

尚未开始：Task 6–10。

## 3. 当前公共 motor 模型

### 3.1 不透明对象

`motor_handle_t` 是 512 字节、`max_align_t` 对齐的静态不透明存储。
私有实现只在 `foc/motor/motor_private.h` 中定义。

GCC x64 测试构建下：

- `sizeof(motor_handle_t) == 512`；
- `sizeof(motor_impl_t) == 480`；
- 为 Task 6 强制保留 32 字节；
- production `_Static_assert` 保证实现不超过 `512 - 32`。

私有类型使用局部 GCC/Clang `may_alias` 属性解决声明字节存储的有效类型
问题；没有全局关闭 strict aliasing。

### 3.2 生命周期

公共生命周期状态：

```text
UNINITIALIZED → IDLE → STARTING → RUNNING → STOPPING → IDLE
                         └───────────────→ FAULT
```

外部通过 `motor_Start()`、`motor_Stop()` 提交意图。真正状态迁移只由
`motor_RunFSM()` 执行。`motor_EmergencyStop()` 是唯一立即关闭功率并进入
FAULT 的公共操作。

启动 FSM 当前实现：

```text
CALIBRATE → WAIT_DELAY → ENABLE → COMPLETE
```

校准在 PWM 关闭时执行；延时使用注入的毫秒时钟并支持 `uint32_t` 回绕；
PWM enable/disable 与状态镜像在短同步临界区内原子提交。

### 3.3 命令和同步

- 单槽 mailbox 保存 START/STOP；
- STOP 可以覆盖尚未消费的 START；
- RunFSM 在同步区内原子消费命令并发布 STARTING/STOPPING；
- `motor_sync_if_t` 由目标绑定临界区 enter/exit；
- host 单线程测试允许不绑定 sync；
- HAL enable/disable/duty callback 必须是有界、非阻塞的寄存器操作，且不得
  回调 motor；这些操作在短临界区中完成，以消除急停竞态；
- source/controller callback 在锁外执行；返回后重新验证状态和 pending STOP；
- HF 和 LF 可以并行，但同一 motor 的 HF 不可重入、LF 不可重入；重复调用
  返回 `FOC_RESULT_BUSY`。

### 3.4 安全 Reset

`motor_Reset()` 仅在以下条件同时满足时生效：

- 已初始化；
- IDLE；
- PWM 已关闭；
- 无 pending command。

START pending、RUNNING 或 FAULT 下调用为安全 no-op，不能制造
`IDLE + PWM enabled` 的不一致状态。

## 4. 当前统一位置源

唯一公共抽象为 `foc_position_source_if_t`：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(
        void *pContext,
        const foc_position_input_t *ptInput,
        foc_position_output_t *ptOutput);
} foc_position_source_if_t;
```

旧类型已删除：

- `sensor_interface_t`；
- `observer_interface_t`；
- `foc_observer_if_t`；
- `motor_AttachSensor()`；
- `motor_AttachObserver()`；
- `motor_AttachPositionSource()`。

位置源只能在 `motor_Start()` 的 run config 中绑定。motor 校验接口后复制
descriptor；descriptor 对象本身可为临时值，但其 `pContext` 指向的算法或
硬件对象必须覆盖整个运行期。

已适配：

- Hall：adapter context 持有 Hall 对象、读码回调和硬件 context；
- SMO；
- NLFO；
- observer selector。

输出只有一套 canonical validity flags，不再保留 `bValid` 或
`tAngle/qSpeed` 别名。

位置 helper 已支持：

- 机械角到电角转换；
- direction、pole pairs、机械零位、电角偏移；
- required-mask freshness；
- `uint32_t` 回绕时间戳排序；
- 最短圆周角度误差；
- 浮点与定点全范围安全插值；
- Hall 非法码和非法跳变 fault。

第一版明确拒绝两个不同的非空 initial/target source。支持：

- null/null：纯开环；
- initial/null：直接位置源闭环；
- null/target：开环拖动后接管；
- initial==target：直接闭环。

不要在 Task 6 擅自增加多源注册表或后备链。

## 5. 当前控制路径

### 5.1 周期和单位

`motor_config_t` 显式配置：

- 高频周期：归一化秒；
- 低频周期：归一化秒；
- 开环速度：电角 turns/s；
- 开环加速度：电角 turns/s²；
- 速度环参考：机械 turns/s；
- 位置环参考：单圈机械 turns。

禁止从 `get_system_ms()` 推导高频角度。

### 5.2 高频顺序

```text
进入 HF 非重入 guard
→ 同步区内快照状态和 references
→ 电流采样
→ 更新 direct source 或 target candidate / open-loop generator
→ 机械角到电角转换
→ Clarke / Park
→ 电压直通或 Id/Iq controller
→ inverse Park
→ modulation
→ 单一同步事务：复核 RUNNING/no fault/PWM/no STOP
   + HAL duty write
   + runtime/snapshot 数据提交
→ 退出 guard
```

软件 duty/runtime 和硬件 duty 不再分两次提交。

HAL duty 失败：

- 返回原 HAL error；
- 锁存 `MOTOR_FAULT_HARDWARE`；
- 急停。

算法、位置源或非法控制数据失败：锁存
`MOTOR_FAULT_INVALID_COMMAND` 或对应位置 fault。

### 5.3 开环和 target-only

开环速度斜坡支持：

- 正目标；
- 负目标；
- 正负跨零；
- 减速；
- 越过目标时精确钳位。

target-only SPEED/POSITION 启动时：

- Start 仍要求目标模式所需 controller 全部绑定；
- `bOuterLoopActive == false`；
- HF 使用开环角度并运行 Id/Iq inner loop；
- LF 返回 OK 但不调用 speed/position controller；
- Task 6 完成角度接管后才允许激活外环。

### 5.4 低频级联

- POSITION：单圈机械角 shortest wrapped error，position controller 生成
  机械速度参考；
- SPEED/POSITION：机械速度进入 speed controller，生成 Iq reference；
- 只有电速度时按 pole pairs 安全换算机械速度；
- Id reference 保持应用设置值。

## 6. 关键测试

新增：

- `tests/foc/compile_fail_motor_member_access.c`；
- `tests/foc/compile_pass_motor_type.c`；
- `tests/foc/check_motor_encapsulation.bat`；
- `tests/foc/test_motor_encapsulation.c`；
- `tests/foc/test_motor_fsm.c`；
- `tests/foc/test_motor_position.c`；
- `tests/foc/test_motor_control_runtime.c`。

覆盖内容：

- opaque member negative compile；
- float/fixed；
- strict alias O2；
- 多实例；
- NULL/未初始化 handle；
- 校准、延时、enable/disable failure；
- STOP 覆盖 START；
- callback 内 Stop/Emergency；
- source/controller/HF 重入；
- direct source 和 target-only；
- position wrap；
- 正负/跨零/减速 open ramp；
- sample/source/missing-angle/duty failure；
- 精确 fault 分类。

## 7. 最后验证命令

PowerShell 中必须显式设置 PATH，并为 encapsulation recipe 指定 cmd shell：

```powershell
$env:Path = 'D:\0_software\msys64\mingw64\bin;' +
            'D:\0_software\msys64\usr\bin;' + $env:Path

& 'D:\0_software\msys64\mingw64\bin\mingw32-make.exe' `
    -C tests/foc SHELL=cmd.exe CC=gcc all

& 'D:\0_software\msys64\mingw64\bin\mingw32-make.exe' `
    -C tests/foc SHELL=cmd.exe CC=gcc `
    STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2' all
```

2026-07-18 最后结果：

- encapsulation：PASS；
- float：PASS，0 failures；
- fixed：PASS，0 failures；
- O2 strict-alias float/fixed：PASS；
- `git diff --check`：PASS（可能显示 LF/CRLF warning）。

不要使用 clang；用户已明确统一使用 GCC。

## 8. 工作区注意事项

- `tests/foc/foc_test_float.exe` 和 `foc_test_fixed.exe` 是 tracked 生成物，测试
  后显示 modified；没有得到用户授权前不要用 `git restore/checkout`。
- `modus` 子模块在隔离 worktree 中已初始化；perf_counter 内容曾从原工作区
  同步以补齐嵌套依赖。不要提交子模块内容变化。
- 没有 stage 或 commit。
- 设计、实施计划和本交接文档都应保留在合并后的本地 `master`。

## 9. 当前已知断点

完整目标固件暂时不能编译，原因是 Task 8 尚未迁移 `foc/app`：

- `foc_app.c` 仍直接访问旧 `tRt/tCurrent` 成员；
- `phase_test.c` 仍调用已撤除的低层公开 API；
- app 仍使用旧状态名和旧控制 API；
- app config 尚未填写控制周期、sync/time 和 run config。

不要为临时编译重新暴露 motor 成员或恢复 `motor_Enable/SetDuty` 公共接口。

README 也仍含旧 API 示例，留给 Task 9。

## 10. 下一步：Task 6

Task 6 只实现 initial NULL + target source 的开环到闭环接管：

1. 为 candidate source 输入提供可靠 timestamp。当前 position input timestamp
   为 0，表示调度器没有提供时钟；Task 6 必须决定使用 time interface 或内部
   sample counter，不能把 0 当新鲜样本。
2. 连续资格判定：valid flags、fault-free、freshness、confidence、最低速度、
   方向一致、最大角度误差、连续样本数。
3. 高频最短路径角度融合。
4. 融合中保持 Id/Iq reference 连续。
5. 失败或超时默认急停，不实现自动退回开环。
6. 接管完成后设置 `bOuterLoopActive = true`，再启用 SPEED/POSITION 外环。
7. 需要在 production 32 字节余量内实现；若不足，先压缩/复用 FSM scratch，
   不扩大 512 字节 public handle。
8. 不允许两个不同的非空 source，不实现 source registry/fallback/flying start。

Task 6 开始前先重新运行第 7 节两组测试，确认接手环境一致。

## 11. 后续 Task 7–10

- Task 7：固定容量数字事件记录、稳定诊断快照；容量必须可裁剪，不能破坏
  32 字节 Task 6 预算。
- Task 8：迁移 `foc_app`、Shell、button、ISR 和 phase diagnostic；恢复目标构建。
- Task 9：重写 README 四种用法与位置源 adapter 文档。
- Task 10：host matrix、默认目标、AT32F413、STM32G431 全验证。
