# FOC Motor 对象重构交接文档

> 日期：2026-09-08  
> 工程：`D:\2_xundoc\project\modus_template`  
> 目的：在没有硬件的情况下完成软件迁移和自动化验证，硬件回来后继续完成台架回归。

## 1. 当前结论

当前 AS5600 速度闭环已经接入 `motor_t` 的控制路径，同时保留原有数值口径和
高频安全边界。电压、CURRENT、SPEED 三种控制模式都经过 App 到 Motor 的 wrapper。

编码器电气零位标定目前仍由 `foc_app` 的非阻塞服务承载，尚未完全下沉到 Motor。
这是有意保留的安全边界：没有硬件时先锁定软件数据流，避免同时改变对齐电流、等待
时间和零位捕获时序。

## 2. 已完成内容

### Motor 对象

- 新增 `foc/motor/motor.h`、`motor.c`、`motor_params.h`、`motor_position.h`。
- 提供静态内存、单活动位置 provider、ADC/PWM/context 注入接口。
- 提供生命周期：初始化、空闲、ADC 校准、运行、故障。
- 提供高频步、1 kHz 时钟步、后台步、Start/Stop、参考量和状态/反馈查询。
- 速度 PI 只在 SPEED 模式运行；CURRENT 模式的固定 Iq 不被速度环覆盖。
- 故障路径先调用 PWM emergency stop，再锁存故障状态。

### Core 和 App

- `foc_core_state_t` 不再保存角度、速度和三相原始输入的重复副本。
- `foc_app_t` 初始化并持有 `motor_t`。
- App 位置适配器支持开环角度和编码器反馈。
- 极对数、方向反转和电气零位换算保持现有口径。
- 现有 `foc_app_*` 对外 API 保持不变。

### AS5600

- 新增 `g_tAs5600PositionOps`。
- 1 kHz 慢速路径调用 I2C 更新缓存。
- 高频路径只读取缓存，不访问 I2C、不阻塞。
- provider 测试覆盖缓存读取、极对数电角度换算和总线访问隔离。

### 接口上下文

ADC/PWM 回调统一增加 `void *pContext`。当前 STM32G431 单板可以继续使用 `NULL`
上下文；主机测试已使用独立上下文验证多实例隔离。

## 3. 已验证结果

工具链路径：

```powershell
$env:Path = "D:\0_software\msys64\mingw64\bin;D:\0_software\msys64\usr\bin;$env:Path"
```

完整主机测试：

```powershell
mingw32-make -C D:\2_xundoc\project\modus_template\tests\foc minimal
```

结果：core、encoder、App lifecycle、Motor、AS5600 provider 的 float/fixed 测试均为
`PASS (0 failures)`。

固件构建：

```powershell
$env:MAKE_EXE = 'D:\0_software\msys64\mingw64\bin\mingw32-make.exe'
.\make.bat BUILD=debug
```

最近一次结果：

```text
text 61684
data   640
bss  18952
dec  81276
```

边界检查：

```powershell
git diff --check
rg -n "as5600|I2C|mdebug|printf|perfc_task_pt|HAL_" foc/motor/motor.c
```

`motor.c` 未发现具体传感器、I2C、HAL、日志或任务依赖。`git diff --check` 没有空白
错误；Git 可能提示工作树文件的 LF/CRLF 转换警告，该警告不是代码错误。

## 4. 用户提供的硬件基线

以下数据来自用户 2026-09-07 的实测，当前没有重新测量：

| 项目 | 基线 |
|---|---|
| cal offset 重复性 | 0.4597 / 0.4630 / 0.4648 / 0.4614，极差 0.0051 turn |
| cal 对齐电流 | Id ref 0.10 → actual 0.110，Vd=0.108 |
| 100 eHz 5 s 波形 | mean 98.5 eHz，std 4.9，范围 95-105 |
| Id / Iq | 约 0 / ±0.06，残留 ±0.2 纹波，约 200-450 Hz，无高频声 |
| 100 eHz 上限 | 12 V 可稳定运行，BEMF 约 2.1 V |

必须保留的限制：5-30 eHz 的不平滑来自 AS5600 1 kHz 量化，误差约 ±1.7 eHz；
50+ eHz 为平滑工作区；1 kHz 更新台阶属于正常现象。

## 5. 下一步任务

### 优先级 P0：无硬件即可完成

1. 将编码器零位标定状态机逐步下沉到 Motor。
2. 添加 fake provider 标定测试：
   - 对齐开始；
   - ADC 校准完成；
   - 对齐等待时间；
   - 位置无效；
   - 超时；
   - 捕获电气零位；
   - 急停和故障恢复。
3. 清理 `foc_app_t` 中不再参与控制的重复 Core/PID/位置运行状态。
4. 复查 `foc_app_GetStatus()`、波形通道和 Shell 查询是否只读取 Motor 快照。
5. 继续运行 float/fixed 主机测试和 ARM debug/release 构建。

### 优先级 P1：硬件回来后完成

1. 重复四次 cal offset 测试。
2. 复测 Id=0.10 对齐电流和 Vd。
3. 复测 100 eHz、5 秒速度波形。
4. 复测 Id/Iq 纹波、声音和 12 V 下 100 eHz 稳定性。
5. 如果数据回归，先增加可复现测试和数据流检查，不要直接修改 PI 增益。

## 6. 当前已知注意事项

- 不要在 `motor_HighFrequencyStep()` 或其调用链加入 I2C、日志、阻塞等待或任务 PT。
- 不要为了消除 AS5600 台阶而在 20 kHz 路径增加未经评审的插值。
- 不要把用户现有 `makefile`、`modus`、`diagrams/` 修改重置或覆盖。
- 当前工作树没有提交；不要执行 `git reset --hard`、`git checkout`、分支切换、提交
  或推送，除非用户明确要求。
- 硬件回归前不要刷写设备，也不要把软件仿真结果写成 offset、BEMF 或噪声实测。

## 7. 重要文件索引

- 设计：[2026-09-08-foc-motor-object-refactor-design.md](../specs/2026-09-08-foc-motor-object-refactor-design.md)
- 计划：[2026-09-08-foc-motor-object-refactor.md](../plans/2026-09-08-foc-motor-object-refactor.md)
- Motor：[motor.h](../../../foc/motor/motor.h)、[motor.c](../../../foc/motor/motor.c)
- App：[foc_app.h](../../../foc/app/foc_app.h)、[foc_app.c](../../../foc/app/foc_app.c)
- AS5600：[as5600.h](../../../peripheral/driver/as5600.h)、[as5600.c](../../../peripheral/driver/as5600.c)
- 主机测试：[test_motor.c](../../../tests/foc/test_motor.c)、[test_as5600.c](../../../tests/foc/test_as5600.c)、[test_foc_minimal_lifecycle.c](../../../tests/foc/test_foc_minimal_lifecycle.c)

