# FOC 极简重构电流环与速度环验证计划

> 本计划只定义验证流程，不预先修改控制参数。执行时保持母线测试窗口短，每一步完成后根据证据决定是否进入下一步。

**目标：** 证明极简 FOC 重构后，电流环能稳定跟踪，速度环能利用 AS5600 编码器闭环运行，并且电机实际连续旋转。

**范围：** STM32G431、12 V 母线、2205 电机、7 对极、AS5600；基线为 `727f8bb2fbba9ec76cace5512d28` 的外设初始化、CH4 触发和三相相序。

**非目标：** 在证据不足时调整 ADC、TIM1、CH4、三相相序或一次修改多个控制参数；不把 `Id/Iq` 有数值或 `RUNNING` 状态单独当作电机已转动。

## 当前已知基线

- 已验证的启动修复位于 `foc/app/foc_app.c`：START 前先恢复 ADC 触发，避免急停关闭 CH4 后无法产生 ISR。
- 回归测试位于 `tests/foc/test_foc_minimal_lifecycle.c`，float/fixed 主机测试已通过。
- 已经观察到 `motor current 0.05 2` 可进入 `RUNNING`，但现场未确认电机连续机械旋转；这只说明电流控制链路有响应。
- 曾在测试结束、母线关闭后读到一次 AS5600 `ML=1`，该样本不用于判断测试期间的编码器有效性；编码器有效性必须在本轮上电前重新采集。
- 三相相序保持 727f8bb 基线：U=ADC1 injected rank 0，V=ADC2 rank 1，W=ADC2 rank 0。

## 执行前安全条件

1. 母线关闭时完成源代码核对和 AS5600 静态读数。
2. 母线开启前确认 `motor stop` 已执行；上电后先读 `motor status`，必须是 `state=IDLE`、`faults=0`、`pwm=0`。
3. 一次只保持一个 RTT shell 读取连接，避免输出分片；波形采集使用独立的被动通道。
4. 运行中出现异常声响、明显发热、机械不动但电流持续上升、故障位非零，立即执行 `motor stop`，本轮停止，不继续升压或提速。
5. 不执行 halt、reset、GDB 或寄存器写入；如确需侵入式调试，另行确认。

## 阶段 0：确认软件和硬件测试基线

### 步骤 0.1：源代码只读核对

核对 `foc/app/foc_app.c`、`peripheral/stm32g431/haladc.c`、`peripheral/stm32g431/haltim1.c` 与 727f8bb 的差异。

通过条件：本轮只接受应用层启动/状态逻辑变化；ADC 初始化、TIM1 CH4 触发值、PWM 初始化和三相采样映射均与基线一致。

### 步骤 0.2：主机回归

运行：

```text
cd tests/foc
mingw32-make -f Makefile minimal
```

通过条件：minimal core、minimal encoder、minimal lifecycle 的 float/fixed 测试均为 PASS；否则不进行母线测试。

## 阶段 1：母线关闭时确认编码器前置条件

保持母线关闭、只给 MCU/调试器供电，连续执行三次：

```text
encoder
```

记录每次 `raw`、`valid`、`MD`、`ML`、`MH` 和 `seq`。

通过条件：三次均为 `valid=1`、`MD=1`、`ML=0`、`MH=0`；转轴静止时 raw 基本稳定；手动小角度转轴时 raw 连续变化并可回到原值。

停止条件：任一次 `valid=0`、`ML=1` 或 `MH=1`。此时只检查磁铁与 AS5600 的间隙、同心度和固定，不修改 FOC 算法，也不带母线运行。

## 阶段 2：验证启动链路和静态锁角

### 步骤 2.1：启动前快照

母线开启后依次执行：

```text
motor stop
motor status
encoder
```

要求：`IDLE/PWM=0/faults=0`，并重新确认编码器有效。若编码器状态与阶段 1 不同，以本次上电前快照为准，不引用旧样本。

### 步骤 2.2：锁角 0

执行 `motor lock 0`，只保持短时间观察，然后依次执行：

```text
motor status
motor timing
motor stop
motor status
```

通过条件：

- 启动后进入 `RUNNING` 且 `pwm=1`；
- `motor timing` 的 ISR 周期统计为非零；
- 电机保持在一个静态位置，无持续转动；
- stop 后确认 `IDLE/PWM=0`。

该步骤只验证 PWM/ADC/ISR 和静态 Vd 场，不判定电流环旋转能力。

### 步骤 2.3：锁角 0.25

确认上一步已停机后执行 `motor lock 0.25`，以相同时间窗口读取状态并 stop。

通过条件：相对 `lock 0`，转子位置变化约 90 电角度，即约 `90/7=12.9` 机械角度，并在新位置保持；若只改变软件 angle 而转子没有对应变化，记录为锁角/相位验证失败。

## 阶段 3：电压开环旋转，先隔离相序和机械链路

每次只执行一个速度点，先用低速低压：

```text
encoder
motor spin 2 0.03
```

在运行期间观察电机是否连续旋转；随后执行：

```text
motor status
encoder
motor stop
motor status
```

记录启动前后 raw 差分、运行中的方向、是否有啸叫/振荡以及 status 的 angle 和 speed。

通过条件：AS5600 raw 连续变化且方向与 `motor spin` 符号一致，电机实际连续旋转，无明显失步。通过后才允许执行 `motor spin -2 0.03` 验证反向。

判定分支：

- ISR/PWM 正常但 raw 不变：优先查 Vd/Vq 方向、相序、电流极性或转矩不足；
- raw 变化但电机肉眼不转：检查编码器安装耦合、磁铁是否随转子同步；
- 只锁定或抖动：先停机，不能直接进入速度环。

## 阶段 4：电流环验证

### 步骤 4.1：波形和状态采集

确保编码器前置条件已通过，先执行 `encoder cal`，等待其完成并回到 IDLE，记录 offset 和校准时的 ADC 零偏。然后执行：

```text
motor current 0.05 2
```

同步被动采集 0.5~1 s 波形通道：`Iu/Iv/Iw/Id/Iq/Angle/Speed/Vd/Vq`；观察结束后立即执行：

```text
motor status
encoder
motor stop
motor status
```

### 步骤 4.2：电气验收

通过条件：

- `Iq` 稳态跟踪 `0.05 pu`，误差不超过 `±0.005 pu`；
- `Id` 稳态接近 0，误差不超过 `±0.005 pu`；
- 波形没有持续积分、饱和或明显振荡；
- stop 后 PWM=0。

### 步骤 4.3：机械验收

单独用 encoder raw 的连续变化确认电机是否实际运动。只有“电气验收”和“机械验收”同时满足，才记录为 T4 通过。

若 `Iq` 跟踪正常但 raw 不变，记录为“电流环电气部分通过、机械启动失败”，不修改 PI 参数；下一轮只允许验证一个变量：电气零位、方向极性、相序或电流限幅，不能同时改变。

## 阶段 5：速度环验证

仅在阶段 1、阶段 3 和阶段 4 全部通过后执行。

### 步骤 5.1：低风险速度启动

重新确认 `encoder` 有效，执行 `encoder cal`，完成后先运行较低目标：

```text
motor enc 0.10 10
```

读取 `motor status` 和 `encoder`，确认方向、速度反馈和 Iq 没有异常后 stop。

### 步骤 5.2：目标速度 50 eHz

重新启动：

```text
motor enc 0.10 50
```

采集至少一个稳定窗口的 `Speed/Angle/Iq/Id`，随后 stop 并读取最终状态。

通过条件：

- AS5600 raw 连续变化；
- status 的 speed 稳定在 `50 e-turn/s ±5%`；
- `Id` 接近 0，Iq 未持续顶到限幅；
- 无跑飞、反向、强振荡或故障；
- stop 后 PWM=0。

若 10 eHz 通过而 50 eHz 失败，只调速度环相关变量；若 raw 不动，先回到阶段 4，不调整速度 PI。

## 证据记录格式

每个阶段记录以下最小信息：

```text
母线状态：OFF/ON
命令：
运行前：motor status + encoder
运行中：motor status + timing + 波形文件名
运行后：encoder + motor status（确认 pwm=0）
结论：PASS / FAIL / BLOCKED
下一步：只写一个待验证变量
```

## 修改决策顺序

1. 触发链路失败：只修复启动/触发状态，不动控制参数。
2. 锁角或开环旋转失败：只验证相序、Vd/Vq 方向、电流极性和电气零位。
3. 电气电流环失败：只调整电流采样/PI 对应问题，并先增加主机回归测试。
4. 电流环通过、速度环失败：只检查编码器角度连续性、速度单位和速度 PI。
5. 每次源代码修改都必须先补/更新测试，再运行 float/fixed 测试、构建、烧录和同一阶段硬件复测；不修改外设初始化作为默认约束。
