# FOC 极简框架上电验证测试指南

> 配套：`docs/HANDOFF-2026-08-11-foc-minimal-refactor.md`（进度）
> 环境：STM32G431 + AS5600 编码器 + 12V 母线 + 2205 电机（7 对极）

## 0. 上电准备

```powershell
# 烧录 + RTT（float 版默认；fixed 需 FOC_NUMERIC=fixed）
.\make.bat auto TARGET_CHIP=stm32g431 BUILD=debug-rel
```

RTT Shell 恢复（若 probe 掉线）：kill openocd → 重新插拔 probe → 手动启动 `openocd -s D:/software/msys64/mingw64/share/openocd/scripts -f target/stm32g431/openocd.cfg -f tools/rtt_server_manual.cfg`。

上电后先确认：

```text
motor status        → state=IDLE faults=0x0 pwm=0 calib done=1
encoder             → valid=1 MD=1 ML=0 MH=0 => ok（磁铁正常）
```

---

## 1. 指令清单（当前固件可用）

| 指令 | 模式 | 角度源 | 用途 | 等价旧框架 |
|---|---|---|---|---|
| `motor start` | VOLTAGE | 开环 2 eHz ramp | 电压自同步启动 | `motor start` |
| `motor lock [turns]` | VOLTAGE | 开环固定角度 + Vd | **静态锁电机** | `motor_verify static <turns>` |
| `motor spin <hz> <vq>` | VOLTAGE | 开环旋转 | 电压开环旋转测试 | `motor_verify run <hz> <vq>` |
| `motor current <iq> <hz>` | CURRENT | 开环旋转 | **电流环测试** | `motor_verify current <iq> <hz>` |
| `motor enc <iq> <hz>` | SPEED | 编码器 | 速度闭环 | `motor enc` |
| `motor vd <x>` / `motor vq <x>` | VOLTAGE 在线 | — | 调 Vd/Vq（pu） | `motor vq <x>` |
| `motor stop` | — | — | 先急停再回 IDLE | `motor stop` |
| `motor clear` | — | — | 清故障 | `motor clear` |
| `motor status` | — | — | 全状态一次快照 | `motor status` |
| `encoder` | — | — | **读编码器位置/状态** | `encoder` |
| `encoder cal` | CURRENT Id 电流对齐 | 开环电角 0 | **电气零位标定**（Id=0.10 吸转子到 D 轴） | `encoder cal` |

安全钳位（shell 层）：`|Vd|,|Vq| ≤ 0.30 pu`；`|speed| ≤ 100 eHz`；`Iq ∈ [0.02, 0.15] pu`。

---

## 2. 验证测试指令与验收标准（按顺序执行）

### T1 编码器可读（无电）

```text
encoder
```

| 验收 | 标准 |
|---|---|
| raw 值 | 转轴时 raw 连续变化（0~4095），不跳变 |
| valid | `valid=1`，`status` 中 MD=1（检测到磁铁） |
| ML/MH | 均 0（磁场强度正常）；任一为 1 → 检查磁铁安装 |
| 每次上电 | raw 随物理位置固定（绝对式，无上电歧义） |

### T2 静态锁电机（电压 SVPWM 定点停住）

```text
motor lock 0          # 电角度 0，Vd=0.04
motor status          # 确认 state=RUNNING pwm=1
```

| 验收 | 标准 |
|---|---|
| 上电时序 | `motor lock` 后 ~25.6ms 校准（PWM 关闭）→ RUNNING → PWM 使能 |
| 转子行为 | 转子对齐到 Vd 场并**静止保持**，无啸叫、无转动（需转子先在电角 0 附近，漂移后 Id 反灌属预期） |
| 电流 | `status` Iq≈0，Iu/Iv/Iw 三相和≈0（Vd 直流量） |
| 抗扰 | 手动轻拨转子，回中（弹簧效应），不振荡发散 |
| 变角 | `motor stop` → `motor lock 0.25` → 转子转到 90° 电角停住 |

### T3 电压开环旋转（自同步启动）

```text
motor spin 2 0.03     # 2 eHz, Vq=0.03 → 启动
motor vq 0.05         # 在线升压
motor spin 10 0.05    # 提高速度
motor stop
```

| 验收 | 标准 |
|---|---|
| 启动 | 1~2 s 内自同步旋转，无失步震荡 |
| 方向 | `motor spin -2 0.03` 反转 |
| 转速 | 转速随开环速度单调（status speed ≈ 设定 eHz） |
| 停止 | `motor stop` 立即急停，无尾转 |

### T4 电流环测试（改造前核心回归项）

```text
motor current 0.05 2    # Iq=0.05pu, 开环 2 eHz 旋转
motor status            # 关键：Iq ref vs actual
```

| 验收 | 标准 |
|---|---|
| **Iq 跟踪** | `Iq` 实际值 ≈ 参考 0.05 pu，误差 < 10%（±0.005） |
| Id | `Id` ≈ 0（±0.005），无耦合拖尾 |
| 旋转 | 电流环在开环角度下平稳旋转，无转矩脉动暴走 |
| 变参 | `motor stop` → `motor current 0.10 2` → Iq 跟踪 0.10 |
| 低速量化 | 2 eHz 下 angle 平滑（外推生效，无 18° 阶梯） |

### T5 速度闭环（编码器）

```text
encoder cal           # 电气零位标定（Id 电流对齐转子到 D 轴，勿碰轴，约 1.5s）
motor enc 0.10 50     # 速度目标（eHz）
motor status          # speed 字段
```

| 验收 | 标准 |
|---|---|
| 标定 | `encoder cal` 打印 offset；重复 cal offset 极差 < 0.143 turn（实测 0.0051） |
| 启动 | 校准后正常进 RUNNING，速度环接管 |
| 速度 | `status` speed ≈ 目标（±5%），无跑飞（实测 100 eHz：mean 98.5 ±5%） |
| 反转 | `motor stop` → `motor enc 0.10 -50` 反转 |
| 震动 | 电机平稳，无"只震不转"、无 20 kHz 啸叫 |
| 低速限制 | 5-30 eHz 不平滑为 AS5600 1 kHz 量化固有（±1.7 eHz 占 17-34%），非控制缺陷 |

### T6 安全路径（每轮必测）

```text
# 运行中随时 stop
motor current 0.05 2
motor stop            # 立即急停

# 故障恢复
motor status          # 若 faults≠0（如采样失败）
motor clear           # FAULT 且 PWM 关闭时 → IDLE
motor start           # 可重新启动
```

| 验收 | 标准 |
|---|---|
| STOP | 任何状态先关 PWM 再回 IDLE，无延迟 |
| FAULT | 故障后 pwm=0（急停已执行），clear 后 IDLE |
| 校准偏移非法 | 零偏超 [20000, 60000] → FAULT_CALIBRATION（上电漂移保护） |
| 编码器拔除 | `motor enc` 运行中磁铁移除 → FAULT_ANGLE 急停 |

---

## 3. 波形验证（float 版，MStudio Waveform）

10 路通道：`Iu/Iv/Iw/Id/Iq/Angle/Speed/Vd/Vq/EncMech`。其中 `EncMech` 是 AS5600 机械角度（turn），按 1 kHz 更新。Speed 通道 scale=100（电速度可显示到 ±100 eHz；scale=1000 时 >32.8 eHz 会 int16 饱和卷绕成 ±32.7 假象）。

| 场景 | 波形判据 |
|---|---|
| 静态锁角 | Id≈0，Iq≈0，Vd 直流 0.04，Angle 恒定 |
| 电流环 | Iq 跟踪收敛无振荡；EncMech 显示机械角变化 |
| 速度闭环 | Speed 稳定在 ~100（±5%）；Angle 连续无阶梯；Iq 稳态小纹波 |

## 3a. 实测记录（2026-09-07，motor enc 0.05 100）

| 项 | 实测 |
|---|---|
| cal offset 重复性 | 0.4597/0.4630/0.4648/0.4614，极差 0.0051 turn |
| cal 对齐电流 | Id ref 0.10 → actual 0.110，Vd=0.108（电流环跟踪 ✓）|
| Speed（5s 波形）| mean 98.5 eHz，std 4.9（±5%），范围 95-105 |
| Id / Iq | ≈0 / ±0.06（残留 ±0.2 纹波，~200-450 Hz，无高频声）|
| 100 eHz 上限 | 修正：12V 可稳定跑（BEMF≈2.1V）；此前"48 eHz"作废 |

已知限制：5-30 eHz 低速不平滑 = AS5600 1 kHz 量化（±1.7 eHz 占 17-34%）固有；50+ eHz 平滑（量化 <5%）。Speed 波形呈 1 kHz 更新台阶为正常现象。

---

## 4. 验收汇总表

| # | 项 | 通过标准 |
|---|---|---|
| 1 | 编码器可读 | valid=1，raw 随轴连续变化 |
| 2 | 静态锁电机 | Vd 场对齐静止保持，变角跟随（需转子先在电角 0 附近）|
| 3 | Id 电流零位标定 | offset 极差 <0.143 turn（实测 0.0051）|
| 4 | 电流环 | Iq 跟踪误差 <10%，Id≈0 |
| 5 | 速度闭环 | **100 eHz** 稳定 ±5%（实测 mean 98.5 ±5%），无跑飞无震动 |
| 6 | 急停/故障 | STOP 立即关 PWM；故障先急停再 FAULT |
| 7 | 波形 | 10 路直读变量无丢帧，EncMech 按 1 kHz 更新 |

遗留（不阻塞）：低速不平滑（量化固有）、Id/Iq ±0.2 纹波（电流环 PI 待调）、波形时间列损坏（工具侧）、整体提交未收口。
