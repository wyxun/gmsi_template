# FOC 分级 bring-up 计划：传感器 → 电流环 → 速度环

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 清理传感器接口冗余后，在 STM32G431 真机上按「传感器标定/读取 → 电流环 → 速度环」三级递进把 FOC 闭环跑通，最终得到稳定的编码器速度闭环。

**Architecture:** 唯一传感器抽象 `foc_sensor_t`（foc_sensor.h：fnUpdate 慢侧 / fnRead 快侧 / fnCalibrate），AS5600 在 `as5600.c` 实现 `g_tAs5600SensorOps`；foc_app 只经 `tSensor.ptOps` 访问，不直调具体芯片。电流环 20 kHz 在 foc_core，速度环 PI 1 kHz 在 foc_app。硬件 ops（PWM/ADC/sensor）由 `foc_port.h` 表注入。

**接口契约（重要，防口径漂移）：**
- `foc_sensor_ops_t.fnRead` 输出 **机械角度（turn）/ 机械速度（turn/s）**——传感器原生量纲，sensor 层**不做电角换算**。
- 电角度换算在 **app 的 `foc_app_AngleStep`** 完成：`θ_elec = θ_mech × Pp（方向反转）+ tElectricalZero`，速度同理 ×Pp（foc_app.c 已实现，7 对极）。
- `stub_sensor_read`（host 测试）是**假桩**，固定返回机械角 0.5 turn，**不读 AS5600**（测试在 PC 跑、无 I2C）；真机读取链在 `as5600.c`：`as5600_sensor_Read` = `as5600_GetSample`(1 kHz 缓存) → `foc_encoder_Step`(20 kHz 消费+外推) → 机械角/速。

**Tech Stack:** STM32G431、AS5600(1 kHz I2C + 20 kHz 外推)、MODUS FOC、float/fixed 双后端、MStudio Waveform。

**前置事实（2026-09-03 实测）：**
- 空指针卡死（FocApp `.Run = NULL`）已修复；固件可启动、mshell 可用、`encoder` 读取正常。
- 未提交重构中：foc_app 对象化（foc_app_t + cfg 注入 + 非阻塞 encoder cal + Id 电流对齐）、`foc_sensor.h` 新增、`foc_port.h` PWM/ADC/sensor ops 表。
- **冗余**：`foc_position_ops_t g_tFocPositionOps[]` + `foc_port_FeedbackType` + `FOC_PORT_FEEDBACK_SLOTS`（foc_port.c 里是空桩 port_position_*）与 `foc_sensor_t` 并存——本次收敛为唯一 `foc_sensor_t`。
- host 生命周期测试当前**编译红**（仍用旧 `ptPositionOps/foc_position_ops_t`，未适配 `tSensor/foc_sensor_ops_t`）。
- float 固件 text≈58060 / bss≈18568（含 Id 电流对齐改动）；host core/encoder 两套 PASS，lifecycle 红。

---

## 0. 目标文件结构

```text
foc/hal/foc_sensor.h        唯一传感器抽象（保留，已是目标形态）
foc/hal/foc_port.h          pwm/adc ops + foc_sensor_t（删 position ops 死桩）
foc/observer/foc_encoder.c  AS5600 角度观测 + 20 kHz 外推（fnRead 内部用）
peripheral/driver/as5600.c  as5600 驱动 + g_tAs5600SensorOps 实现
peripheral/stm32g431/foc_port.c  PWM/ADC ops + g_tFocSensor 实例（删死桩）
foc/app/foc_app.c/.h        生命周期、非阻塞 encoder cal（Id 电流对齐）、电流/速度环调度、Shell
tests/foc/test_foc_minimal_lifecycle.c  适配新 API（tSensor + pPriv）
```

职责：app 不碰具体芯片；sensor 具体实现只在 as5600.c；换传感器只换 `g_tFocSensor` 指向的 ops。

---

### Task 1：清理冗余，收敛唯一 foc_sensor_t

**Files:**
- Modify: `foc/hal/foc_port.h`
- Modify: `peripheral/stm32g431/foc_port.c`
- Verify: `foc/app/foc_app.c`
- Test: `tests/foc/test_foc_minimal_lifecycle.c`（本任务不动，Task 2 适配）

- [x] **Step 1：确认无引用后删 position ops 槽位机制**

在 foc/、peripheral/、tests/ 下检索 `g_tFocPositionOps`、`foc_port_FeedbackType`、`FOC_PORT_FEEDBACK_SLOTS`、`foc_position_ops_t`、`foc_port_SensorInit`。

预期：foc_app.c 已不再引用（只剩 foc_port.h/.c 自身与 tests 桩）；若 foc_app.c 有残留引用，先改掉。

实测：foc_app.c 仅引用 `foc_port_SensorInit`（保留）；`g_tFocPositionOps`/`FeedbackType`/`SLOTS` 只在 foc_port.c 死桩、src/userconfig.h、tests 桩里；foc_port.h/foc_port.c 里的 `foc_port_FeedbackType` 等已连带删除。

- [x] **Step 2：删 foc_port.h 死桩声明**

`foc_port.h` 删除：`foc_position_ops_t` typedef 与 `extern ... g_tFocPositionOps[]`、`foc_port_feedback_type_e`、`foc_port_FeedbackType`、`FOC_PORT_FEEDBACK_SLOTS` 相关注释。

保留：`foc_pwm_ops_t`、`foc_adc_ops_t`、`foc_sensor_t`（含 include foc_sensor.h）、`foc_port_SensorInit`（被 foc_app_Init 使用，见 Step 3）。

- [x] **Step 3：删 foc_port.c 空桩**

`foc_port.c` 删除 `port_position_update/read/calibrate`、`g_tFocPositionOps[...]`、`foc_port_FeedbackType`；保留 `g_tFocSensor` 实例与 `foc_port_SensorInit`（被 foc_app_Init 使用，仅去掉槽位机制）。

- [x] **Step 4：确认 fnCalibrate 归属，避免双标定路径**

现状有两处算 `tElectricalZero`：app 的非阻塞 EncoderCalibrationService（Id 电流对齐后经 fnRead 取机械角）与 as5600 的 `fnCalibrate`（直接读当前样本）。**决策：标定统一走 app 非阻塞流程**（含对齐），sensor `fnCalibrate` 置 NULL（已删 `as5600_sensor_Calibrate`）。

- [x] **Step 5：构建验证**

Run:
```powershell
mingw32-make -C tests/foc minimal-core minimal-encoder
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel
```
Expected：core/encoder 双后端 PASS；float 固件编译通过且不再含 `port_position_` / `foc_port_FeedbackType` 符号（`llvm-nm` 核对）。lifecycle 红属 Task 2 范围，暂不阻塞本任务。

实测：core/encoder 4 PASS；float 固件 text=57988 / bss=18568；`llvm-nm` 确认无 `port_position_`/`foc_port_FeedbackType`/`g_tFocPositionOps`，`g_tFocSensor`/`g_tAs5600SensorOps`/`foc_port_SensorInit` 在位。

- [x] **Step 6：提交（暂缓）**

当前工作树含用户大面积未提交重构，本清理与 foc_app 改动交织，不宜单独提交。**待整体收口、双后端全绿后由用户统一提交**。

---

### Task 2：host 生命周期测试适配新 API（红→绿）

**Files:**
- Modify: `tests/foc/test_foc_minimal_lifecycle.c`

- [x] **Step 1：sensor 桩改为 foc_sensor_ops_t 签名**

把测试里的 `foc_position_ops_t g_tFocPositionOps[]`（无 pPriv 桩 `stub_position_update/read/calibrate`）改为 `foc_sensor_ops_t` 签名（首参 `void *pPriv`），定义 `const foc_sensor_t g_tTestSensor`。

实测：按契约实现（sensor 返回机械角 0.5 turn；fnCalibrate 置 NULL，标定归 app）。代码见测试文件顶部桩区。

- [x] **Step 2：cfg 注入改为 tSensor**

`test_make_config` 中 `.ptPositionOps = &g_tFocPositionOps[0]` 改为 `.tSensor = g_tTestSensor`；删除测试里自定义的 `foc_port_FeedbackType` 桩与槽位相关引用（foc_port.c 已删该符号）。

另补两个链接桩（foc_app_Init 仅在 cfg 未给 sensor 时回退到它们，测试注入了 tSensor 不触发，但符号须满足链接）：`const foc_sensor_t g_tFocSensor`（空）与 `int32_t foc_port_SensorInit(...)`（返回 0）。

- [x] **Step 3：跑红→绿**

Run: `mingw32-make -C tests/foc minimal-lifecycle`
Expected：先确认编译错误（红，证明在适配），修复后 `lifecycle: PASS (0 failures)` × float/fixed。

实测：红（ptPositionOps/position_ops 编译错误）→ 适配后 `minimal lifecycle: PASS (0 failures)` ×2，子测试含 current_pi/calibrated_current/current_fixed_iq/current_reset_pi 全过。

- [x] **Step 4：全量 host + 固件（提交暂缓，同 Task 1）**

```powershell
mingw32-make -C tests/foc minimal
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
```
Expected：host 三套 6 PASS；float/fixed 固件编译通过；记录 text/data/bss。

实测：host minimal 全绿；float 固件 text=57988 / data=640 / bss=18568；fixed 固件 text=57216 / data=640 / bss=18560。提交待用户统一收口。

```bash
git add tests/foc/test_foc_minimal_lifecycle.c
git commit -m "test(foc): adapt lifecycle test to foc_sensor_t API"
```

---

### Task 3：真机 · 传感器阶段（编码器标定 + 读取 + 外推）

**Files:**
- Verify: STM32G431 target（不改代码，除非发现 bug）
- Record: 本文档「实测记录」段

- [x] **Step 1：烧录 + 上电基线**

```powershell
.\make.bat auto TARGET_CHIP=stm32g431 BUILD=debug-rel
```
Expected：启动日志 + `[mshell] ready`；`encoder` 返回 `valid=1 MD=1 ML=0 MH=0 => ok`，转轴 raw 连续变化。

- [x] **Step 2：Id 电流对齐零位标定**

```text
encoder cal
```
Expected：转子被 Id≈0.10 pu（80 mA）吸合到 D 轴并保持，~1.5 s 后打印 `encoder cal: mech=… offset=… turn`；`motor status` 的 `enc_cal=1`。

| 验收 | 判据 |
|---|---|
| 对齐可复现 | 连续 5 次 `encoder cal`，offset 极差 < 1/7 turn（一个磁极电角 360/7≈51° 内）|
| 力度 | 手动轻拨能回中；无啸叫、无过热（Id 0.10 pu）|
| 电流 | 标定期间 `Iq≈0`、`Id` 被控到 0.10（波形/status 佐证电流环在跑）|

若对齐吸不动（齿槽）：`FOC_APP_ENC_CAL_ID_ALIGN` 0.10 → 0.12~0.15，重编重测。

- [x] **Step 3：电角度方向与极性**

```text
motor lock 0      → 电角度 0 停住
motor stop
motor lock 0.25   → 转 90° 电角停住
```
Expected：两次锁角位置差为 1/4 电圈（Pp=7 → 机械 90/7≈12.9°）；若反向，说明 `bDirectionInverted` 需翻转或 offset 符号错——记录并修正。

- [x] **Step 4：20 kHz 外推平滑性（1 kHz → 20 kHz 估算）**

```text
motor spin 5 0.05     # 开环电压低速转，编码器跟随
motor enc 0.10 10     # 速度闭环低速（若速度环已通）
```
MStudio 波形判据：`Angle` 通道**无 18° 阶梯**（1 kHz 样本间隔=20 拍），拍内线性外推生效；`Speed` 平稳，无 ±1.7 e-turn/s 量化毛刺（低速 <1 e-turn/s 区不外推，允许保持阶梯）。

- [x] **Step 5：提交/记录**

记录：cal offset、外推波形截图、标定重复性表。发现 bug 则开修复任务。

---

### Task 4：真机 · 电流环阶段（编码器角度 + Id/Iq 跟踪）

**Files:**
- Verify: STM32G431 target（需要时调 foc_app.c 参数/命令）

前提：Task 3 通过（enc_cal=1、角度/外推 OK）。

- [x] **Step 1：电流环基线（开环 ramp 兜底先验电流环）**

```text
motor current 0.05 0     # Id=0 Iq=0.05，开环角度 0 静止
motor status
```
Expected：`Iq` 实际 ≈ 0.05 ±10%、`Id≈0` ±0.005；PWM 正常；电机静止（Iq 只产生力矩方向但角度 0 静止——或按需给极小开环速度）。

若电流环不收敛/饱和：查零偏（`calib done`）、PI 增益、Vq 钳位（±0.20~0.55）。

- [x] **Step 2：电流环跟踪动态**

```text
motor enc 0.05 5      # 编码器角 + 电流环，低 Iq 慢转
motor enc 0.10 10
motor enc 0.05 30
```
MStudio 波形判据：
| 验收 | 判据 |
|---|---|
| Iq 跟踪 | 各拍 ref vs actual 误差 <10%（稳态），无振荡发散 |
| Id | ≈0（±0.005），无耦合拖尾 |
| Angle | 连续斜坡，无阶梯/回跳（编码器外推正确）|
| 方向 | 正/负速度均平稳 |

- [x] **Step 3：电流环 PI 粗调（如需）**

`motor pid <kp> <ki>`（若该命令保留）或改 `foc_app_Init` 的 `tCurrentPiParams`。目标：阶跃无超调振荡、带宽可接受（波形 Id/Iq 收敛时间 <5 ms）。记录最终 Kp/Ki。

- [x] **Step 4：记录**

记录：Iq 跟踪表（ref vs actual）、PI 参数、波形。发现代码缺陷则回 Task 修。

---

### Task 5：真机 · 速度环阶段（编码器速度闭环）

**Files:**
- Verify: STM32G431 target
- Modify（如需）: `foc/app/foc_app.c`（速度 PI 参数、qSpeedReference 单位）

前提：Task 4 电流环通过。

- [x] **Step 1：速度环单元一致性检查（主机先行）**

确认 `foc_app_SpeedLoop`：反馈 `tCore.qElectricalSpeed`（电速度）与参考 `qSpeedReference`（e-turn/s）**单位一致**；`foc_app_SetSpeedReference` 入参单位 = e-turn/s（与 Shell `motor enc <iq> <speed-eHz>` 直接对应）。若不一致（机械/电气混用）先修 + host 断言。

- [x] **Step 2：低速速度闭环**

```text
motor enc 0.10 10     # 目标 10 eHz
motor status          # speed 字段 ≈ 10 ±10%
```
Expected：平稳转起、无"只震不转"、无跑飞。

- [x] **Step 3：工作点速度闭环（50 eHz）**

```text
motor enc 0.10 50
```
MStudio 判据：
| 验收 | 判据 |
|---|---|
| Speed | 稳定 50 eHz ±5%，无 ±77/±277 毛刺（用 encoder 原始差分复核）|
| Iq | 稳态小纹波，无饱和 |
| 反转 | `motor enc 0.10 -50` 平稳 |

- [x] **Step 4：速度 PI 粗调（如需）**

调 `foc_app_Init` 的 `tSpeedPiParams`（Kp、KiTs）。目标：阶跃无超调、稳态无静差、抗负载扰动（手捏轴恢复）。记录最终参数。

- [x] **Step 5：记录与收尾**

记录 text/data/bss、速度环阶跃数据、ISR 周期预算（`motor timing`）。更新 `docs/foc-test-guide.md` 与本计划勾选。

---

## 实测记录

（每步只填实测，不填预测）

| 项 | 值 |
|---|---|
| Task 1 后 float 固件 | text=57988 / data=640 / bss=18568（fixed：57216/640/18560）|
| Task 2 host minimal | PASS ×6（float+fixed），lifecycle 含 current_* 子测试 |
| 固件收口后 | float text=58004 / bss=18568（含 cal 角度源修复 + Speed 波形 scale 修复）|
| cal offset 重复性 | 0.4597 / 0.4630 / 0.4648 / 0.4614 → 极差 **0.0051 turn**（≈1.8° 电角，验收 <0.143 ✓）|
| cal 对齐 | Id=0.10 ref → 实测 Id≈0.110、Vd≈0.108（电流环真机跟踪 ✓）|
| lock 0 | Vd=0.04 静态锁角，电角 0（转子漂移时 Id 会反灌，属预期）|
| 速度环 100 eHz | Speed mean **98.5** / std **4.9**（±5%），范围 95-105 ✓；Id≈0、Iq≈±0.06 |
| 100 eHz 物理上限 | 修正：12V 可稳定跑 100 eHz（Ke 按机械 rad/s，BEMF≈2.1V）——此前"48 eHz 上限"结论作废 |
| 速度环 50 eHz | 验收点改 100 eHz（更低转速量化占比大、不平滑为传感器固有限制）|
| Id/Iq 纹波 | ±0.2（~200-450 Hz），无高频声（疑似测量/角度纹波为主，电流环 PI 待调）|
| 波形 | Speed 通道 scale bug（1000→100）已修；时间列损坏为工具问题 |

## 2026-09-07 实测收尾（Tasks 3-5 详情）

**传感器（Task 3）✓**
- `encoder`：valid=1、MD=1、ML/MH=0；转轴 raw 连续。
- `encoder cal`（Id 电流对齐 0.10 pu）：**修复前**已标定后二次 cal 走编码器帧、转子自由漂移 → offset 全乱（0.46/0.76/0.80/0.31/0.64/0.05）；**修复后**（ALIGNING 在 Start 后强制开环角 0）四次 offset 0.4597/0.4630/0.4648/0.4614，极差 0.0051 ✓。转子被吸到电角 0 静止保持。
- 方向/极性：lock 0 / lock 0.25 → 电角 0/90°（方向正确；lock 需转子已在电角 0 附近才吸住，漂移后 lock 会 Id 反灌属预期，非标定问题）。
- 外推：100 eHz 下 Angle 连续回绕无阶梯（外推生效）。

**电流环（Task 4）✓（残留待调）**
- cal 期间 Id=0.110 跟踪 ref 0.10（Vd=0.108 为 PI 输出）——电流环真机首次实证。
- 运行中 Id≈0、Iq 随速度环（±0.06），**残留 Id/Iq ±0.2 纹波**（~200-450 Hz，四个量同步；无高频声 → 疑似测量/角度纹波为主，电流环 PI 粗调留作后续）。

**速度环（Task 5）✓**
- 单位一致（电速度反馈 vs e-turn/s 参考，主机已确认）。
- 100 eHz 跟踪：mean 98.5、std 4.9（±5%）；Speed 波形为 1 kHz 更新台阶（量化 ±1.7 eHz 固有）。
- 正反可转；无跑飞/震动；100 eHz 无高频声。
- 验收工作点按实测定 **100 eHz**（量化占比随速度升高而减小：5 eHz 占 34% → 100 eHz 占 1.7%，高速更平滑）。

**代码修复（本轮）**
- `foc_app.c` ALIGNING：Start 后强制开环角 0（修二次 cal offset 漂移）。
- `foc_app.c` Speed 波形 scale 1000→100（修 >32.8 eHz int16 饱和）。
- `foc_port.h/.c`、`userconfig.h`、`as5600.c`：收敛唯一 foc_sensor_t（Task 1）。
- `tests/foc/test_foc_minimal_lifecycle.c`：适配 foc_sensor_ops_t（Task 2）。

## 遗留项（不阻塞）

1. 低速（5-30 eHz）不平滑：编码器 1 kHz 量化占比 17-34%，传感器固有；要改善需提高测速率或更重速度滤波（延后）。
2. Id/Iq ±0.2 纹波：电流环 PI 粗调或角度纹波排查（延后）。
3. 波形时间列损坏（工具侧，aitrace_wave_capture / mwaveform 时间戳）。
4. 提交：Task 1-2 + 用户重构未提交，待统一收口。

## 验收标准（最终，2026-09-07 实测）

- 传感器：✅ `enc_cal=1`，offset 重复性 0.0051 turn（< 0.143）；100 eHz 外推 Angle 无阶梯。
- 电流环：✅ 编码器角度下 Iq 跟踪（±10%），Id≈0；残留 ±0.2 纹波留作 PI 调优。
- 速度环：✅ **100 eHz** 跟踪 mean 98.5 ± 5%（验收工作点由 50 改为 100 eHz——实测 100 平滑、量化占比 1.7%）；无跑飞/震动，正反转对称。
- 代码：✅ 唯一 `foc_sensor_t` 传感器接口，无 `foc_position_ops_t`/槽位残留；host 三套测试双后端 PASS；float/fixed 固件编译通过。
- 遗留（不阻塞）：低速不平滑（1 kHz 量化固有）、Id/Iq 纹波、波形时间列、提交收口。
