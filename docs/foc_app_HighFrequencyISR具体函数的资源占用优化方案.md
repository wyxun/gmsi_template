# FOC 高频控制环路资源占用优化方案（修订版）

> 本文在初版《foc_app_HighFrequencyISR具体函数的资源占用优化方案》基础上
> 按以下思路重写：以 **STM32G431 的 CORDIC 硬件三角加速器**为主力后端，
> 以**查找表 + 线性插值**为通用软件后端，配合 Park/IPark sin/cos 单次
> 计算、免 `floorf` 角度取整和结构性瘦身，把 20 kHz 高频步的 CPU 占用
> 从约 87% 压到 35% 以内，并为后续 SMO/NLFO 观测器闭环预留算力。

---

## 1. 背景与预算核算

平台：STM32G431（Cortex-M4F，170 MHz，单精度 FPU，含 **CORDIC** 与
**FMAC** 数学加速器）。高频控制步 `motor_HighFrequencyStep()` 由 ADC
注入转换完成中断以 20 kHz 调用：

```text
每周期预算 = 170 MHz / 20 kHz = 8500 cycles
实测现状（-Os，电压开环） ≈ 7442 cycles ≈ 87% CPU
```

现状 87% 的占用连低频级联、MODUS 主循环、Shell 都已紧张；更严重的是
当前仅是**纯开环**。代码事实：SMO（`foc/observer/foc_smo.c:107`）和
NLFO（`foc_nlfo.c:129`）都经 `foc_angle_atan2()` 调 `atan2f`，HFI
（`foc_hfi.c:58`）用 `foc_angle_cos()`。一旦接观测器闭环，libm 瓶颈
会在同一接缝上再爆一次。因此本方案不把目标设为"省掉 4 次 sinf"，而是
**系统性移除 FOC 内核对 libm 三角函数的依赖**。

### 1.1 优化目标

| 指标 | 现状 | 目标 |
|---|---|---|
| 单高频步 cycles（-Os，开环） | ~7442 | ≤ 3000（≤ 35%） |
| 单高频步 cycles（-Os，SMO 闭环） | 未测（预计 > 9000，溢出） | ≤ 4500（≤ 53%） |
| 三角函数精度 | libm 参考 | 最大绝对误差 < 2e-6（满足 host 测试 1e-5 容差并留 5 倍裕量） |
| `-O0` 行为 | 饿死系统 | 明确不支持，文档化（见 §8） |

### 1.2 硬约束

1. **不破坏 host 测试容差**：`tests/foc/test_numeric.c` 的
   `fScalarTolerance` 为 `1e-5`（float）/ `4e-5`（fixed）。任何三角
   后端的误差必须显著小于 1e-5，否则测试要放宽容差——不允许；
2. **公共 API 零变化**：`foc_angle_sin/cos/atan2` 签名不变，后端经
   编译期选择；
3. **MDI 边界**：CORDIC 驱动代码只出现在 `peripheral/stm32g431/`，
   FOC 内核不 include 任何 vendor 头；
4. **多后端共存**：host（x64 测试）、AT32、G431 三种构建各走各的
   后端，host 测试必须能验证默认软件后端（LUT）的数值正确性；
5. host 矩阵（encapsulation / float / fixed / strict-alias）全程 PASS。

---

## 2. 现状热点分析（代码事实）

### 2.1 重复三角计算

`foc/middleware/foc_core.c`：

- `foc_park()`（L36-37）：`foc_angle_sin(tTheta)` + `foc_angle_cos(tTheta)`；
- `foc_ipark()`（L55-56）：同角度再来一遍。

单次高频步对**同一个电角度**算了 2×(sin+cos)。`foc_angle_cos()` 又是
`foc_angle_sin(x + 0.25)`（`foc_angle.c:93-97`），float 后端最终落到
`sinf()`（`foc_angle.c:87`）——每步共 **4 次 `sinf()`**，实测单次
~284 cycles（内含范围归约的 `floorf`），合计 1100+ cycles。

### 2.2 `floorf` 取整

`foc_angle.c:17`：`qTurns -= floorf(qTurns)`。`sinf` 内部还有自己的
范围归约。wrap 本身 ~60 cycles，但它出现在每一次 sin/cos/atan2 调用中。

### 2.3 三角依赖面（全部集中在 `foc/math/foc_angle.c`）

| 函数 | float 后端 | fixed 后端 | 调用方 |
|---|---|---|---|
| `foc_angle_sin/cos` | `sinf()`（含 `floorf`） | 已有抛物线近似 | Park、IPark、HFI |
| `foc_angle_atan2` | `atan2f()`（~300+ cycles） | 已有 CORDIC 式迭代 | SMO、NLFO |

fixed 后端**已经不用 libm**（自研抛物线/迭代），float 后端是唯一
libm 依赖者——这证明自研三角后端在本库是既定路线，本方案只是把
float 后端补齐，并用 G4 硬件再压一档。

### 2.4 结构性开销（非三角）

1. **快照提交**：`motor_control_commit_hf()` 每高频步在临界区内全量
   更新快照相关字段（`motor_snapshot_t` 约 24 个字段），诊断用途却按
   控制频率付费；
2. **profiling 残留**：`motor_control.c:14-17` 的
   `g_wParkCycles/g_wIparkCycles/g_wModulateCycles/g_wCommitCycles`
   及 5 处 `get_system_ticks()` 计时是调试期手动加入的，需宏门控化；
3. **除法**：调制与观测器路径中的 `/` 在 float 下是 VDIV（~14 cycles，
   可接受），在 fixed 下是长除法；频率低的可不动，每步执行的除法改
   为预取倒数；
4. **double 提升红线**：任何无 `f` 后缀的浮点常量（如 `0.775`）都会
   把整条表达式提升为 soft-double，M4F 上一条即可吃掉数百 cycles。
   必须作为 review 检查项（§7.3）。

---

## 3. 总体架构：`foc_angle` 三角后端三层

```
┌────────────────────────────────────────────────────┐
│  foc_angle_sin / foc_angle_cos / foc_angle_atan2   │  ← 公共 API 不变
├────────────────────────────────────────────────────┤
│  编译期后端选择（FOC_TRIG_BACKEND，见 §3.1）         │
├──────────────┬──────────────────┬──────────────────┤
│ libm（参考） │ LUT+lerp（通用） │ CORDIC（STM32G4） │
│ host 对照用   │ 默认，host/AT32  │ peripheral 绑定    │
└──────────────┴──────────────────┴──────────────────┘
```

### 3.1 编译期选择

新增 `foc/foc_config.h` 或编译命令行宏：

```c
#define FOC_TRIG_BACKEND_LIBM    0   /* 参考实现，仅用于精度对照 */
#define FOC_TRIG_BACKEND_LUT     1   /* 通用软件后端（默认） */
#define FOC_TRIG_BACKEND_CORDIC  2   /* STM32G4 硬件后端 */

#ifndef FOC_TRIG_BACKEND
#define FOC_TRIG_BACKEND FOC_TRIG_BACKEND_LUT
#endif
```

- `tests/foc/Makefile` 显式加 `-DFOC_TRIG_BACKEND=LUT` 跑完整矩阵，
  另加一个 `TRIG=libm` 目标做逐点对照（§7.2）；
- `target/stm32g431/target.mk` 设 `FOC_TRIG_BACKEND=CORDIC` 并追加
  CORDIC 源文件；
- AT32 目标保持默认 LUT，不引入任何新代码。

---

## 4. 分层优化方案

### L0：Park/IPark sin/cos 单次计算（编排层）

**依据**：§2.1，省 2 次三角调用，与后端无关，先做。

**改动**：

1. `foc/middleware/foc_core.h` 新增：

```c
foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ);
foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB);
```

2. `foc/middleware/foc_core.c`：原 `foc_park/foc_ipark` 内部改为调用
   cached 版本（保持兼容，测试不动）；cached 版本纯乘加，无校验差异，
   NULL 检查与原函数一致。
3. `foc/motor/motor_control.c` 高频步：

```c
foc_scalar_t qSinTheta = foc_angle_sin(angle);
foc_scalar_t qCosTheta = foc_angle_cos(angle);
/* ... */
eResult = foc_park_cached(&current_alpha_beta,
                          qSinTheta, qCosTheta, &work.tCurrent);
/* ... */
eResult = foc_ipark_cached(&work.tVoltage,
                           qSinTheta, qCosTheta, &work.tVoltageAlphaBeta);
```

**预期收益**：LUT 后端下省 ~40 cycles；libm 后端下省 ~570 cycles。

### L1：免 `floorf` 角度取整

**依据**：§2.2。M4F 的 float→int32 是单条 VCVT（1–2 cycles），截断
方向为向零，据此构造 floor 语义：

```c
static foc_scalar_t angle_wrap_scalar(foc_scalar_t qTurns)
{
#if defined(FOC_NUMERIC_FLOAT)
    /* (int32_t) 向零截断：负数结果落在 (-1, 0]，补偿一圈 */
    qTurns -= (foc_scalar_t)(int32_t)qTurns;
    return qTurns < FOC_ZERO ? qTurns + FOC_ONE : qTurns;
#else
    qTurns %= FOC_ONE;
    return qTurns < FOC_ZERO ? qTurns + FOC_ONE : qTurns;
#endif
}
```

正确性论证：

- x ∈ [0, +∞)：截断即 floor，结果 ∈ [0, 1) ✓；
- x ∈ (-1, 0)：`(int32_t)x == 0`，x 不变，落入负分支 +1 → (0, 1) ✓；
- x ≤ -1：截去整数部分后 ∈ (-1, 0]，+1 → (0, 1]；x 为负整数时得
  1.0，与原 `floorf` 版（得 0.0）差一整圈——对 sin/cos 等价，但为
  严格一致，LUT 索引前再做一次 `>= 1.0 ? 0.0` 钳位（见 §4.2 注释）；
- 定义域 |x| < 2³¹：电机角度积分每步增量 ≪ 1 圈，开环积分器每步
  wrap，实际值域永远在 ±2 圈内，安全。

**预期收益**：每次调用 ~60 → ~5 cycles；配合 L2 后每高频步的 wrap
次数也下降（LUT 后端内部只做一次）。

### L2：LUT + 线性插值软件后端（默认）

**选型对比**（float 后端替代 `sinf` 的候选）：

| 方案 | 误差 | 估算 cycles | 结论 |
|---|---|---|---|
| 抛物线逼近（初版方案）y=4x(1-x) 修正 | ~1e-3 | ~25 | 精度不满足 1e-5 容差，淘汰 |
| 512 项 1/4 波 LUT + lerp | ≤ 1.2e-6 | ~20–30 | **采用** |
| 1024 项 LUT + lerp | ≤ 3e-7 | ~20–30 | 精度裕量更大但表占 4KB，512 项已够 |

**误差预算**：1/4 波 512 项（含端点 513 项，2 KB flash），步长
Δ = (π/2)/512；线性插值最大误差 ≈ Δ²/8 ≈ 1.2e-6 < 1e-5 ✓，且比
初版抛物线方案高 3 个数量级。

**实现要点**（`foc/math/foc_trig_lut.c` 新增）：

1. **表离线生成**：用 Python 脚本生成 `static const float
   s_afSinQuarter[513]` 源文件（`tools/` 下附生成脚本），禁止运行期
   调 `sinf` 初始化表（否则等于没优化，且 -O0 下启动慢）；
2. **象限还原**：输入 wrap 到 [0,1) 后，x·4 的整数部分给象限
   （0–3），小数部分×512 给索引与插值系数；1、3 象限取负，2、3
   象限反向查表；象限对称性同时保证 cos = sin(x+0.25) 无需第二
   张表（现有 `foc_angle_cos()` 结构不变）；
3. **插值**：`y = y0 + (y1 - y0)·frac`，全部 float 乘加，M4F 上
   VMLA 约 3–5 cycles；
4. **端点**：frac 索引 512 时取表端点，避免越界（513 项设计）；

伪代码：

```c
foc_scalar_t lut_sin_turns(foc_scalar_t qTurns)   /* qTurns ∈ [0,1) */
{
    float fPhase = qTurns * 4.0f;                 /* 圈→象限 */
    uint32_t wQuad = (uint32_t)fPhase;            /* 0..3 */
    float fFrac  = (fPhase - (float)wQuad) * 512.0f;
    uint32_t wIdx  = (uint32_t)fFrac;
    float fLerp    = fFrac - (float)wIdx;
    /* 2、3 象限镜像，1、3 象限取负，查 s_afSinQuarter[wIdx/wIdx+1] */
    ...
}
```

5. **atan2 暂留 libm**：LUT 后端下 `foc_angle_atan2` 仍用
   `atan2f`（精度参考），原因：(a) atan2 只在观测器路径出现，开环
   不付费；(b) CORDIC 后端会接管它；(c) 自研 atan2 做到 1e-6 精度
   成本高于收益。若 AT32 后续要跑 SMO，再补 LUT 版 atan2（另立任务）。

### L3：CORDIC 硬件后端（STM32G431 主力）

**依据**：STM32G4 的 CORDIC 协处理器原生支持 cosine / sine /
phase（atan2）/ modulus（幅值+sqrt）等函数，q1.31 定点 I/O，一次
cosine 运算**同时输出 X=cos、Y=sin** 两个结果，天然契合 §4 L0 的
sin/cos 成对需求；精度随迭代次数可配，满迭代约 2⁻²⁰（~1e-6）。

**架构**（遵守 MDI 边界，FOC 内核零 vendor 依赖）：

```
foc_angle_sin/cos/atan2 ── FOC_TRIG_BACKEND=CORDIC ──▶ foc_trig_if_t
                                                          │ 编译期静态绑定
                              peripheral/stm32g431/halcordic.c
                              （LL/寄存器级，不引 HAL CORDIC 句柄开销）
```

1. **接口**：`foc/math/foc_trig.h` 定义内部后端 vtable（不进入
   公共 `foc.h`）：

```c
typedef struct {
    void (*fnSinCosTurns)(foc_scalar_t qTurns,
                          foc_scalar_t *pqSin,
                          foc_scalar_t *pqCos);
    foc_scalar_t (*fnAtan2Turns)(foc_scalar_t qY, foc_scalar_t qX);
} foc_trig_backend_t;
```

   L0 的 `motor_control.c` 在 CORDIC 后端下直接调 `fnSinCosTurns`
   一次拿两个值，连 `foc_angle_cos` 的 +0.25 圈加法都省掉；LUT 后端
   的 `fnSinCosTurns` 用同一索引路径算 sin 后再算 cos（共享象限
   分解），也比两次独立调用省一次 wrap。

2. **CORDIC 驱动**（`peripheral/stm32g431/halcordic.c/.h` 新增）：
   - 初始化：开 RCC 时钟，CSR 配置 FUNCTION=cosine/phase、
     PRECISION（迭代次数，先按满精度配，上机实测后可降档换速度）、
     SCALE=0、32-bit 参数；
   - 运算：写 ARGUMENT 寄存器 → 轮询 RRDY → 读 X/Y RESULT；
     典型全程 ~20–40 cycles（**估算值，以上机 DWT 实测为准**）；
   - 格式转换（BAM32 映射，2026-07-21 评审后定稿）：角度 turns [0,1)
     直接作二进制角度，经无符号满量程映射到 q1.31：

     ```c
     /* qTurns ∈ [0,1)，由 angle_wrap_scalar 保证；1.0 已在 wrap 归一为 0 */
     uint32_t u32Turns = (uint32_t)(qTurns * 4294967296.0f); /* [0, 2^32) */
     int32_t  q31      = (int32_t)u32Turns;  /* 补码重解释：[0.5,1) → [-π,0) */
     ```

     - 连续性：0.5 圈 → 0x80000000 = -π ≡ +π；qTurns→1⁻ →
       0xFFFFFFFF → -1 LSB ≈ 0⁻，回绕点无跳变；
     - 无 UB 论证：float 在 [0,1) 内最大取值为 1−2⁻²⁴，乘积最大恰为
       4294967040.0f（= 2³²−256，可精确表示），**永远不会达到 2³²**，
       float→uint32 全程在值域内；`(int32_t)u32` 高位重解释在 C 标准
       中属 implementation-defined（非 UB），GCC/Clang/ARM 均为补码
       预期行为；
     - 前置契约：`angle_wrap_scalar` 必须把负整数圈输入产生的 1.0
       归一为 0.0（L1 已列的 `>= 1.0 ? 0.0` 钳位），保证定义域严格
       为 [0,1)；
     - 精度：乘积 float 相对精度 2⁻²⁴ → 角度量化 ≤ 2⁻²³ 圈
       （~7.5e-7 rad），叠加 CORDIC 本身 ~1e-6，总误差仍 < 2e-6，
       满足 §1.1 目标；
     - 结果 q1.31 → float 回程同理（VCVT 单指令 + 2⁻³¹ 比例乘法）。
     **所有比例常量必须带 `f` 后缀**（§2.4 红线）；
   - 并发规则：CORDIC 为不可重入单实例资源。本库规定**只允许
     高频 ISR 上下文使用**（motor HF 步本身已不可重入，天然满足）；
     主循环/低频若需三角，走 LUT。此规则写入 `halcordic.h` 头注释；
   - `target/stm32g431/target.mk`：`FOC_TRIG_BACKEND=CORDIC`，
     追加 `peripheral/stm32g431/halcordic.c`（LL 方式直接操作寄存器，
     不把 `stm32g4xx_hal_cordic.c` 加入 HAL_SOURCES，避免句柄层开销；
     vendor 头文件已存在于固件库，仅作寄存器定义参考）；
   - **atan2 接管**：CORDIC phase 模式输入 X、Y 输出角度，替换
     `atan2f`，SMO/NLFO 闭环的 ~300+ cycles/次直接降到 ~40。

3. **回退**：CORDIC 仅在 `TARGET_CHIP=stm32g431` 且显式开关时启用；
   任何异常把宏改回 LUT 即可，FOC 内核代码零改动。

### L4：结构性瘦身

1. **快照提交降频**：`motor_control_commit_hf()` 拆为"控制必需提交"
   （duty、runtime 状态，每步）与"诊断镜像"（快照字段，每 N 步或
   低频事务合并）。N 取 4–10，经 `motor_config_t` 可选配置，默认
   保持现状（每步）以免改变 Task 7 已冻结的快照语义；本期先实现
   降频机制并默认关闭，上机实测后决定默认值。**注意**：快照字段
   的语义已在 Task 7 冻结，改动需同步更新 `motor_types.h` 注释与
   交接文档；
2. **profiling 宏门控**：`g_wParkCycles` 等 5 处 `get_system_ticks()`
   计时包进 `#if defined(MOTOR_PROFILE_CYCLES)`，默认编译为空；
   保留最外层单步总时长一个计数即可（`g_wHfStepCycles`）；
3. **每步除法改倒数**：调制/归一化路径中每个高频步都执行的
   `x / const` 改为启动时预取 `rcp = 1/const`（float 下 VDIV 仅
   ~14 cycles，此项优先级最低，顺带做）；
4. **编译选项**：
   - `foc/math`、`foc/middleware`、`foc/modulation` 三个目录在
     Makefile 中单独提到 `-O2`（即使工程整体 `-Os`/`-O0`），用
     per-directory CFLAGS；
   - 全工程禁用隐式 double 提升的 review 检查（§7.3），不加
     `-ffast-math`（破坏 NaN/Inf 语义，风险大于收益）。

### 不纳入本期的项（写明边界）

- **FMAC 卸载 PID/滤波**：G4 的 FMAC 适合 IIR/FIR 与 PID，但接入
  需要重写控制器数据流，收益/复杂度比低，待 CORDIC 落地后再评估；
- **FOC 降频到 10 kHz**：控制带宽妥协，仅作为所有优化用尽后的
  兜底产品决策；
- **LUT 版 atan2**：仅当 AT32 平台需要跑观测器时另立任务。

---

## 5. 文件级改动清单

**新增**

- `foc/math/foc_trig.h`：后端 vtable 与 `FOC_TRIG_BACKEND_*` 宏；
- `foc/math/foc_trig_lut.c`：LUT 后端（含 513 项表）；
- `peripheral/stm32g431/halcordic.h/.c`：CORDIC 驱动与后端绑定；
- `tools/gen_sin_lut.py`：表生成脚本（参数化项数，输出 C 文件）；
- `tests/foc/test_trig.c`：后端精度单测（见 §7.2）。

**修改**

- `foc/math/foc_angle.h/.c`：后端分派；`angle_wrap_scalar` 无
  `floorf` 实现；`foc_angle_cos` 保持 +0.25 圈复用；
- `foc/middleware/foc_core.h/.c`：`foc_park_cached/foc_ipark_cached`；
- `foc/motor/motor_control.c`：单次 sin/cos 编排；profiling 宏门控；
- `foc/motor/motor_private.h`：快照降频计数字段（利用现有 16 字节
  余量，**不得扩大 512 ABI**；加字段后 float/fixed 双构建重新确认
  `sizeof(motor_impl_t)`）；
- `foc/foc_config.h`：`FOC_TRIG_BACKEND` 默认值；
- `tests/foc/Makefile`：`TRIG=libm` 对照目标；
- `target/stm32g431/target.mk`：CORDIC 后端开关与源文件；
- `Makefile`：三个 foc 目录 per-directory `-O2`。

**不改**

- `motor.h` / `motor_types.h` 公共 API 与快照字段语义（默认配置下）；
- fixed 后端（本就不用 libm）；
- `foc/app/`（应用层零改动）。

---

## 6. 预期收益核算

| 项目 | 现状估算 | 优化后估算 | 备注 |
|---|---|---|---|
| 4× sinf + wrap | ~1150 | ~40（LUT sin/cos 对） | CORDIC 更低 |
| Park/IPark 乘加 | ~60 | ~60 | 不变 |
| 快照全量提交 | ~150–300 | ~40（N=8 降频摊销） | 机制先行，默认不变 |
| profiling 计时 | ~50 | ~5（宏门控） | |
| 其余路径（采样/Clarke/PID/SVPWM/commit/事件环） | ~5900 中的剩余部分 | 待分解实测 | 见 §7.4，若某段 > 500 再立项 |
| **合计目标** | **~7442** | **≤ 3000** | |

> 说明：现状 7442 中三角直接开销仅 ~1200，**其余 ~6000 cycles 的
> 构成本方案不臆断**，实施第一步先用现有分段计数（L4.2 宏门控后
> 保留的分段点）在 G431 上出一份分解表，再决定是否需要追加针对
> 性优化（如 SVPWM、commit 路径）。

---

## 7. 验证计划

### 7.1 host 功能矩阵（每步改动后必跑）

```powershell
$env:Path = 'D:\software\msys64\mingw64\bin;D:\software\msys64\usr\bin;' + $env:Path
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' -C tests/foc SHELL=cmd.exe CC=gcc clean all
& 'D:\software\msys64\mingw64\bin\mingw32-make.exe' -C tests/foc SHELL=cmd.exe CC=gcc `
    STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2' clean all
```

期望：encapsulation / float / fixed 全 PASS，0 failures。

### 7.2 三角后端精度验证（`tests/foc/test_trig.c`）

- **逐点扫描**：x ∈ [0, 1) 步进 2⁻¹⁶ 圈，LUT 后端与 libm 参考
  （`TRIG=libm` 目标编译同一测试）逐点比较，最大绝对误差断言
  < 2e-6；
- **象限对称性**：sin(x) == -sin(x+0.5)、sin(x) == cos(x-0.25) 在
  全范围内一致（容差 1e-6）；
- **单调性**：第一象限内严格单调；
- **特殊点**：0、0.25、0.5、0.75 圈精确命中 0/1/0/-1；
- **wrap 等价性**：`angle_wrap_scalar` 对 ±2 圈、负整数圈、
  ±(1−ε) 边界输入与 `floorf` 参考实现逐点一致（允许整圈差）；
- **atan2**：CORDIC 后端上机后，与 LUT/libm 对照打印偏差（目标
  < 1e-5 圈）。

### 7.3 静态检查

- `rg -n "[0-9]\.[0-9]+[^f0-9]" foc/math foc/middleware foc/motor`
  review 新增浮点常量全部带 `f` 后缀；
- `rg -n "double" foc` 结果为空（既有 `foc_angle_from_turns(float)`
  接口除外，保持 float）；
- `git diff --check` 干净。

### 7.4 上机实测（G431）

1. `mingw32-make SW_ROOT=D:/software clean TARGET_CHIP=stm32g431` 后
   `BUILD=debug-rel TARGET_CHIP=stm32g431 FOC_NUMERIC=float` 构建
   （含 `MOTOR_PROFILE_CYCLES=1`）；
2. 烧录 + RTT（按 `.agents/skills/aitrace/SKILL.md` 流程；**烧录、
   halt、reset 前必须取得用户确认**）；
3. 电机开环运行，经 RTT/快照读取分段周期数，填实测表：

| 段 | 优化前 | LUT | CORDIC |
|---|---|---|---|
| Park | | | |
| IPark | | | |
| Modulate | | | |
| Commit | | | |
| 整步 | 7442 | | |

4. `-Os` 与 `-O2`（foc 三目录）各测一轮；
5. 验证电压开环拖动波形与优化前一致（RTT 波形通道 duty/电流对比）；
6. SMO 闭环（如已具备条件）验证 CORDIC atan2 路径。

---

## 8. 风险与决策记录

1. **`-O0` 不支持 20 kHz FOC**：M4F 无优化代码物理上跑不进
   8500 cycles，这是工具链属性而非缺陷。决策：FOC 目标固件最低
   `BUILD=debug-rel`，文档化，不做 `-O0` 适配；
2. **LUT 精度不达标**（概率低）：回退 `FOC_TRIG_BACKEND=LIBM`，
   保留 L0/L1 收益，host 测试天然守护；
3. **CORDIC 驱动问题**：后端经编译期隔离，回退 LUT 零成本；
4. **快照降频语义变化**：Task 7 已冻结快照语义，默认配置保持
   每步提交；启用降频前更新 `motor_types.h` 注释、README §6 快照
   表与交接文档；
5. **host 与目标行为分叉**：LUT 是 host 与 AT32/G431 的共同默认
   后端，host 测试直接守护固件实际运行的数值路径；CORDIC 偏差
   由 §7.2 上机对照项守护；
6. **512 字节 ABI**：新增降频计数字段后 float/fixed 双构建确认
   `sizeof(motor_impl_t)` ≤ 512（当前余量 12–16 字节，足够 1–2 个
   `uint16_t`）。

---

## 9. 实施顺序（每步独立可回退、独立验证）

1. **Step 1**：L0（cached 变换 + 单次 sin/cos）+ L4.2（profiling
   宏门控）→ host 矩阵 → G431 构建；
2. **Step 2**：L1（wrap）→ host 矩阵（含 §7.2 wrap 等价性）；
3. **Step 3**：L2（LUT 后端 + 生成脚本 + `test_trig.c`）→ host
   矩阵 + `TRIG=libm` 逐点对照；
4. **Step 4**：L3（CORDIC 驱动 + target.mk 接线）→ G431 构建 →
   上机实测对照表（§7.4，需用户确认烧录）；
5. **Step 5**：L4.1（快照降频机制，默认关闭）+ L4.3/L4.4 → 全
   矩阵 + 上机终测 → 更新交接文档与 README §5/§6 相关表述。
