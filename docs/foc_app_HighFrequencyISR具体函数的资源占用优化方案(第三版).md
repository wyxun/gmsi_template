# FOC 高频控制环路资源占用优化方案（第三版 / V3）

## 1. 目的与适用范围

本文是《foc_app_HighFrequencyISR具体函数的资源占用优化方案（第二版）》之后的 V3 实测驱动修订版。前两版完成 BAM32 角度模型、Profile 分级闭包、调用闭包 -O2 之后，本版以**当前实测数据**为基线，目标从"降低既有链路占用"升级为：

> **为无感算法（观测器 + PLL + 开环启动切换）腾出预算，保证含无感的完整高频步在 20kHz 下长期稳定运行。**

约束沿用前两版：不降低 20kHz 控制频率、不削弱硬件保护、`foc/` 内核不引入厂商头文件、float/fixed 后端语义一致。

---

## 2. V3 基线（实测，非推算）

构建配置（自 V3 起）：全局 `-Os`（`BUILD=debug-rel` 为默认），`foc_*.c` 与 `motor_control.c` 追加 `-O2`（makefile:266-268）。

STM32G431 @170MHz，20kHz，周期预算 **8500 cycles**，电流闭环 2Hz 运行态：

| 阶段 | 实测 cycles | 占比 | 说明 |
| --- | ---: | ---: | --- |
| sample（电流采样/重建） | 414 | 4.9% | 含 4~5 层调用 + 3×VDIV |
| clarke | 134 | 1.6% | |
| park | 184 | 2.2% | |
| ipark | 166 | 2.0% | |
| mod（SVPWM） | 452 | 5.3% | |
| commit（PWM 提交） | 706 | 8.3% | **单项最大** |
| **已计量小计** | **2056** | **24.2%** | |
| **未计量缺口** | **~1782** | **21.0%** | 见 §3.1 |
| **total（FOC_HF_PROFILE Level 2）** | **3838** | **45.2%** | |

无感预算评估：SMO/磁链观测器 + PLL 在 M4F float `-O2` 下典型 1500~2500 cycles。直接叠加将到 5500~6300（65~75%），可运行但余量偏紧。因此 V3 目标：

| 指标 | V3 基线 | V3 目标 | 含无感后目标 |
| --- | ---: | ---: | ---: |
| 高频步 total（mean） | 3838 (45%) | ≤ 3000 (≤ 35%) | ≤ 5500 (≤ 65%) |
| P99 | 未测 | ≤ 4250 (50%) | ≤ 6800 (80%) |

---

## 3. V3 核心发现与优化项定位

### 3.1 未计量缺口 ~1782 cycles 的候选构成

`motor_HighFrequencyStep()`（foc/motor/motor_control.c:297）中，六个已计量阶段之外的开销：

1. **`work = *ptControl` 全量结构体拷贝**（motor_control.c:332）——`motor_control_t` 体量上百字节，`-Os` 下展开为 memcpy/逐字搬运，每周期一次，嫌疑最大；
2. D/Q 两路 PI 环（未单独计量）；
3. 位置源 `foc_position_source_Step()` 接口转发（函数指针）；
4. 入口状态快照临界区 + 结尾 publish 临界区（两次 `motor_private_enter/exit`）；
5. Profile 自身：Level 2 共 14 处 DWT CYCCNT 读取与局部变量往返（估 100~200 cycles）。

**处理规则：先照明、后动刀。** 未分项实测前禁止凭猜测重写任何一段。

### 3.2 已证实的教训：优化等级迁移的硬件时序陷阱

全局 `-O0 → -Os` 迁移中，`haladc.c` 的 `regulator_stabilize()` / `post_calib_delay()` 非 volatile 空循环被编译器删除，导致 ADC2 稳压器未稳定即校准、`ADRDY` 超时后静默 `LL_ADC_Disable()`，三相电流只剩 U 相有效（V/W 恒显 1.0000 pu，即 `(offset-0)/offset`）。

两条硬性规则由此产生：

- **凡靠空循环等待硬件时序的代码，循环变量必须 `volatile` 或使用 cycle-accurate 延时**（perf_counter delay）。已修复 haladc.c:25-37，全 peripheral 目录已扫描无其他同类；
- **硬件初始化兜底分支（如 ADRDY 超时弃疗）必须显式 log/置故障**，禁止静默降级。

### 3.3 调用链冗余审计（实测逐层核实）

**commit 链（706 cycles，注意：该分项是整个 `motor_control_commit_hf()`，含临界区 + ~15 字段写回 + SetDuty）：**

```text
motor_control_commit_hf()                        motor_control.c:183   临界区/状态检查/写回
└─ foc_hal_SetDuty()                             foc_hal.c             包装：2 次空检查
   └─ fnSetDuty 函数指针 → mdi_pwm_set_duty()    foc_hal_mdi_adapter.c 适配器：3 次空检查
      └─ 3× MDI_Write → mdi_pwm_SetDuty()        mdi.h (inline)        每次 2 次空检查
         └─ fnSetDuty 函数指针 → pwm_setduty()   port_mdi.c:105        pPriv 还原通道号 + if/else 链
            └─ LL_TIM_OC_SetCompareCHx()         本质：TIM1->CCRx = wDuty
```

**诊断：分层没错，设备粒度过细。** 底部只是三条 `TIM1->CCRx = wDuty`，但每相要付 1 次间接调用 + 通道号重解码 + 重复空检查（三个 mdi_pwm_t 的 `fnSetDuty` 指向同一个 `pwm_setduty`，空检查做了三遍）。根因是把同一 TIM1 的三个 CCR——硬件上同一预装载原子组——建模成三个独立 MDI 设备。`foc_hal_SetDuty(u,v,w)` 这个三相接口粒度本来就是对的，冗余全部在适配器之下。

**sample 链（414 cycles）同构：** 底部是三条读 JDR，上面叠 4 层（`foc_hal_CurrentReconstruct` 包装 → `fnReconstruct` 指针 → `mdi_adc_get_raw` → 跨 TU 的 `haladc_GetInjected`×3）。函数指针层是 HAL 多芯片多态所必需，保留；跨 TU 小函数无法内联是纯浪费，消除。

**收益重估（修正 V3 初稿）：** SetDuty 转发链本身估 150~250 cycles，不是 706 的全部；706 的其余部分是临界区与写回。P0 必须先把 commit 拆成 SetDuty 子段与 publish 子段再定刀法。

### 3.4 优化项定位表

| 优化项 | 定位 | 预期收益 | 依据 |
| --- | --- | ---: | --- |
| Profile 分项补全（PI/publish/位置源/拷贝/SetDuty 子段） | **P0 前置照明** | 0（诊断项） | §3.1 / §3.3 |
| `work = *ptControl` 改按需取字段 | P1 | ~200~400 | 全量拷贝在热路径无必要 |
| commit：适配器内三相合并为单次 port 层直写 | P1 | ~150~250 | §3.3，消 2 次间接调用 + 通道重解码 |
| `haladc_GetInjected` 头文件 `static inline` 化 | P2 | ~100~150 | 跨 TU 无法内联，函数体仅读 JDR |
| `-flto` 全链路（foc + motor + peripheral） | P2 | 整体 10~15% | 跨模块小函数内联，免手工搬运包装层 |
| 发布构建关 `FOC_HF_PROFILE` | P3 | ~100~200 | Level 2 计量成本 |
| CORDIC sin/cos | **不做** | — | park+ipark 仅 350 cycles，收益有限且引入延迟错配风险 |
| 砍 `foc_hal_*` 包装层 / 函数指针改直绑 | **不做** | — | 多芯片多态所需，-flto 后成本趋零 |

---

## 4. 分阶段实施路线

### P0：Profile 分项补全（照明缺口）

**目标：** 把 ~1782 cycles 的缺口、以及 commit 分项内部构成，全部分解到具名阶段，为 P1 提供实测依据。

**整改内容：**
1. 在 `motor_HighFrequencyStep()` 内对以下区段增加 `FOC_HF_PROFILE_LEVEL >= 2` 分项计时：入口快照临界区（含 `work = *ptControl` 拷贝）、D 轴 PI、Q 轴 PI、位置源 Step；
2. 将现 commit 分项拆为两个子段：`foc_hal_SetDuty()` 调用（SetDuty 子段）与临界区写回/事件判断（publish 子段）；
3. `motor_hf_profile_snapshot_t` 扩展对应字段与有效位，心跳打印同步扩展；
4. **准入：** 各分项之和与 total 的残差 ≤ 300 cycles（即 Profile 自身开销上界），超出则先校准 DWT 读取成本。

### P1：结构体拷贝瘦身 + commit 三相合并直写

**目标：** 针对 P0 实测的两个最大项动刀，合计压降 400~650 cycles。

**整改内容：**
1. 若 P0 证实拷贝段为大头：消除 `work = *ptControl` 全量拷贝，改为在临界区内只读取本周期必需字段（eMode、tVoltageReference、tCurrentReference、tVoltageAlphaBeta 等），写出路径（publish）保持单临界区一次性写回；
2. **commit 合并直写（框架不破坏方案）：**
   - `foc_hal_SetDuty(ptPwm, u, v, w)` 接口与 `fnSetDuty` 绑定关系**不变**；
   - 在 `peripheral/stm32g431/port_mdi.c` 新增单次三相写函数，体即三条 LL 写：
     ```c
     /* 三相预装载原子组：一次调用写完 TIM1 三个 CCR */
     int32_t port_mdi_MotorPwmSetDuty3(uint32_t wU, uint32_t wV, uint32_t wW)
     {
         LL_TIM_OC_SetCompareCH1(TIM1, wU);
         LL_TIM_OC_SetCompareCH2(TIM1, wV);
         LL_TIM_OC_SetCompareCH3(TIM1, wW);
         return 0;
     }
     ```
   - `mdi_pwm_set_duty()`（foc_hal_mdi_adapter.c:72）内的 3× `MDI_Write` 改为单次调用该函数；占空比 ×period 换算保持原位；
   - 三个 `mdi_pwm_t` 设备**保留不删**——`MDI_Enable`（pwm_enable→haltim1_Start/Stop）与其他低频用户继续走 MDI 设备模型，modus 子模块零改动；
   - 其他芯片 target 不受影响：本改动只发生在 stm32g431 的 port/适配层，`foc/` 内核无感知；
3. **禁止**为此改变 PWM 预装载（preload）语义与刹车/急停路径；
4. **验收：** commit 分项 ≤ 400 cycles（若 P0 显示 publish 子段占大头，则该项验收以 SetDuty 子段 ≤ 100 cycles 计）。

### P2：采样链内联 + LTO

**目标：** 消除跨编译单元的小函数调用开销。

**整改内容：**
1. `haladc_GetInjected()` 移入 `peripheral/stm32g431/haladc.h` 作 `static inline`（函数体仅三目选 ADC + 读 JDR）；`.c` 中原定义删除，调用点（适配器 get_raw、foc_app.c 心跳 raw 读取）不变；
2. makefile 对 FOC 链路对象（`foc_*`、`motor*`、`haladc`、适配器）试开启 `-flto`，按 target 逐一验证链接与运行；`foc_hal_*` 包装层靠 LTO 自然消解，**不手工搬运到 header**；
3. **验收：** sample ≤ 300 cycles，total 较 P1 后再降 ≥ 5%。

### P3：发布形态固化

**整改内容：**
1. `BUILD=release` 固化 `FOC_HF_PROFILE=0`，并用 `llvm-nm` 校验 0 Profile 符号残留（沿用 V2.1 P0 准入）；
2. 保留 `BUILD=debug-rel`（-Os + Profile）作为日常调试默认；
3. 无感算法接入前，以本形态重测基线并更新本文 §2 表格。

---

## 5. 无感接入预算门槛（准入条件）

无感算法合入前必须满足：

1. 无感关闭时 total mean ≤ 3000 cycles（35%）、P99 ≤ 4250（50%）；
2. 无感算法自身（观测器 + PLL + 切换管理）在目标板上单独计量，mean ≤ 2500 cycles；
3. 合入后：mean ≤ 5500（65%）、P99 ≤ 6800（80%）、连续 100,000 周期无 overrun；
4. 无感代码遵守既有分层：观测器/PLL 在 `foc/observer/`，经 `foc_position_source_if_t` 接入，不触碰 `peripheral/`；硬件加速仅走适配层。

---

## 6. 验收标准与验证矩阵

1. **Host 单测：** `tests/foc` float / fixed / strict-aliasing 三组合全量 Pass；
2. **性能验收：** 每阶段完成后以心跳 Profile 实测对比本文 §2 基线，数据回写文档；STM32G431 门槛见 §3 表与 §5；
3. **正确性回归：** 每阶段验证三相电流零点（IDLE 时 Iu/Iv/Iw 均在 ±0.01 pu 内）、校准 offset 三相均在 20000~60000 合理区间、`motor status` 无故障位；
4. **硬件时序审计：** 凡本版触碰的初始化/时序代码，检查是否存在非 volatile 空转等待；新增硬件等待一律用 volatile 或 cycle-accurate 延时并加超时 log；
5. **跨芯片：** 本版改动集中在 `foc/motor/` 与 `peripheral/stm32g431/`，其他 target 行为不变；`-flto` 按 target 独立开关，不允许因某目标链接失败而全局开启。
