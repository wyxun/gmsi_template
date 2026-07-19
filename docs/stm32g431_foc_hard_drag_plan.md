# STM32G431 FOC 高频环接通与小电流硬拖验证计划

日期：2026-07-19
目标芯片：stm32g431（Makefile 默认 TARGET_CHIP）
参考工程：`reference/STOPLL_FOC_2205/`（同一块板子、同一台 2205 云台电机，ST MCSDK 6.4.1 生成）

---

## 1. 要实现的功能

1. **接通 FOC 高频控制环**：TIM1 CH4 触发 ADC 注入转换完成（每个 20 kHz PWM
   载波一次）→ `ADC1_2_IRQHandler` → `foc_app_HighFrequencyISR()` →
   `motor_HighFrequencyStep()`。目前该链路在 stm32g431 上是断的，高频步从未运行。
2. **小电流硬拖（电压开环强拖）**：内部开环角度发生器以 1 e-turn/s 旋转电角度，
   Vq = 0.03 pu（12V 母线下约 0.36V，堵转电流约 0.14A）。选电压开环是因为
   **电流反馈不在控制环内**——即使采样线序/极性接错也不会失控过流，是验证
   采样链最安全的方式。
3. **验证三相电流采样线序**：通过 RTT 波形（1 kHz 推送 DutyU/V/W + Iu/Iv/Iw）
   和 shell `motor status` 观察三相电流，应为零均值、互差 120°、U→V→W 次序的
   正弦波。

---

## 2. 硬件端口配置现状（已逐位核对，与参考工程一致）

### 2.1 电流采样模拟前端（OPAMP 差分）

硬件为三电阻采样，每相采样电阻两端接到一路 OPAMP 的差分输入，
`peripheral/stm32g431/halopamp.c` 配置如下（与硬件原理图、参考工程
`main.c` 完全一致）：

| 相 | 信号 | OPAMP | VINP(+) | VINM0(-) | VOUT | 模式/增益 |
|----|------|-------|---------|----------|------|-----------|
| U | Curr_fd1 +/- | OPAMP1 | PA1 | PA3 | PA2 → ADC1_IN3 | PGA_IO0_BIAS, x16 |
| V | Curr_fd2 +/- | OPAMP2 | PA7 | PA5 | PA6 → ADC2_IN3 | PGA_IO0_BIAS, x16 |
| W | Curr_fd3 +/- | OPAMP3 | PB0 | PB2 | PB1 → ADC1_IN12（外部脚）+ 内部 VOPAMP3 → ADC2（未用） | PGA_IO0_BIAS, x16 |

- GPIO：PA1/2/3、PA5/6/7、PB0/1/2 全部 analog 无上下拉（`halopamp.c:16-35`）。
- OPAMP1/2 内部输出关闭（走外部引脚回读 ADC）；OPAMP3 内部输出使能
  （冗余，ADC2 的 VOPAMP3 通道采样结果当前无人消费）。
- 工厂 trimming，NORMALSPEED。
- 参考工程备注：`power_stage_parameters.h` 里 `AMPLIFICATION_GAIN 9.14` 是
  含 VINM 外部偏置网络的等效增益，不是裸 PGA 增益；当前软件只做
  `(raw - offset)/offset` 归一化，不换算物理电流，硬拖验证线序不需要它。

### 2.2 ADC 注入组（三相电流，`peripheral/stm32g431/haladc.c`）

- 时钟：PLL_P（42.5 MHz），12 bit 右对齐，单端。
- 触发：`LL_ADC_INJ_TRIG_EXT_TIM1_CH4` 上升沿（G4 上 TIM1_CH4 是直连
  OC4REF 的独立触发源，不依赖 TRGO/TRGO2 配置，两条路径都成立）。
- 采样时间：注入通道 6.5 cycles；队列模式 DISABLE；双 ADC 均已
  `LL_ADC_INJ_StartConversion` 武装。
- 注入序列（与参考工程 rank→相 映射逐位一致）：

| ADC | Injected Rank 1 | Injected Rank 2 |
|-----|-----------------|-----------------|
| ADC1 | CH3 = PA2 = OPAMP1 出 = **U** | CH12 = PB1 = OPAMP3 出 = **W** |
| ADC2 | VOPAMP3 内部 = W（冗余未用） | CH3 = PA6 = OPAMP2 出 = **V** |

- 相序取用（`foc_hal_mdi_adapter.c:128-138`）：
  `U = ADC1 rank1`，`V = ADC2 rank2`，`W = ADC1 rank2`。**映射正确**。

### 2.3 ADC 常规组（母线电压/温度/电位器）

- PA0 = ADC1_IN1 母线电压、PB12 = ADC1_IN11 电位器、PB14 = ADC1_IN5 温度；
  软件触发，无 DMA，47.5 cycles 采样（`haladc.c:76-119`）。

### 2.4 PWM / 触发定时器（`peripheral/stm32g431/haltim1.c`）

- TIM1，20 kHz 中心对齐（CENTER_DOWN），ARR 预载关闭，RepetitionCounter=1。
- PA8/PA9/PA10 = CH1/CH2/CH3（U/V/W 高边）；PC13/PA12/PB15 = CH1N/CH2N/CH3N
  （低边）；互补输出带死区，刹车输入来自 COMP1/2/4（硬件过流保护）。
- CH4 = PWM2 模式，`CCR4 = ARR/2 - 10`（载波中心略前），OC4REF 为 ADC
  注入触发；`LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4)`（冗余但无害）。
- 计数器在初始化时即启动（`haltim1.c:111`），输出由 `haltim1_Start`/MDI 使能
  才打开——因此**零偏校准期间注入触发持续产生，JDR 数据是新鲜的**，校准
  轮询无需修改。

### 2.5 定时/系统

- SysTick 1 ms → `HAL_IncTick` + perf_counter + `modus_Clock` +
  `peripheral_Clock`；FOC 低频步 `motor_LowFrequencyStep()` 挂在
  `foc_app_Clock()`（1 ms）。

---

## 3. 现状数据链路（重点：断点）

```
TIM1 CH4 (OC4REF 上升沿, 20 kHz)
  → ADC1/ADC2 注入转换（已配置、已武装）✅
  → JEOS 中断源使能        ❌ 全工程无 LL_ADC_EnableIT_JEOS
  → NVIC ADC1_2_IRQn 使能  ❌ 全工程无 HAL_NVIC_EnableIRQ(ADC1_2_IRQn)
  → ADC1_2_IRQHandler      ⚠️ 只清 JEOS 标志，不调 FOC（stm32g4xx_it.c:75-79）
  → foc_app_HighFrequencyISR() → motor_HighFrequencyStep() ❌ 从未被调用
  → 采样重构 mdi_adc_reconstruct → clarke/park → SVPWM → 占空比 ✅ 代码就绪
```

对照 at32f413（已工作的版本）：
- `peripheral/at32f413/haladc.c`：`adc_interrupt_enable(ADC1, ADC_PCCE_INT)` +
  `nvic_irq_enable(ADC1_2_IRQn, 1, 0)`。
- `target/at32f413/at32f413_it.c:72-81`：查 PCCE 标志 → 清标志 →
  `foc_app_HighFrequencyISR()`（`motor_HighFrequencyStep()` 唯一调用点）。

---

## 4. 遗漏点清单（需要更改的地方）

| # | 位置 | 问题 |
|---|------|------|
| 1 | `peripheral/stm32g431/haladc.c` | JEOS 中断源未使能；NVIC `ADC1_2_IRQn` 未配置/未使能 |
| 2 | `target/stm32g431/stm32g4xx_it.c` | `ADC1_2_IRQHandler` 只清标志，未调 `foc_app_HighFrequencyISR()`；`TIM1_UP_TIM16_IRQHandler` 内有过时 TODO；文件头注释与实际不符 |
| 3 | `target/stm32g431/target.mk:7-8` | 注释仍写"FOC 高频 ISR 故意不接" |
| 4 | `foc/app/foc_app.c` `s_tMotorConfig.tParams` | 电机参数是占位值（24V/10A/4 极对/100 mΩ/100 µH），与 2205 电机（12V/0.8A/7 极对/2.5 Ω/500 µH）不符 |
| 5 | `foc/app/foc_app.c` `FOC_APP_VOLTAGE_REF_Q` | 0.05 pu 硬拖电流偏大（约 0.24A），降为 0.03 pu（约 0.14A） |

### 已评估、确认无需修改

- ADC/OPAMP 初始化：与参考工程逐位一致（见第 2 节）。
- `mdi_adc_offset_calib` 背靠背轮询：TIM1 计数器常开，JDR 20 kHz 持续刷新，
  200 次平均有效，不会读陈旧值。
- GRBLHAL 条件编译块：stm32g431 不构建 grblHAL，不动。
- 不启用 ADC2 VOPAMP3 冗余通道；不加 `motor iq` 命令（电压开环硬拖够用，
  运行时可用 `motor vq <x>` 微调）。

---

## 5. 修改清单（具体改动）

### 5.1 `peripheral/stm32g431/haladc.c` — 使能 JEOS 中断 + NVIC

`haladc_Init()` 在两个 ADC 初始化完成后追加（对齐 at32 侧做法，优先级 1/0）：

```c
void haladc_Init(void)
{
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* 注入序列完成中断：双 ADC 同沿触发、等长等速序列，
     * 只需 ADC1 的 JEOS 作为高频环时基（同 MCSDK 做法）。 */
    LL_ADC_EnableIT_JEOS(ADC1);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}
```

（需补 `#include "stm32g4xx_hal.h"` 以获得 HAL_NVIC_* 声明。）

### 5.2 `target/stm32g431/stm32g4xx_it.c` — 接通 FOC 高频入口

对齐 `target/at32f413/at32f413_it.c:72-81` 的结构：

```c
/* ---- ADC1_2 (current sensing end-of-conversion) ---- */
void ADC1_2_IRQHandler(void)
{
    if ((ADC1->ISR & ADC_ISR_JEOS) != 0U) {
        ADC1->ISR = ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOS;
        /* FOC 高频控制步：TIM1 CH4 触发的注入转换完成，
         * 每个 20 kHz PWM 载波一次。这是 motor_HighFrequencyStep()
         * 的唯一调用点（经 app 层）。 */
        foc_app_HighFrequencyISR();
    }
}
```

同时：
- 顶部补 `#include "foc_app.h"`；
- 删除 `TIM1_UP_TIM16_IRQHandler` 里过时的 `/* TODO: FOC control loop entry */`；
- 修正文件头注释使其与实际一致。

### 5.3 `foc/app/foc_app.c` — 电机参数对齐 2205 + 小电流硬拖

`s_tMotorConfig.tParams`（值取自参考工程 `pmsm_motor_parameters.h` /
`power_stage_parameters.h` / `drive_parameters.h`）：

| 字段 | 现值 | 新值 | 来源 |
|------|------|------|------|
| `wResistanceMilliOhm` | 100 | 2500 | RS = 2.5 Ω |
| `wLdMicroHenry` / `wLqMicroHenry` | 100 | 500 | LS = 0.5 mH |
| `wRatedVoltageMilliVolt` | 24000 | 12000 | 母线 12V |
| `wRatedCurrentMilliAmp` | 10000 | 800 | NOMINAL_CURRENT 0.8A |
| `wBackEmfMicroVoltPerRadSec` | 10000 | 23000 | 1.7 Vrms/kRPM ≈ 22.96 mV·s/rad |
| `chPolePairs`（tParams 与 tPosition 两处） | 4 | 7 | POLE_PAIR_NUM 7 |

硬拖 run config（保持 `MOTOR_CONTROL_VOLTAGE_OPEN_LOOP` 不变）：

- `FOC_APP_VOLTAGE_REF_Q`：`0.05f → 0.03f`（≈0.36V @12V 母线，堵转电流
  约 0.14A；运行时可 `motor vq <x>` 调整）。
- `FOC_APP_OPEN_LOOP_SPEED` 保持 `1.0f`（1 e-turn/s，7 极对下 ≈ 8.6 RPM）。
- 更新第 33 行注释，注明参数来源为参考工程 2205 电机。

### 5.4 `target/stm32g431/target.mk:7-8` — 更新过时注释

把"FOC high-frequency ISR is intentionally NOT wired … yet"改为：
已通过 ADC1 JEOS 中断接至 `foc_app_HighFrequencyISR()`（见
`stm32g4xx_it.c` 与 `haladc.c`）。

---

## 6. 验证

1. **构建**：
   - `mingw32-make TARGET_CHIP=stm32g431 clean all`（工具链路径问题加
     `SW_ROOT=D:/software`，见 task8 交接）。
   - `mingw32-make TARGET_CHIP=at32f413 all` 确认公共文件 `foc_app.c`
     改动不影响 at32 构建。
2. **host 测试**：`mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all`
   应全 PASS（foc 核心逻辑未动）。
3. **上电硬拖验证**（人工执行）：
   - flash 后 shell 执行 `motor start`（或按按钮）；
   - RTT 波形（`make.bat rtt` 端口 9091）观察 Iu/Iv/Iw：应为三条零均值、
     互差 120°、U→V→W 次序的正弦；
   - `motor status` 查看快照电流/占空比/角度；
   - 判定：某相恒为 0 → 该相采样链断；次序错 → 两相互换；某相反相 →
     该相 OPAMP 输入对调或软件取反。按第 2.1/2.2 节表格核对。
