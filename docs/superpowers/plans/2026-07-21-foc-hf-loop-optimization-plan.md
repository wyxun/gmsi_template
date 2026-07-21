# FOC 高频控制环路资源占用优化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 系统性移除 FOC 内核对标准 libm 三角函数的依赖，利用 CORDIC 硬件加速和高速 LUT 插值软件后端，将 20kHz 电压开环高频步 CPU 占用降至 35% 以内。

**Architecture:** 
1. 提取电角度公共正余弦计算，改写 Park/IPark 为 `cached` 版本以避免重复计算。
2. 剥离标准 `floorf()`，编写基于无符号满量程 BAM32 定点映射的角度快速取整。
3. 建立三层三角后端：默认 LUT 插值软件后端（513项，1/4波，最大绝对误差 <1.2e-6），STM32G4 专属 CORDIC 寄存器级硬件后端（最大绝对误差 <2e-6），以及 libm 参考后端。

**Tech Stack:** C99, Cortex-M4F LL, STM32G4 CORDIC Co-processor, Python (table generation).

---

## Proposed Changes

### Task 1: 缓存 FOC 核心变换（Park/IPark）与 Profiling 宏门控

#### [MODIFY] [foc_core.h](file:///e:/Project/modus_template/foc/middleware/foc_core.h)
#### [MODIFY] [foc_core.c](file:///e:/Project/modus_template/foc/middleware/foc_core.c)
#### [MODIFY] [motor_control.c](file:///e:/Project/modus_template/foc/motor/motor_control.c)
#### [MODIFY] [foc_app.c](file:///e:/Project/modus_template/foc/app/foc_app.c)
#### [MODIFY] [stm32g4xx_it.c](file:///e:/Project/modus_template/target/stm32g431/stm32g4xx_it.c)

- [ ] **Step 1: 声明 cached 版本的 Park 和 IPark 变换**
  在 `foc_core.h` 的末尾添加以下声明：
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

- [ ] **Step 2: 实现 cached 变换并重构原 API 接口**
  在 `foc_core.c` 中，修改 `foc_park` 与 `foc_ipark` 如下，确保不破坏原接口兼容性：
  ```c
  foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                               foc_scalar_t qSin,
                               foc_scalar_t qCos,
                               foc_dq_t *ptDQ)
  {
      if (ptAB == NULL || ptDQ == NULL) {
          return FOC_RESULT_NULL;
      }
      ptDQ->qD = foc_add_sat(foc_mul_pu(ptAB->qAlpha, qCos),
                             foc_mul_pu(ptAB->qBeta, qSin));
      ptDQ->qQ = foc_sub_sat(foc_mul_pu(ptAB->qBeta, qCos),
                             foc_mul_pu(ptAB->qAlpha, qSin));
      return FOC_RESULT_OK;
  }

  foc_result_t foc_park(const foc_ab_t *ptAB,
                        foc_angle_t tTheta,
                        foc_dq_t *ptDQ)
  {
      return foc_park_cached(ptAB, foc_angle_sin(tTheta), foc_angle_cos(tTheta), ptDQ);
  }

  foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                                foc_scalar_t qSin,
                                foc_scalar_t qCos,
                                foc_ab_t *ptAB)
  {
      if (ptDQ == NULL || ptAB == NULL) {
          return FOC_RESULT_NULL;
      }
      ptAB->qAlpha = foc_sub_sat(foc_mul_pu(ptDQ->qD, qCos),
                                 foc_mul_pu(ptDQ->qQ, qSin));
      ptAB->qBeta = foc_add_sat(foc_mul_pu(ptDQ->qD, qSin),
                                foc_mul_pu(ptDQ->qQ, qCos));
      return FOC_RESULT_OK;
  }

  foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                         foc_angle_t tTheta,
                         foc_ab_t *ptAB)
  {
      return foc_ipark_cached(ptDQ, foc_angle_sin(tTheta), foc_angle_cos(tTheta), ptAB);
  }
  ```

- [ ] **Step 3: 提取公共三角计算并调用 cached 变换，同时引入 profiling 宏门控**
  在 `motor_control.c` 中：
  1. 引入 `g_wParkCycles` 等计时变量时用 `#if defined(MOTOR_PROFILE_CYCLES)` 包裹。
  2. 重构 `motor_HighFrequencyStep`：在最前面计算一次 `qSinTheta` 与 `qCosTheta`，其余处调用 `foc_park_cached` 和 `foc_ipark_cached`。
  具体代码修改块如下：
  ```c
  #if defined(MOTOR_PROFILE_CYCLES)
  volatile uint32_t g_wSampleCurrentCycles = 0;
  volatile uint32_t g_wClarkeCycles = 0;
  volatile uint32_t g_wParkCycles = 0;
  volatile uint32_t g_wIparkCycles = 0;
  volatile uint32_t g_wModulateCycles = 0;
  volatile uint32_t g_wCommitCycles = 0;
  #endif
  ```
  在 `motor_HighFrequencyStep` 中：
  ```c
      foc_scalar_t qSinTheta = foc_angle_sin(angle);
      foc_scalar_t qCosTheta = foc_angle_cos(angle);

  #if defined(MOTOR_PROFILE_CYCLES)
      uint32_t t2 = (uint32_t)get_system_ticks();
  #endif
      eResult = foc_park_cached(&current_alpha_beta, qSinTheta, qCosTheta, &work.tCurrent);
  #if defined(MOTOR_PROFILE_CYCLES)
      g_wParkCycles = (uint32_t)get_system_ticks() - t2;
  #endif
      if (eResult != FOC_RESULT_OK) {
          goto fail;
      }
      
      // ...
      
  #if defined(MOTOR_PROFILE_CYCLES)
      uint32_t t3 = (uint32_t)get_system_ticks();
  #endif
      eResult = foc_ipark_cached(&work.tVoltage, qSinTheta, qCosTheta, &work.tVoltageAlphaBeta);
  #if defined(MOTOR_PROFILE_CYCLES)
      g_wIparkCycles = (uint32_t)get_system_ticks() - t3;
  #endif
  ```

- [ ] **Step 4: 隐藏 profiling 门控宏相关的外部日志声明**
  在 `foc_app.c` 中：将打印 `g_wSampleCurrentCycles` 等外部变量的逻辑也用 `#if defined(MOTOR_PROFILE_CYCLES)` 包裹，未开启时降级到基础的心跳信息输出。
  在 `stm32g4xx_it.c` 中：也将 `g_wTestSinfCycles` 等临时计时变量用 `#if defined(MOTOR_PROFILE_CYCLES)` 宏包起来。

- [ ] **Step 5: 验证 Host 单测及 Target 构建**
  在 Host 下运行测试矩阵：
  ```powershell
  $env:Path = 'D:\software\msys64\mingw64\bin;D:\software\msys64\usr\bin;' + $env:Path
  mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
  ```
  预期结果：全量 Pass，无 Error。

---

### Task 2: 角度快速取整 `angle_wrap_scalar` 优化

#### [MODIFY] [foc_angle.c](file:///e:/Project/modus_template/foc/math/foc_angle.c)

- [ ] **Step 1: 重写 angle_wrap_scalar 函数**
  在 `foc_angle.c` 中，利用 VCVT 强转替换标准库 `floorf`：
  ```c
  static foc_scalar_t angle_wrap_scalar(foc_scalar_t qTurns)
  {
  #if defined(FOC_NUMERIC_FLOAT)
      /* 利用强转向零截断的性质构造向下取整 */
      int32_t ipart = (int32_t)qTurns;
      if (qTurns < 0.0f) {
          ipart -= 1;
      }
      qTurns -= (foc_scalar_t)ipart;
      /* 针对负整数圈产生的 1.0f 进行钳位归一，确保值域为严格的 [0.0f, 1.0f) */
      if (qTurns >= 1.0f) {
          qTurns = 0.0f;
      }
      return qTurns;
  #else
      qTurns %= FOC_ONE;
      return qTurns < FOC_ZERO ? qTurns + FOC_ONE : qTurns;
  #endif
  }
  ```

- [ ] **Step 2: 在 Host 验证精度与等价性**
  执行 Host 测试，保证浮点和定点双构建的功能正常：
  ```powershell
  mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
  ```

---

### Task 3: 通用软件查表（LUT）线性插值三角后端

#### [NEW] [gen_sin_lut.py](file:///e:/Project/modus_template/tools/gen_sin_lut.py)
#### [NEW] [foc_trig.h](file:///e:/Project/modus_template/foc/math/foc_trig.h)
#### [NEW] [foc_trig_lut.c](file:///e:/Project/modus_template/foc/math/foc_trig_lut.c)
#### [NEW] [test_trig.c](file:///e:/Project/modus_template/tests/foc/test_trig.c)
#### [MODIFY] [foc_config.h](file:///e:/Project/modus_template/foc/foc_config.h)
#### [MODIFY] [foc_angle.h](file:///e:/Project/modus_template/foc/math/foc_angle.h)
#### [MODIFY] [foc_angle.c](file:///e:/Project/modus_template/foc/math/foc_angle.c)
#### [MODIFY] [Makefile (tests)](file:///e:/Project/modus_template/tests/foc/Makefile)

- [ ] **Step 1: 编写 Python 三角查表生成脚本**
  创建 `tools/gen_sin_lut.py`：
  ```python
  import math

  def main():
      size = 512
      print("/* Automatically generated by gen_sin_lut.py. Do not edit. */")
      print('#include "foc_numeric.h"')
      print('#include "foc_trig.h"')
      print("")
      print("#if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)")
      print("const float s_afSinQuarter[513] = {")
      for i in range(size + 1):
          angle_rad = (i / size) * (math.pi / 2.0)
          val = math.sin(angle_rad)
          print(f"    {val:.9f}f,")
      print("};")
      print("#endif")

  if __name__ == "__main__":
      main()
  ```

- [ ] **Step 2: 生成 s_afSinQuarter 数据表**
  在终端中执行 Python 脚本，重定向生成 `foc_trig_lut.c` 的前身：
  ```powershell
  python tools/gen_sin_lut.py > foc/math/foc_trig_lut.c
  ```

- [ ] **Step 3: 定义三角后端接口 `foc_trig.h`**
  创建 `foc/math/foc_trig.h`：
  ```c
  #ifndef FOC_TRIG_H
  #define FOC_TRIG_H

  #include "foc_numeric.h"
  #include "foc_angle.h"

  #define FOC_TRIG_BACKEND_LIBM    0
  #define FOC_TRIG_BACKEND_LUT     1
  #define FOC_TRIG_BACKEND_CORDIC  2

  /* 依据 target.mk 或 外部 Makefile 决定默认后端，host 默认 LUT */
  #ifndef FOC_TRIG_BACKEND
  #define FOC_TRIG_BACKEND FOC_TRIG_BACKEND_LUT
  #endif

  typedef struct {
      void (*fnSinCosTurns)(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
      foc_scalar_t (*fnAtan2Turns)(foc_scalar_t qY, foc_scalar_t qX);
  } foc_trig_backend_t;

  #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
  extern const float s_afSinQuarter[513];
  float lut_sin_turns(float fTurns);
  #endif

  #endif /* FOC_TRIG_H */
  ```

- [ ] **Step 4: 实现 LUT 查表插值算法**
  在 `foc/math/foc_trig_lut.c` 中追加查表与对称插值的 C 实现：
  ```c
  #include "foc_trig.h"

  #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
  float lut_sin_turns(float fTurns)
  {
      float fPhase = fTurns * 4.0f;
      uint32_t wQuad = (uint32_t)fPhase;
      float fFrac = (fPhase - (float)wQuad) * 512.0f;
      uint32_t wIdx = (uint32_t)fFrac;
      float fLerp = fFrac - (float)wIdx;

      if (wQuad & 1) { /* 象限 1 和 3 */
          wIdx = 511 - wIdx;
          fLerp = 1.0f - fLerp;
      }

      float fY0 = s_afSinQuarter[wIdx];
      float fY1 = s_afSinQuarter[wIdx + 1];
      float fRes = fY0 + (fY1 - fY0) * fLerp;

      if (wQuad >= 2) { /* 象限 2 和 3 */
          fRes = -fRes;
      }
      return fRes;
  }
  #endif
  ```

- [ ] **Step 5: 接入 `foc_config.h` 并修改 `foc_angle.c` 编译期后端绑定**
  1. 在 `foc_config.h` 中包含 `#include "foc_trig.h"`。
  2. 在 `foc_angle.c` 中：根据宏静态绑定 `foc_angle_sin`、`foc_angle_cos`：
  ```c
  #include "foc_trig.h"

  foc_scalar_t foc_angle_sin(foc_angle_t tAngle)
  {
  #if defined(FOC_NUMERIC_FLOAT)
    #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
      return lut_sin_turns(angle_wrap_scalar(tAngle.qTurns));
    #else
      return sinf(angle_wrap_scalar(tAngle.qTurns) * FOC_TWO_PI_F);
    #endif
  #else
      return angle_sin_fixed(tAngle.qTurns);
  #endif
  }
  ```

- [ ] **Step 6: 编写 `tests/foc/test_trig.c` 精度单元测试**
  创建单测文件，对 `[0, 1)` 圈进行 $2^{16}$ 步进采样扫描，与标准库断言误差 $< 2\times 10^{-6}$。
  将 `test_trig.c` 添加 to `tests/foc/Makefile` 的测试清单中，并在 Host 上执行：
  ```powershell
  mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
  ```

---

### Task 4: CORDIC 寄存器级硬件后端（STM32G431）

#### [NEW] [halcordic.h](file:///e:/Project/modus_template/peripheral/stm32g431/halcordic.h)
#### [NEW] [halcordic.c](file:///e:/Project/modus_template/peripheral/stm32g431/halcordic.c)
#### [MODIFY] [target.mk](file:///e:/Project/modus_template/target/stm32g431/target.mk)
#### [MODIFY] [foc_angle.c](file:///e:/Project/modus_template/foc/math/foc_angle.c)

- [ ] **Step 1: 编写 CORDIC 底层驱动定义 `halcordic.h`**
  创建 `peripheral/stm32g431/halcordic.h`：
  ```c
  #ifndef HALCORDIC_H
  #define HALCORDIC_H

  #include "foc_numeric.h"

  void hal_cordic_Init(void);
  void hal_cordic_SinCos(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
  foc_scalar_t hal_cordic_Atan2(foc_scalar_t qY, foc_scalar_t qX);

  #endif /* HALCORDIC_H */
  ```

- [ ] **Step 2: 寄存器级驱动实现 `halcordic.c`**
  创建 `peripheral/stm32g431/halcordic.c`。通过直接对 RCC 和 CORDIC 寄存器操作，实现高速的零等待读取：
  ```c
  #include "halcordic.h"
  #include "stm32g4xx.h"
  #include <math.h>

  void hal_cordic_Init(void)
  {
      /* 开启 CORDIC 外设时钟 */
      RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
      __DSB();
  }

  void hal_cordic_SinCos(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos)
  {
      /* 1. Turns [0, 1) 映射到 q1.31 定点二进制角度 BAM32 */
      uint32_t u32Turns = (uint32_t)(qTurns * 4294967296.0f);
      int32_t q31Angle = (int32_t)u32Turns;

      /* 2. 配置 CSR：Cosine功能，24次迭代(PRECISION=6), 双结果读取(NRES=1) */
      CORDIC->CSR = (6U << CORDIC_CSR_PRECISION_Pos) | 
                    (0U << CORDIC_CSR_FUNC_Pos) | 
                    CORDIC_CSR_NRES;

      /* 3. 写入 WDATA 触发硬件计算 (零等待总线阻塞式) */
      CORDIC->WDATA = q31Angle;

      /* 4. 依次读取 X_RESULT (cos) 和 Y_RESULT (sin) */
      int32_t qCos = CORDIC->RDATA;
      int32_t qSin = CORDIC->RDATA;

      /* 5. 格式转换回 float */
      *pqCos = (float)qCos * 4.6566128730773926e-10f;
      *pqSin = (float)qSin * 4.6566128730773926e-10f;
  }

  foc_scalar_t hal_cordic_Atan2(foc_scalar_t qY, foc_scalar_t qX)
  {
      float fMax = fmaxf(fabsf(qX), fabsf(qY));
      if (fMax <= 0.0f) {
          return 0.0f;
      }
      
      /* 归一化输入，防止 CORDIC Q1.31 溢出 */
      int32_t q31X = (int32_t)((qX / fMax) * 2147483647.0f);
      int32_t q31Y = (int32_t)((qY / fMax) * 2147483647.0f);

      /* 配置 CSR：Phase (atan2), NARGS=1 (双参数，X+Y), NRES=0 (单结果) */
      CORDIC->CSR = (6U << CORDIC_CSR_PRECISION_Pos) | 
                    (2U << CORDIC_CSR_FUNC_Pos) | 
                    CORDIC_CSR_NARGS;

      /* 写入 X 紧接着写入 Y 触发计算 */
      CORDIC->WDATA = q31X;
      CORDIC->WDATA = q31Y;

      /* 读取 RDATA 获得 BAM32 角度 */
      int32_t qAngle = CORDIC->RDATA;

      /* 转换为 turns 范围 [0, 1) */
      float fTurns = (float)qAngle * 2.3283064365386963e-10f;
      if (fTurns < 0.0f) {
          fTurns += 1.0f;
      }
      return fTurns;
  }
  ```

- [ ] **Step 3: 将 CORDIC 配置接入 G431 构建路径并静态派发**
  1. 在 `target/stm32g431/target.mk` 中，设置：
     ```makefile
     C_DEFS += -DFOC_TRIG_BACKEND=2
     ```
     并将 `peripheral/stm32g431/halcordic.c` 附加到 `C_SOURCES`。
  2. 在 `foc_angle.c` 中，将 CORDIC 接口派发给 `foc_angle_sin`、`foc_angle_cos` 及 `foc_angle_atan2`：
     ```c
     #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
     #include "halcordic.h"
     #endif

     foc_scalar_t foc_angle_sin(foc_angle_t tAngle)
     {
     #if defined(FOC_NUMERIC_FLOAT)
       #if (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_CORDIC)
         foc_scalar_t qSin, qCos;
         hal_cordic_SinCos(tAngle.qTurns, &qSin, &qCos);
         return qSin;
       #elif (FOC_TRIG_BACKEND == FOC_TRIG_BACKEND_LUT)
         return lut_sin_turns(angle_wrap_scalar(tAngle.qTurns));
       #else
         return sinf(angle_wrap_scalar(tAngle.qTurns) * FOC_TWO_PI_F);
       #endif
     // ...
     ```

- [ ] **Step 4: 上机烧录与开环性能量化对照**
  通过 `mingw32-make BUILD=debug-rel TARGET_CHIP=stm32g431` 编译并烧录。
  重新拉起 RTT 服务器，以 CORDIC 后端运行电机，验证 `HF_cycles` 数据是否稳定在 3000 以下，且波形无异常跳变。

---

### Task 5: 结构性瘦身与编译器级优化（`-O2` 门控）

#### [MODIFY] [Makefile](file:///e:/Project/modus_template/makefile)
#### [MODIFY] [motor_control.c](file:///e:/Project/modus_template/foc/motor/motor_control.c)

- [ ] **Step 1: 开启 foc 目录的编译器 `-O2` 优化**
  在主 `Makefile` 的适当位置，为 `foc` 目录（尤其是 `foc/math` 和 `foc/middleware`）增加目录级的编译标志，强制开启 `-O2`：
  ```makefile
  build/foc_%.o: CFLAGS += -O2
  ```

- [ ] **Step 2: 浮点常量与除法倒数静态分析检查**
  1. 使用 ripgrep 对 `foc` 目录扫描，确保所有浮点字面量有 `f` 后缀，没有发生隐式的 double 提升。
  2. 针对高频路径中的 `x / 8.0f` 一类除法运算，重构为乘以 `0.125f` 预取倒数。

- [ ] **Step 3: 全量自动化回归与交付**
  1. Host 功能测试与 strict-aliasing 规则测试全跑一遍。
  2. 上机量化并记录最终性能表，保存到 Walkthrough 总结报告。

---

## Verification Plan

### Automated Tests
- 在 Host 执行完整测试矩阵验证：
  `mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all`
  `mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2' clean all`

### Manual Verification
- 编译并在目标板（G431）上烧录：
  `mingw32-make BUILD=debug-rel TARGET_CHIP=stm32g431`
  `mingw32-make flash BUILD=debug-rel TARGET_CHIP=stm32g431`
- 通过 Python 脚本和 RTT 终端，抓取电机运行时的性能统计，记录 `HF_cycles` 并对照输出正弦电流波形是否平滑。
