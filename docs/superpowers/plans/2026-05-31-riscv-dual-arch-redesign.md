# Modus v0.5.0.x 双架构升级与高内聚重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `modus` 核心库中原生增加通用 RISC-V 架构周期测量与异常诊断支持，同时在 `modus_template` 中完成 Cortex-M 与 RISC-V 全芯片胶水代码的彻底净化，并以沁恒 `CH592` 为 Demo 验证载体打通 100% 的命令行一键编译与 Mstudio RTT 上位机调试。

**Architecture:** 
1. 核心库 `modus/src/mdebug/perfc_port.c` 宏门控统一收归，统一暴露时钟获取 API。
2. `target.mk` 直接重用 vendor 官方子模块的原始汇编启动文件。
3. 编写强符号 `ch592_exception.S`，对官方弱符号 HardFault 向量进行零污染替换与诊断拦截。
4. 组件目录在 `peripheral/` 下完全物理对齐，target 彻底净化。

**Tech Stack:** C11, RISC-V Assembly, GNU Make, LLVM (Clang 21, LLD, llvm-nm), OpenOCD, SEGGER RTT.

---

### Task 1: Modus 核心库 `perfc_port.c` 双架构统一移植

**Files:**
- Create: `e:/Project/modus/src/mdebug/perfc_port.c`
- Modify: `e:/Project/modus/modus.mk`

- [ ] **Step 1: 创建统一的底层移植源文件 `perfc_port.c`**
  写入以下通用移植逻辑，通过 `__riscv` 与 `__ARM_ARCH` 分支门控：
  
  ```c
  #undef __PERF_COUNT_PLATFORM_SPECIFIC_HEADER__
  #include <stdint.h>
  #include <stdbool.h>

  #define __IMPLEMENT_PERF_COUNTER
  #include "perf_counter.h"

  /* 全局统一的时钟解耦桥梁 */
  __attribute__((weak)) uint32_t get_system_core_clock_hz(void)
  {
      return 60000000UL; /* 默认兜底频率 */
  }

  #if defined(__riscv)
  /*============================ RISC-V 架构通用移植 ==============================*/

  static inline int64_t get_riscv_cycle64(void)
  {
      uint32_t cycle_l, cycle_h1, cycle_h2;
      do {
          __asm__ __volatile__("csrr %0, cycleh" : "=r"(cycle_h1));
          __asm__ __volatile__("csrr %0, cycle" : "=r"(cycle_l));
          __asm__ __volatile__("csrr %0, cycleh" : "=r"(cycle_h2));
      } while (cycle_h1 != cycle_h2);
      return (((int64_t)cycle_h1) << 32) | cycle_l;
  }

  bool perfc_port_init_system_timer(bool bIsTimeOccupied)
  {
      (void)bIsTimeOccupied;
      return true;
  }

  uint32_t perfc_port_get_system_timer_freq(void)
  {
      return get_system_core_clock_hz();
  }

  bool perfc_port_is_system_timer_ovf_pending(void)
  {
      return false;
  }

  int64_t perfc_port_get_system_timer_top(void)
  {
      return 0x7FFFFFFFFFFFFFFFLL;
  }

  int64_t perfc_port_get_system_timer_elapsed(void)
  {
      return get_riscv_cycle64();
  }

  void perfc_port_clear_system_timer_ovf_pending(void) {}
  void perfc_port_stop_system_timer_counting(void) {}
  void perfc_port_clear_system_timer_counter(void) {}

  __attribute__((noinline)) uintptr_t __perfc_port_get_sp(void)
  {
      uintptr_t result;
      __asm__ volatile ("mv %0, sp" : "=r" (result));
      return result;
  }

  __attribute__((noinline)) void __perfc_port_set_sp(uintptr_t nSP)
  {
      uintptr_t nAlignSP = nSP;
      __asm__ volatile ("mv sp, %0" : : "r" (nAlignSP));
  }

  #elif defined(__ARM_ARCH) || defined(__CORTEX_M) || defined(__arm__)
  /*=========================== Cortex-M 架构通用移植 ==============================*/

  #include "core_cm.h" /* 使用 CMSIS 标准 DWT 寄存器定义 */

  bool perfc_port_init_system_timer(bool bIsTimeOccupied)
  {
      (void)bIsTimeOccupied;
      /* 开启 DWT 硬件周期计数器 */
      #if defined(__DWT_PRESENT) && __DWT_PRESENT
          CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
          DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
      #endif
      return true;
  }

  uint32_t perfc_port_get_system_timer_freq(void)
  {
      return get_system_core_clock_hz();
  }

  bool perfc_port_is_system_timer_ovf_pending(void)
  {
      return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0;
  }

  int64_t perfc_port_get_system_timer_top(void)
  {
      return (int64_t)SysTick->LOAD;
  }

  int64_t perfc_port_get_system_timer_elapsed(void)
  {
      #if defined(__DWT_PRESENT) && __DWT_PRESENT
          return (int64_t)DWT->CYCCNT;
      #else
          return (int64_t)(SysTick->LOAD - SysTick->VAL);
      #endif
  }

  void perfc_port_clear_system_timer_ovf_pending(void) {}
  void perfc_port_stop_system_timer_counting(void) {}
  void perfc_port_clear_system_timer_counter(void) {}

  __attribute__((noinline)) uintptr_t __perfc_port_get_sp(void)
  {
      uintptr_t result;
      __asm__ volatile ("mrs %0, msp" : "=r" (result));
      return result;
  }

  __attribute__((noinline)) void __perfc_port_set_sp(uintptr_t nSP)
  {
      uintptr_t nAlignSP = nSP;
      __asm__ volatile ("msr msp, %0" : : "r" (nAlignSP));
  }

  #endif
  ```

- [ ] **Step 2: 修改 `modus.mk` 注册通用移植文件**
  修改 `e:/Project/modus/modus.mk` 第 65-71 行的 `MODUS_SRCS_DEBUG`，将原来的 `$(MODUS_ROOT)/src/mdebug/mdebug_riscv.c` 去掉（或保留），并追加 `$(MODUS_ROOT)/src/mdebug/perfc_port.c`。
  
  ```diff
   MODUS_SRCS_DEBUG = \
       $(MODUS_ROOT)/src/mdebug/mshell.c \
       $(MODUS_ROOT)/src/mdebug/trace.c \
       $(MODUS_ROOT)/src/mdebug/trace_fmt.c \
       $(MODUS_ROOT)/src/mdebug/util_debug.c \
  +    $(MODUS_ROOT)/src/mdebug/perfc_port.c \
       $(MODUS_ROOT)/src/mdebug/mdebug_riscv.c \
       $(MODUS_ROOT)/src/mdebug/segger_rtt/SEGGER_RTT.c
  ```

- [ ] **Step 3: 同步复制到子模块的 `modus` 目录**
  确保 `modus_template` 的 `e:/Project/modus_template/modus` 子模块中同步完成 `perfc_port.c` 的复制和 `modus.mk` 的修改。
  指令: `powershell -NoProfile -Command "Copy-Item e:/Project/modus/src/mdebug/perfc_port.c e:/Project/modus_template/modus/src/mdebug/perfc_port.c"`

- [ ] **Step 4: 提交**
  ```bash
  git add src/mdebug/perfc_port.c modus.mk
  git commit -m "feat(mdebug): unify perf_counter porting for CM and RISCV into perfc_port.c"
  ```

---

### Task 2: 模板仓库全芯片胶水代码彻底废除与 makefile 重构

**Files:**
- Modify: `e:/Project/modus_template/makefile`
- Delete: 全芯片 `target/<chip_name>/perfc_port_user.c` 与 `.h`

- [x] **Step 1: 删除全芯片 target 下散落的 perf_counter 移植文件**
  运行 PowerShell 彻底删除以下胶水：
  `powershell -NoProfile -Command "Remove-Item -Force e:/Project/modus_template/target/stm32g431/perfc_port_user.*"`
  `powershell -NoProfile -Command "Remove-Item -Force e:/Project/modus_template/target/at32f421/perfc_port_user.*"`
  `powershell -NoProfile -Command "Remove-Item -Force e:/Project/modus_template/target/at32f413/perfc_port_user.*"`

- [x] **Step 2: 修改 `makefile` 进行协议映射与宏简化**
  修改 `e:/Project/modus_template/makefile`：
  1. 第 14 行添加小写 `target` 映射（已在 Spec 中通过）：
     ```make
     ifdef target
         TARGET_CHIP = $(target)
     endif
     TARGET_CHIP ?= stm32g431
     ```
  2. 废除 `C_DEFS` 里的自定义替换宏 `-D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port_user.h\"`，保留基本门控：
     ```make
     C_DEFS += \
         -D__PERFC_USE_USER_CUSTOM_PORTING__=1 \
         -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1 \
         -D__COMPILER_HAS_GNU_EXTENSIONS__=1 \
         -DTRACE_USE_LIBC_PRINTF=0 \
         -DTRACE_MCU_WRITE_STRING="extern void user_trace_output(const char*); user_trace_output" \
         -DMODUS_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""
     ```

- [x] **Step 3: 验证 Cortex-M 芯片在“零冗余”架构下的编译**
  运行: `.\make.bat target=stm32g431 clean all`
  验证: 固件顺利生成，且无任何关于 SysTick、DWT 或 `perfc_port_user.h` 的缺失报错。

- [x] **Step 4: Commit (Deferred per parent agent request — wait for centralized commit)**
  ```bash
  # git commit was skipped locally per high-priority subagent instructions to avoid fragmented commits
  ```

---

### Task 3: CH592 验证载体底层驱动组件对称化开发

**Files:**
- Create: `e:/Project/modus_template/peripheral/ch592/port_sys.c`
- Create: `e:/Project/modus_template/peripheral/ch592/port_mdi.c`
- Delete: 原有 target 下分散的手写底层驱动
- Modify: `e:/Project/modus_template/target/ch592/target.mk`

- [x] **Step 1: 删除 target 下原有的杂乱芯片驱动**
  彻底净化 target 目录：
  `powershell -NoProfile -Command "Remove-Item -Force e:/Project/modus_template/target/ch592/peripheral_ch592.*; Remove-Item -Force e:/Project/modus_template/target/ch592/system_ch592.c; Remove-Item -Force e:/Project/modus_template/target/ch592/startup_CH592.S"`

- [x] **Step 2: 创建高内聚的系统初始化源文件 `port_sys.c`**
  写入以下代码，直接引用沁恒官方 `CH592SFR.h` 寄存器定义：
  
  ```c
  #include "peripheral.h"
  #include "CH592SFR.h"
  #include <stdint.h>
  #include <stdbool.h>

  /* 系统内核时钟强符号回传，完美覆盖 perf_counter 中的弱符号 */
  uint32_t SystemCoreClock = 60000000UL;

  uint32_t get_system_core_clock_hz(void)
  {
      return SystemCoreClock;
  }

  /* 官方中断向量表默认调用的时钟/SysTick初始化入口 */
  void SystemInit(void)
  {
      /* 1. 重置并配置硬件 SysTick 全局自减计数器 */
      SysTick->CTLR = 0;
      SysTick->CNTL = 0;
      SysTick->CNTH = 0;
      SysTick->SR = 0;

      /* 2. 配置 60MHz 时钟下 1ms 的比较中断值 (60,000 counts) */
      SysTick->CMPLR = 60000UL;
      SysTick->CMPHR = 0;

      /* 3. 开启 System Timer，使用 HCLK 作为基准并开启中断 */
      SysTick->CTLR = STK_CTLR_STE | STK_CTLR_STIE | STK_CTLR_STCLK;
  }

  /* 串口0波特率配置算法 */
  static void Uart0_Init(uint32_t baudrate)
  {
      /* 1. 引脚配置: PB7(TX0) 设为输出高电平，PB4(RX0) 设为输入 */
      R32_PB_OUT |= (1 << 7);
      R32_PB_DIR |= (1 << 7);
      R32_PB_DIR &= ~(1 << 4);

      /* 2. 算波特率分频因子 (16C550 整除公式) */
      uint32_t x = 10 * get_system_core_clock_hz() / 8 / baudrate;
      uint32_t div = (x + 5) / 10;

      /* 3. 写入 UART 寄存器组并使能 FIFO */
      R8_UART0_LCR = RB_LCR_DLAB; /* 开启 Latch 访问 */
      R16_UART0_DL = (uint16_t)div;
      R8_UART0_DIV = 1;
      R8_UART0_LCR = RB_LCR_WORD_SZ_8; /* 8N1，退出 DLAB */

      R8_UART0_FCR = 0x07; /* 清空并使能 FIFO */
      R8_UART0_IER = 0;    /* 轮询模式 */
  }

  /* 外设初始化总入口 */
  void peripheral_Init(void)
  {
      /* 1. 开启 ROM/RAM 时钟及 UART0 时钟 */
      R32_SLP_POWER_CFG |= RB_SLP_ROM_CODE | RB_SLP_RAM16K | RB_SLP_CLK_UART0;

      /* 2. 板载 GPIO LED (PA8) 初始化 */
      R32_PA_DIR |= (1 << 8);
      R32_PA_OUT |= (1 << 8); /* 默认灭 (低电平亮) */

      /* 3. 串口 0 初始化为 115200 */
      Uart0_Init(115200);
  }

  void peripheral_Clock(void) {}
  ```

- [x] **Step 3: 创建全局外设池静态对象 `port_mdi.c`**
  写入以下对接 `HW` 对象池的代码：
  
  ```c
  #include "mdi_hw.h"
  #include "CH592SFR.h"
  #include <stdint.h>
  #include <stdbool.h>

  /* GPIO LED (PA8) 对象驱动 */
  static int32_t ch592_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
  {
      if (eLevel == MDI_GPIO_HIGH) {
          R32_PA_OUT |= (1 << 8); /* 灭 */
      } else {
          R32_PA_OUT &= ~(1 << 8); /* 亮 */
      }
      return 0;
  }

  static int32_t ch592_gpio_Get(void *pPriv)
  {
      return (R32_PA_PIN & (1 << 8)) ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
  }

  static int32_t ch592_gpio_Toggle(void *pPriv)
  {
      R32_PA_OUT ^= (1 << 8);
      return 0;
  }

  static mdi_gpio_t s_tLedGpio = {
      .pPriv    = NULL,
      .fnSet    = ch592_gpio_Set,
      .fnGet    = ch592_gpio_Get,
      .fnToggle = ch592_gpio_Toggle,
  };

  /* UART0 流传输对象驱动 */
  static int32_t ch592_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
  {
      if (!pchData) return 0;
      for (uint32_t i = 0; i < wLen; i++) {
          while (!(R8_UART0_LSR & RB_LSR_TX_FIFO_EMP));
          R8_UART0_THR = pchData[i];
      }
      return wLen;
  }

  static int32_t ch592_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
  {
      if (!pchBuf) return 0;
      uint32_t i = 0;
      while (i < wLen) {
          if (R8_UART0_LSR & RB_LSR_DR) {
              pchBuf[i++] = (uint8_t)R8_UART0_RBR;
          } else {
              break;
          }
      }
      return i;
  }

  static int32_t ch592_stream_IsBusy(void *pPriv)
  {
      return (R8_UART0_LSR & RB_LSR_TX_ALL_EMP) ? 0 : 1;
  }

  static mdi_stream_t s_tStreamSerial = {
      .pPriv    = NULL,
      .fnWrite  = ch592_stream_Write,
      .fnRead   = ch592_stream_Read,
      .fnIsBusy = ch592_stream_IsBusy,
  };

  /* 静态注册 */
  const mdi_hardware_t HW = {
      .ptLedStatus   = &s_tLedGpio,
      .ptSerial      = &s_tStreamSerial,
  };
  ```

- [x] **Step 4: 修改 `target.mk` 实现无污染 SDK 包含**
  修改 `e:/Project/modus_template/target/ch592/target.mk`：
  1. `CHIPLIB_ROOT` 归口指向官方子模块 SDK 的核心源路径：
     `CHIPLIB_ROOT = vendor/riscv/ch592/EVT/EXAM/SRC`
  2. `STARTUP_S` 直接重定向至官方子模块汇编：
     `STARTUP_S = $(CHIPLIB_ROOT)/Startup/startup_CH592.S`
  3. `SYSTEM_C` 移除，`CHIP_SOURCES` 指定为 `peripheral/ch592/port_sys.c`。
  4. 头文件包含加入：
     `TARGET_INCLUDES = -I$(CHIPLIB_ROOT)/StdPeriphDriver/inc -I$(CHIPLIB_ROOT)/RVMSIS -Itarget/ch592 -I$(SYS_INC_PATH)`

---

### Task 4: CH592 强符号异常拦截与编译链接全线跑通

**Files:**
- Create: `e:/Project/modus_template/target/ch592/ch592_exception.S`
- Modify: `e:/Project/modus_template/target/ch592/target.mk`

- [ ] **Step 1: 创建高性能强符号 HardFault 异常拦截汇编**
  在 `e:/Project/modus_template/target/ch592/ch592_exception.S` 中写入以下纯寄存器备份拦截汇编，完美覆盖官方 `.weak` 中断入口：
  
  ```assembly
  .section .text, "ax"
  .global HardFault_Handler
  .align 2
  HardFault_Handler:
      /* 1. 自动在栈上分配 124 字节以存储通用寄存器快照 x1-x31 */
      addi sp, sp, -124
      sw x1, 0(sp)
      sw x3, 4(sp)
      sw x4, 8(sp)
      sw x5, 12(sp)
      sw x6, 16(sp)
      sw x7, 20(sp)
      sw x8, 24(sp)
      sw x9, 28(sp)
      sw x10, 32(sp)
      sw x11, 36(sp)
      sw x12, 40(sp)
      sw x13, 44(sp)
      sw x14, 48(sp)
      sw x15, 52(sp)
      sw x16, 56(sp)
      sw x17, 60(sp)
      sw x18, 64(sp)
      sw x19, 68(sp)
      sw x20, 72(sp)
      sw x21, 76(sp)
      sw x22, 80(sp)
      sw x23, 84(sp)
      sw x24, 88(sp)
      sw x25, 92(sp)
      sw x26, 96(sp)
      sw x27, 100(sp)
      sw x28, 104(sp)
      sw x29, 108(sp)
      sw x30, 112(sp)
      sw x31, 116(sp)

      /* 2. 原子提取 RISC-V 异常状态寄存器 (CSR) */
      csrr t0, mcause
      csrr t1, mepc
      csrr t2, mtval

      /* 3. 传递核心现场参数 (a0=sp, a1=mepc, a2=mcause, a3=mtval) 并调用诊断 */
      mv a0, sp
      mv a1, t1
      mv a2, t0
      mv a3, t2
      call mdebug_riscv_DumpException

  _fault_hang:
      j _fault_hang
  ```

- [ ] **Step 2: 修改 `target.mk` 链接该拦截文件**
  在 `target/ch592/target.mk` 中，将 `ch592_exception.S` 加入 `ASM_SOURCES` 链接源：
  ```make
  STARTUP_S += target/ch592/ch592_exception.S
  ```

- [ ] **Step 3: 运行一键极速编译验证**
  在 `e:/Project/modus_template` 目录下运行：
  `.\make.bat target=ch592 clean all`
  验证: 完美编译，且无任何宏错误或重复符号错误，生成 `build/template.elf`、`build/template.bin` 和 `build/template.hex`。

- [ ] **Step 4: Commit**
  ```bash
  git add target/ch592/
  git commit -m "feat(ch592): intercept and dump hardfaults using strong symbol HardFault_Handler in target.mk"
  ```
