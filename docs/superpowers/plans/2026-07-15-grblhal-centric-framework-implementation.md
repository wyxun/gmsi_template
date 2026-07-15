# grblHAL Centric Framework Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the codebase to make grblHAL the absolute core running the main execution loop directly from `main()`, completely decoupling MODUS and its classes, allowing a zero-overhead Release build where MODUS is entirely compiled out.

**Architecture:** 
1. Remove `class/grblhal.c` and relocate elapsed ticks management directly to `grblhal_stubs.c`.
2. Move MODUS execution to grblHAL's `on_execute_realtime` hook in Debug mode.
3. Establish a standard `app_plugins.c` file to handle dynamic plugin registration.
4. Modify `src/main.c`, `at32f407_it.c`, and the root `makefile` to support conditional compilation via the `MODUS_ENABLE` flag.

**Tech Stack:** C (GNU11), make, GCC/LLVM toolchain for ARM.

---

### Task 1: Relocate Elapsed Ticks Management to grblhal_stubs.c

**Files:**
- Modify: `grblhal_adapt/grblhal_stubs.c`
- Modify: `grblhal_adapt/grblhal_driver.h`

- [ ] **Step 1: Declare grblhal_ticks_inc in driver header**
  Add the declaration of `grblhal_ticks_inc` to `grblhal_adapt/grblhal_driver.h`.
  Replace lines 32-34 in `grblhal_adapt/grblhal_driver.h` with:
  ```c
  /* get_elapsed_ticks — driven by 1ms SysTick */
  uint32_t grblhal_get_ticks(void);
  void grblhal_ticks_inc(void);
  ```

- [ ] **Step 2: Implement tick accumulator variables and functions in grblhal_stubs.c**
  Define `s_wGrblhalTicks`, `grblhal_get_ticks`, and `grblhal_ticks_inc` in `grblhal_adapt/grblhal_stubs.c`.
  Add this block near the top (after includes) of `grblhal_adapt/grblhal_stubs.c`:
  ```c
  static volatile uint32_t s_wGrblhalTicks = 0;

  uint32_t grblhal_get_ticks(void)
  {
      return s_wGrblhalTicks;
  }

  void grblhal_ticks_inc(void)
  {
      s_wGrblhalTicks++;
  }
  ```

- [ ] **Step 3: Verify Compilation**
  Run: `.\make.bat BUILD=debug`
  Expected: Success, object files compiled (link might still report warning/conflict for grblhal_get_ticks in class/grblhal.c).

- [ ] **Step 4: Commit**
  Run:
  ```bash
  git add grblhal_adapt/grblhal_stubs.c grblhal_adapt/grblhal_driver.h
  git commit -m "refactor: relocate grblhal elapsed ticks tracker to grblhal_stubs.c"
  ```

---

### Task 2: Remove MODUS grblhal.c Class Wrapper

**Files:**
- Delete: `class/grblhal.c`
- Delete: `class/grblhal.h`
- Modify: `target/at32f407/target.mk`

- [ ] **Step 1: Remove grblhal class from target.mk**
  Open `target/at32f407/target.mk` and delete the following lines:
  ```makefile
  # grblHAL class source
  CLASS_SOURCES += class/grblhal.c
  ```

- [ ] **Step 2: Delete grblhal class source files**
  Delete the files `class/grblhal.c` and `class/grblhal.h` in the workspace.
  Command:
  ```powershell
  Remove-Item class/grblhal.c, class/grblhal.h -ErrorAction SilentlyContinue
  ```

- [ ] **Step 3: Commit**
  Run:
  ```bash
  git add target/at32f407/target.mk
  git rm class/grblhal.c class/grblhal.h
  git commit -m "refactor: remove MODUS class/grblhal.c wrapper completely"
  ```

---

### Task 3: Refactor SysTick Interrupt Handler

**Files:**
- Modify: `target/at32f407/at32f407_it.c`

- [ ] **Step 1: Include MDI headers and refactor SysTick_Handler**
  Open `target/at32f407/at32f407_it.c`. Add includes for `mdi.h` and `mdi_hw.h` at the top. Modify `SysTick_Handler` to wrap `modus_Clock()` in `#if MODUS_ENABLE`, call `grblhal_ticks_inc()`, and toggle the LED via `MDI_Toggle(HW.ptLedStatus)`.
  Replace:
  ```c
  #define CORE_DEBUG_OVERRIDE_FAULT_HANDLER
  #include "at32f403a_407.h"
  #include "at32f403a_407_dma.h"
  #include "perf_counter.h"
  #include "mdebug_cm.h"

  /* Exported by main.c */
  extern void modus_Clock(void);
  extern void peripheral_Clock(void);
  ```
  With:
  ```c
  #define CORE_DEBUG_OVERRIDE_FAULT_HANDLER
  #include "at32f403a_407.h"
  #include "at32f403a_407_dma.h"
  #include "perf_counter.h"
  #include "mdebug_cm.h"
  #include "mdi.h"
  #include "mdi_hw.h"

  /* Exported by main.c */
  extern void modus_Clock(void);
  extern void peripheral_Clock(void);
  ```
  And also replace:
  ```c
  void SysTick_Handler(void)
  {
      static uint32_t s_wLedTicks = 0;
      perfc_port_insert_to_system_timer_insert_ovf_handler();
      modus_Clock();
      at32_usart_timer_1ms(&s_tUsart1Priv);

      if (++s_wLedTicks >= 500) {
          s_wLedTicks = 0;
          gpio_bits_toggle(GPIOD, GPIO_PINS_13);
      }
  }
  ```
  With:
  ```c
  void SysTick_Handler(void)
  {
      static uint32_t s_wLedTicks = 0;
      perfc_port_insert_to_system_timer_insert_ovf_handler();
  #if MODUS_ENABLE
      modus_Clock();
  #endif
      extern void grblhal_ticks_inc(void);
      grblhal_ticks_inc();
      at32_usart_timer_1ms(&s_tUsart1Priv);

      if (++s_wLedTicks >= 500) {
          s_wLedTicks = 0;
          MDI_Toggle(HW.ptLedStatus);
      }
  }
  ```

- [ ] **Step 2: Verify compile**
  Run: `.\make.bat BUILD=debug`
  Expected: Successful compilation of `at32f407_it.c`. (Link may still report missing symbols which is resolved in subsequent tasks).

- [ ] **Step 3: Commit**
  Run:
  ```bash
  git add target/at32f407/at32f407_it.c
  git commit -m "refactor: update SysTick_Handler to use MDI_Toggle for status LED"
  ```

---

### Task 4: Hook MODUS Execution to grblHAL in Debug Mode

**Files:**
- Modify: `grblhal_adapt/grblhal_stubs.c`

- [ ] **Step 1: Implement Debug Realtime Hook in grblhal_stubs.c**
  At the end of `grblhal_adapt/grblhal_stubs.c`, add the hook function and chain it inside `grblhal_driver_setup`.
  Add this block at the end of the helper functions section in `grblhal_adapt/grblhal_stubs.c` (e.g. before `grblhal_driver_setup`):
  ```c
  #if MODUS_ENABLE
  #include "modus.h"
  static on_execute_realtime_ptr s_fnPrevExecuteRealtime = NULL;

  static void debug_modus_realtime_hook(sys_state_t state)
  {
      if (s_fnPrevExecuteRealtime) {
          s_fnPrevExecuteRealtime(state);
      }
      modus_Run();
  }
  #endif
  ```

- [ ] **Step 2: Chain Hook in grblhal_driver_setup**
  In `grblhal_driver_setup` in `grblhal_adapt/grblhal_stubs.c`, hook the function pointer.
  Add right before `return true;` in `grblhal_driver_setup`:
  ```c
  #if MODUS_ENABLE
      s_fnPrevExecuteRealtime = grbl.on_execute_realtime;
      grbl.on_execute_realtime = debug_modus_realtime_hook;
  #endif
  ```

- [ ] **Step 3: Commit**
  Run:
  ```bash
  git add grblhal_adapt/grblhal_stubs.c
  git commit -m "refactor: hook MODUS scheduler to grblHAL realtime handler in Debug mode"
  ```

---

### Task 5: Implement app_plugins.c Dispatcher

**Files:**
- Create: `grblhal_adapt/app_plugins.c`

- [ ] **Step 1: Write app_plugins.c**
  Create a new file `grblhal_adapt/app_plugins.c` containing the overridden `my_plugin_init` function.
  Code content:
  ```c
  /**
   * @file   app_plugins.c
   * @brief  Project Custom Plugins Dispatcher for grblHAL
   */

  #include "grbl.h"
  #include "hal.h"
  #include "task.h"
  #include "report.h"

  // 1. Declare custom user plugins init functions here:
  // extern void custom_safety_init(void);

  /**
   * @brief Override grblHAL weak function my_plugin_init()
   *        This is called by the core grblHAL library after hardware is set up,
   *        but before entering the main loop.
   */
  void my_plugin_init(void)
  {
      // 2. Call custom plugin initializers:
      // custom_safety_init();
      
      // 3. Report loaded plugins:
      // report_plugin("APP_PLUGINS", "1.0");
  }
  ```

- [ ] **Step 2: Commit**
  Run:
  ```bash
  git add grblhal_adapt/app_plugins.c
  git commit -m "feat: add app_plugins.c dispatcher file for decoupled user logic"
  ```

---

### Task 6: Refactor Main Entry Point

**Files:**
- Modify: `src/main.c`

- [ ] **Step 1: Clean and redirect main loop to grbl_enter()**
  Rewrite `src/main.c` to wrap MODUS in `#if MODUS_ENABLE` and handover execution directly to `grbl_enter()`.
  Replace the entire content of `src/main.c` with:
  ```c
  /*============================ INCLUDES ======================================*/
  #include "global_define.h"
  #include <stdio.h>
  #include "peripheral.h"
  #include "mdi_hw.h"
  #include "perf_counter.h"
  #include "util_debug.h"
  #include "debug_transport.h"

  #if MODUS_ENABLE
  #include "modus.h"
  static modus_t s_tModus = { .ptAppFlash = NULL };
  #endif

  #if MODUS_ENABLE && (MSHELL_ENABLE || !defined(__NO_USE_LOG__))
  #include <string.h>
  void user_trace_output(const char *str)
  {
      debug_transport_write_string(str);

      /* Output to Serial too */
      if (str && HW.ptSerial) {
          MDI_Write(HW.ptSerial, (const uint8_t *)str, strlen(str));
      }
  }
  #endif

  int main(void)
  {
      peripheral_Init();
      perfc_init(true);

  #if MODUS_ENABLE
      #if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
          debug_transport_init();
      #endif
      modus_Init(&s_tModus);
  #endif

      /* Direct grblHAL handover (contains its own infinite blocking loop) */
      extern int grbl_enter(void);
      grbl_enter();

      return 0;
  }
  ```

- [ ] **Step 2: Commit**
  Run:
  ```bash
  git add src/main.c
  git commit -m "refactor: boot directly into grblHAL main loop from src/main.c"
  ```

---

### Task 7: Conditionalize MODUS in Makefile

**Files:**
- Modify: `makefile`

- [ ] **Step 1: Define MODUS_ENABLE variable and conditional include logic**
  Open `makefile`, add `MODUS_ENABLE ?= 1` and wrap the include of `modus.mk` in a conditional block. Also ensure `CLASS_SOURCES` is only appended to `C_SOURCES` when enabled.
  Replace:
  ```makefile
  # Build mode: BUILD=debug (default) | BUILD=debug-rel | BUILD=release
  BUILD ?= debug

  # Module switches — default for debug; overridden by release below
  MSHELL_ENABLE    = 1
  MWAVEFORM_ENABLE = 1
  MSTORAGE_ENABLE  = 1
  MBLINFO_ENABLE   = 1
  MODUS_USE_LOG    = 1

  # Build mode overrides (MUST be before modus.mk include)
  ifeq ($(BUILD),release)
      # Production: no debug modules
      OPT = -Os
      MSHELL_ENABLE    = 0
      MWAVEFORM_ENABLE = 0
      MSTORAGE_ENABLE  = 0
      MBLINFO_ENABLE   = 0
      MODUS_USE_LOG    = 0
  else ifeq ($(BUILD),debug-rel)
  ```
  With:
  ```makefile
  # Build mode: BUILD=debug (default) | BUILD=debug-rel | BUILD=release
  BUILD ?= debug

  MODUS_ENABLE ?= 1

  # Module switches — default for debug; overridden by release below
  MSHELL_ENABLE    = 1
  MWAVEFORM_ENABLE = 1
  MSTORAGE_ENABLE  = 1
  MBLINFO_ENABLE   = 1
  MODUS_USE_LOG    = 1

  # Build mode overrides (MUST be before modus.mk include)
  ifeq ($(BUILD),release)
      # Production: no debug modules, completely compiled-out MODUS
      OPT = -Os
      MODUS_ENABLE = 0
      MSHELL_ENABLE    = 0
      MWAVEFORM_ENABLE = 0
      MSTORAGE_ENABLE  = 0
      MBLINFO_ENABLE   = 0
      MODUS_USE_LOG    = 0
  else ifeq ($(BUILD),debug-rel)
  ```

- [ ] **Step 2: Update include of modus.mk with fallback defines**
  Replace:
  ```makefile
  include $(MODUS_ROOT)/modus.mk
  ```
  With:
  ```makefile
  ifeq ($(MODUS_ENABLE),1)
      include $(MODUS_ROOT)/modus.mk
      C_DEFS += -DMODUS_ENABLE=1
  else
      C_DEFS += -DMODUS_ENABLE=0
      MODUS_INCLUDES = \
          -I$(MODUS_ROOT) \
          -I$(MODUS_ROOT)/src \
          -I$(MODUS_ROOT)/src/mdi \
          -I$(MODUS_ROOT)/src/arch \
          -I$(MODUS_ROOT)/src/arch/cortex-m \
          -I$(MODUS_ROOT)/src/arch/riscv \
          -I$(LIB_PLOOC_DIR) \
          -I$(LIB_PERF_DIR)
      MODUS_CFLAGS = -DMSHELL_ENABLE=0 -DMWAVEFORM_ENABLE=0
      MODUS_SRCS = $(MODUS_ROOT)/src/arch/perfc_port.c
  endif
  ```

- [ ] **Step 3: Conditionalize CLASS_SOURCES in C_SOURCES assembly**
  Replace lines 119-132 in `makefile`:
  ```makefile
  # ------------------------------------------------------------------------------
  # All C sources  (chip-specific vars set by target.mk)
  # ------------------------------------------------------------------------------
  C_SOURCES = \
      src/main.c \
      $(PERFC_PORT_C) \
      $(IT_C) \
      $(SYSTEM_C) \
      $(CHIP_SOURCES) \
      $(MODUS_SRCS) \
      $(PERIF_LIB_SOURCES) \
      $(PERIPHERAL_SOURCES) \
      $(CLASS_SOURCES) \
      $(FOC_SOURCES)
  ```
  With:
  ```makefile
  # ------------------------------------------------------------------------------
  # All C sources  (chip-specific vars set by target.mk)
  # ------------------------------------------------------------------------------
  C_SOURCES = \
      src/main.c \
      $(PERFC_PORT_C) \
      $(IT_C) \
      $(SYSTEM_C) \
      $(CHIP_SOURCES) \
      $(MODUS_SRCS) \
      $(PERIF_LIB_SOURCES) \
      $(PERIPHERAL_SOURCES) \
      $(FOC_SOURCES)

  ifeq ($(MODUS_ENABLE),1)
      C_SOURCES += $(CLASS_SOURCES)
  endif
  ```

- [ ] **Step 4: Verify Debug and Release Builds**
  Clean and build both:
  Run: `.\make.bat clean; .\make.bat BUILD=debug`
  Expected: Clean build success for Debug.
  Run: `.\make.bat clean; .\make.bat BUILD=release`
  Expected: Clean build success for Release (size should be significantly smaller, e.g. ~160KB instead of ~250KB).

- [ ] **Step 5: Commit**
  Run:
  ```bash
  git add makefile
  git commit -m "build: make MODUS compilation conditional under MODUS_ENABLE flag"
  ```
