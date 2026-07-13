# AT32F407 grblHAL — Design Spec

## Overview

Add AT32F407 target support using the AT-START-F407 eval board, with full
grblHAL integration over physical UART (USART1).  No flash-driven feature
trimming; stub all hardware drivers initially.

**Guiding principle**: zero regression on existing targets (STM32G431, AT32F413,
AT32F421, CH592).  Shared files use compile-time feature gates; each target's
unique code lives under its own directory.

---

## 1. New Submodule

```
vendor/cortex-m/AT32F403A_407_Firmware_Library
  url  = https://github.com/ArteryTek/AT32F403A_407_Firmware_Library.git
  path = vendor/cortex-m/AT32F403A_407_Firmware_Library
```

AT32F403A and AT32F407 share a single firmware library.  The library provides
standard-peripheral-driver source under `libraries/drivers/src/` and headers
under `libraries/drivers/inc/`, exactly like the existing AT32F413 and AT32F421
submodules.

`.gitmodules` gets a new `[submodule]` block; existing entries are untouched.

---

## 2. target/at32f407/

Five files, modelled on `target/at32f413/`.

### 2.1 `target.mk`

```makefile
GRBLHAL_ENABLE = 1
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
C_DEFS += -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER
C_DEFS += -DGRBLHAL_FULL_FEATURES=1 -DGRBLHAL_STREAM_UART=1

CHIPLIB_ROOT = vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

LDSCRIPT   = target/at32f407/AT32F407xC_FLASH.ld
STARTUP_S  = $(CMSIS_DEV)/startup/gcc/startup_at32f403a_407.s
SYSTEM_C   = $(CMSIS_DEV)/system_at32f403a_407.c
IT_C       = target/at32f407/at32f407_it.c

DRV_SOURCES = (CRM, GPIO, MISC, FLASH, USART, DEBUG — same pattern as at32f413)
CHIP_SOURCES = $(DRV_SOURCES)

CLASS_SOURCES += class/grblhal.c
MODUS_USE_DEFAULT_PERFC_PORT = 1
```

The two new macros (`GRBLHAL_FULL_FEATURES`, `GRBLHAL_STREAM_UART`) are defined
**only** in this target.mk.  Other targets never see them.

### 2.2 `at32f407_conf.h`

Mirrors `at32f413_conf.h` — enables CRM, GPIO, USART, MISC, FLASH, DEBUG.
Includes the library header `at32f403a_407.h`.  All other peripherals off.

### 2.3 `AT32F407xC_FLASH.ld`

Flash origin 0x08000000, length 1024K.  SRAM origin 0x20000000, length 96K.
Stack top 0x20018000.

### 2.4 `at32f407_it.c`

SysTick_Handler: calls `perfc_port_insert_to_system_timer_insert_ovf_handler()`,
`modus_Clock()`, and the USART 1-ms timer.

USART1_IRQHandler: calls the ringbuf interrupt handler (name TBD by port_mdi.h).

All other IRQ handlers are empty stubs (same style as at32f413).

### 2.5 `openocd.cfg`

AT32F407 debug probe configuration (MCU-specific ID and flash algorithm).

---

## 3. peripheral/at32f407/

Four files, modelled on `peripheral/at32f413/`.

### 3.1 `mdi_hw.h` — Hardware resource pool

```c
typedef struct {
    mdi_gpio_t   *ptLedStatus;   // PD13, active-low (AT-START-F407 on-board LED)
    mdi_stream_t *ptSerial;      // USART1 PA9/PA10 (on-board VCP)
} mdi_hardware_t;
```

Minimal — only what grblHAL and the status heartbeat need.

### 3.2 `port_mdi.c` — MDI adapters

**GPIO**: `at32_gpio_Set/Get/Toggle` wrappers (identical pattern to at32f413).

**USART1 ringbuf**: copied from `peripheral/at32f413/port_mdi.c` with one
change — USART1 uses PA9/PA10 (no remap needed for AT-START-F407).

**MDI stream**: `at32_stream_Write/Read/IsBusy` implementations using the
ringbuf, same pattern as at32f413.

**`HW` global instance**: populates `ptLedStatus` and `ptSerial`.

### 3.3 `port_mdi.h`

Exposes `at32_usart_priv_t` struct and declares:
- `at32_usart_init()`
- `at32_usart_irq_handler()`
- `at32_usart_timer_1ms()`
- `at32_usart1_init()`

### 3.4 `port_sys.c`

System clock: 8 MHz HSE → PLL → 240 MHz (AT32F407 max).
- HCLK = 240 MHz, APB1 = 120 MHz, APB2 = 120 MHz
- SysTick = HCLK / 1000 = 240000 ticks/ms

Calls `peripheral_Init()`, `peripheral_Clock()`, `get_system_core_clock_hz()`,
`peripheral_EnableIRQ()` / `peripheral_DisableIRQ()`.

---

## 4. Modified shared files (feature-gated)

### 4.1 `grblhal_adapt/grblhal_stream.c`

Two paths behind a compile-time switch:

```c
#ifdef GRBLHAL_STREAM_UART
  // AT32F407 path: use MDI UART
  static int32_t uart_read(void) {
      uint8_t ch;
      int32_t n = MDI_Read(HW.ptSerial, &ch, 1);
      return (n > 0) ? (int32_t)ch : -1;
  }
  // ... write_n via MDI_Write, etc.
#else
  // Existing STM32G431 path: SEGGER_RTT (unchanged)
#endif
```

The `io_stream_t` struct is filled with the platform-appropriate callbacks.
`GRBLHAL_STREAM_UART` is only defined in `target/at32f407/target.mk`.

### 4.2 `grblhal_adapt/grblhal_config.h`

Two paths behind a compile-time switch:

```c
#ifdef GRBLHAL_FULL_FEATURES
  // AT32F407: full features, no trimming
  #define SPINDLE_ENABLE      1
  #define COOLANT_ENABLE      1
  #define PROBE_ENABLE        1
  #define LIMITS_ENABLE       1
  // ... all other feature toggles = 1
  // NO trimming macros defined
#else
  // STM32G431: trimmed for 128K flash (unchanged)
#endif
```

`GRBLHAL_FULL_FEATURES` is only defined in `target/at32f407/target.mk`.

---

## 5. Files NOT changed

| File | Reason |
|---|---|
| `src/main.c` | Already generic — no target-specific code |
| `src/userconfig.h` | Already generic |
| `class/grblhal.c` | Already generic MODUS Object wrapper |
| `class/grblhal.h` | Already generic public API |
| `grblhal_adapt/grblhal_stubs.c` | Stubs apply to ALL targets; no change needed |
| `grblhal_adapt/grblhal_driver.h` | Interface is unchanged |
| `Makefile` | Already generic — `CLASS_SOURCES += class/grblhal.c` when `GRBLHAL_ENABLE=1` |
| All files under `target/stm32g431/`, `target/at32f413/`, `target/at32f421/`, `target/ch592/` | Untouched |
| All files under `peripheral/stm32g431/`, `peripheral/at32f413/`, `peripheral/at32f421/`, `peripheral/ch592/` | Untouched |
| grblHAL core source (`third_party/grblhal/core/`) | No patches needed (no flash constraint) |

---

## 6. Build & Verify

```bash
# AT32F407
mingw32-make TARGET_CHIP=at32f407 clean
mingw32-make TARGET_CHIP=at32f407

# Confirm existing targets still compile
mingw32-make TARGET_CHIP=stm32g431 clean && mingw32-make TARGET_CHIP=stm32g431
mingw32-make TARGET_CHIP=at32f413  clean && mingw32-make TARGET_CHIP=at32f413
mingw32-make TARGET_CHIP=at32f421  clean && mingw32-make TARGET_CHIP=at32f421

# Size check — should be well under 1024K
mingw32-make TARGET_CHIP=at32f407 size
```

---

## 7. Runtime Behaviour

1. MCU boots → `peripheral_Init()` configures 240 MHz clock, GPIO, USART1
2. `modus_Init()` auto-discovers and calls `grblhal_Init()`
3. `driver_init()` assembles the `hal` struct (all hardware stubs + UART stream)
4. `grbl_enter()` completes grblHAL boot sequence
5. Banner prints over UART (not RTT)
6. Main loop: `protocol_execute_realtime()` + `protocol_main_loop()` read G-code
   from USART1, process it, send responses back over USART1
7. User connects to AT-START-F407's virtual COM port with any serial terminal
   (115200-8-N-1) and interacts with grblHAL directly

---

## 8. Open Items (out of scope for this spec)

- Real stepper pulse generation (currently stubs)
- Real spindle / coolant / limit switch drivers
- NVS (non-volatile settings) storage via internal flash
- SD card streaming
