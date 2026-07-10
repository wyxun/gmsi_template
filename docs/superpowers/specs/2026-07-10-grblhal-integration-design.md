# grblHAL Integration Design

**Date:** 2026-07-10  
**Status:** Draft  
**Target:** STM32G431xB (Phase 1 — compile + algorithm verification)

## Overview

Integrate [grblHAL/core](https://github.com/grblHAL/core) as a `third_party/` git submodule into the modus_template multi-chip project. Phase 1 verifies compile compatibility and core algorithm execution (G-code parsing, planner, motion control) via RTT shell interaction — no real stepper hardware.

## Key Constraints

- **Zero modification** to grblHAL source files. All glue lives in modus_template.
- **Opt-in per target chip.** Only `target/stm32g431/target.mk` enables it via `GRBLHAL_ENABLE=1`. Other targets (AT32F421, AT32F413, CH592) must compile cleanly with no awareness of grblHAL.
- grblHAL runs as a MODUS Object, polled cooperatively in the main loop. The stepper timer ISR is chip-level, placed in `stm32g4xx_it.c` under `#ifdef GRBLHAL_ENABLE`.

## Directory Layout

```
third_party/grblhal/
  core/                           # git submodule: https://github.com/grblHAL/core
    grblhal.mk                    # NEW — build bridge (lives alongside submodule)

grblhal_adapt/                    # NEW — MODUS ↔ grblHAL glue
  grblhal_driver.c                # driver_init() — assembles hal.* function pointer table
  grblhal_driver.h                # internal declarations
  grblhal_stream.c                # io_stream_t → RTT (SEGGER RTT channel 0)
  grblhal_stubs.c                 # all remaining hal handlers as stubs (return 0/true/empty)
  grblhal_config.h                # compile-time settings: axis count, buffer sizes

class/
  grblhal_class.c                 # NEW — MODUS Object wrapping grblHAL main loop
  grblhal_class.h
```

## Configuration Guard Pattern

Follow the existing `FOC_SUPPORT` pattern. Each target that wants grblHAL sets `GRBLHAL_ENABLE=1` in its `target.mk`. The master Makefile guards all grblHAL includes, sources, and defines with `ifdef GRBLHAL_ENABLE`. Source files use `#ifdef GRBLHAL_ENABLE` guards.

## Build Integration (`grblhal.mk`)

Analogous to `modus.mk`. Defines:

- `GRBLHAL_ROOT` — auto-detected from file location
- `GRBLHAL_SRCS` — aggregated .c source list (core always compiled; optional modules gated by feature switches)
- `GRBLHAL_INCLUDES` — include paths
- `GRBLHAL_CFLAGS` — preprocessor defines

Feature switches with conservative defaults: `GRBLHAL_SPINDLE_SYNC ?= 0`, `GRBLHAL_SD_CARD ?= 0`, `GRBLHAL_CAN ?= 0`, `GRBLHAL_MODBUS ?= 0`.

Master Makefile includes it only when `GRBLHAL_ENABLE` is defined, adding sources, includes, and `-DGRBLHAL_ENABLE=1`.

## Adapt Layer (`grblhal_adapt/`)

### grblhal_stream.c — Real I/O (the only working module)

Maps grblHAL `io_stream_t` to SEGGER RTT channel 0. Both G-code input and status reports flow through the RTT terminal. Users interact by typing G-code directly.

### grblhal_stubs.c — All Remaining Modules

Every other `hal.*` function pointer gets a minimal stub:

- `driver_setup` → returns true
- `delay_ms` → busy-wait via SysTick (from perf_counter)
- `set_bits_atomic` / `clear_bits_atomic` / `set_value_atomic` → bare C assignment (non-ISR context is fine for verification)
- `limits.enable` / `limits.get_state` → return no limits triggered
- `control.get_state` → return default signal states
- `coolant.*` → empty
- `stepper.*` → log step/dir to RTT; `interrupt_callback` fires from real TIM2 ISR
- `spindle.*` → empty
- `probe.*` → return not connected
- `nvs` → NULL (use grblHAL compile-time defaults)
- `tool.*` → empty
- `rtc.*` → return false (no RTC)
- `timer.*` → return NULL / false (no hardware timers claimed)

## Stepper Timer ISR (`stm32g4xx_it.c`)

TIM2 is repurposed as the stepper pulse timer. Under `#ifdef GRBLHAL_ENABLE`:

```c
void TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_UIF;
    hal.stepper.interrupt_callback();  // → grblHAL core computes next step
}
```

Timer is configured in `port_sys.c` (simple periodic up-counter, no actual pin toggles in phase 1). The core callbacks `cycles_per_tick` and `pulse_start` are stubbed to RTT-print their arguments — enough to observe planner output.

## MODUS Class (`grblhal_class`)

The class wraps grblHAL's main loop into the MODUS cooperative schedule:

```c
int grblhal_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    grblhal_class_t *ptThis = (grblhal_class_t *)wObjectAddr;
    (void)wObjectCfgAddr;

    grblhal_driver_init();   // populate hal.* function pointer table
    grbl_Init();             // grblHAL settings load + driver_setup

    return mbase_Init(ptThis->ptBase, &s_tGrblhalBaseCfg);
}

int grblhal_Run(uintptr_t wObjectAddr)
{
    (void)wObjectAddr;
    protocol_execute_realtime();  // feed hold, reset, etc.
    protocol_exec_main();         // G-code parser — one line per call
    return MODUS_SUCCESS;
}
```

Registered via `MODUS_DECLARE_OBJECT(grblhal_class, GrblHAL, .hwRingSize=0)` — no ring buffer needed initially. The `.init_infos` linker section ensures `modus_Init()` calls `grblhal_class_Init()` automatically — no manual invocation in `main.c`.

## Runtime Flow

```
main():
  peripheral_Init()
  debug_transport_init()           # RTT up
  modus_Init(&s_tModus)            # auto-calls grblhal_class_Init → driver_init + grbl_Init

  while(1):
    modus_Run()                    # auto-calls grblhal_Run among all objects
```

No `#ifdef GRBLHAL_ENABLE` in `main.c`. The class is fully self-contained — the compiler only links it when `GRBLHAL_ENABLE=1` in the target's `target.mk`.

Stepper ISR path (independent of MODUS, via NVIC):
  `TIM2_IRQHandler → hal.stepper.interrupt_callback() → core → pulse_start (stub)`

## Open Questions / Future Phases

1. **Toolchain compatibility** — grblHAL may use GCC-specific syntax (`__attribute__`, `asm volatile`). Must verify with LLVM Clang. If issues arise, add `-Wno-*` flags or minor compatibility shims.
2. **Flash/RAM budget** — grblHAL core ≈ 60–80 KB flash. Combined with MODUS + HAL on G431 (128 KB flash, 22 KB RAM) should fit. Actual footprint TBD after first compile.
3. **Phase 2** — Real stepper output. Requires hardware step/dir pin mapping and a proper `mdi_gpio_t` pool for motion signals.
4. **Other MCU targets** — Once verified on G431, F103 and F303 would need their own `target.mk` entries and an appropriate timer for the stepper ISR. The adapt layer is reusable since all stubs are chip-agnostic.
