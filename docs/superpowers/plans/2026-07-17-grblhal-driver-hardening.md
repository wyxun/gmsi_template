# grblHAL Driver Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the AT32F407 grblHAL driver safe and contract-correct for step generation, USB CDC/UART streaming, single-direction spindle control, lifecycle, and fatal shutdown.

**Architecture:** Keep hardware access in the AT32F407 port and grblHAL contracts in the adapt layer. Extract small host-testable helpers for timing and ring/stream decisions, then bind them to TMR5, DWT, USB CDC, UART DMA, and GPIO implementations.

**Tech Stack:** C11 bare-metal ARM Cortex-M4F, AT32 standard peripheral library, grblHAL HAL v10, PowerShell/GNU Make.

---

### Task 1: Regression contract tests

**Files:**
- Create: `tests/grblhal_driver_contracts.Tests.ps1`

- [ ] Write failing source-contract and helper tests for separate UART DMA/queue arrays, USB/UART cancel-to-CAN, RX count, complete TX retry, 32-bit TMR5 setup, CPU-clock timing conversion, settings refresh, HAL version check, idempotent setup, spindle registration, and fatal shutdown.
- [ ] Run `.\tests\grblhal_driver_contracts.Tests.ps1` and verify failures identify the missing contracts.
- [ ] Bind pulse and direction delays to the existing perf_counter microsecond API only after the failing timing tests exist.
- [ ] Re-run the tests and retain failures for production bindings not implemented yet.

### Task 2: UART DMA transport

**Files:**
- Modify: `peripheral/at32f407/port_mdi.c`
- Modify: `peripheral/at32f407/port_mdi.h`
- Modify: `target/at32f407/at32f407_it.c`

- [ ] Split DMA staging and `mringbuf` RX storage.
- [ ] Protect RX extraction against main, USART, and DMA re-entry with an IRQ-state-preserving critical section and overflow counter.
- [ ] Export poll, count, free, reset, cancel injection, and TX-all primitives needed by the stream adapter.
- [ ] Make TX-all retry queue-full conditions through a caller callback and abort cleanly when requested.
- [ ] Run the regression tests and verify UART contract cases pass.

### Task 3: USB CDC and stream contracts

**Files:**
- Modify: `grblhal_adapt/grblhal_stream.c`
- Modify: `peripheral/at32f407/port_usb.c`
- Modify: `peripheral/at32f407/port_mdi.h`

- [ ] Add USB RX overflow accounting and poll endpoint packets without consuming normal characters in count/free operations.
- [ ] Make USB TX retry busy endpoints through `hal.stream_blocking_callback`, stopping on callback abort or disconnect.
- [ ] Report USB configured state via the vendor core.
- [ ] Make USB and UART cancel flush normal input and deliver ASCII CAN to the active realtime handler.
- [ ] Set `suspend_read` to NULL until save/restore semantics exist.
- [ ] Return real RX count/free for both transports.
- [ ] Run the regression tests and verify all stream cases pass.

### Task 4: Stepper timing and settings

**Files:**
- Modify: `grblhal_adapt/grblhal_stubs.c`
- Modify: `grblhal_adapt/grblhal_driver.h`

- [ ] Configure TMR5 with `0xFFFFFFFF` and preserve full `uint32_t` periods.
- [ ] Cache pulse width and direction delay as DWT cycles using `SystemCoreClock`.
- [ ] Apply settings step/dir/enable inversion and refresh caches from `settings_changed`.
- [ ] Apply direction setup delay only after a direction change, then generate the configured pulse width.
- [ ] Clear STEP/DIR on requested idle and implement the shared active-low enable without claiming per-axis support.
- [ ] Run tests, then build with `mingw32-make TARGET_CHIP=at32f407 BUILD=debug-rel`.

### Task 5: Lifecycle, spindle, and safe halt

**Files:**
- Modify: `grblhal_adapt/grblhal_stubs.c`
- Modify: `grblhal_adapt/grblhal_driver.h`
- Modify: `peripheral/at32f407/port_sys.c`
- Modify: `vendor/cortex-m/core_debug/core_debug_cm_fault.c` only if a weak application safe-stop hook already exists or can be added without replacing fault diagnostics.

- [ ] Reject `hal.version != HAL_VERSION` in `driver_init`.
- [ ] Make setup and callback/plugin hook installation idempotent.
- [ ] Register an on/off spindle using PA6; keep PA5 inactive and advertise neither direction nor PWM.
- [ ] Add one emergency shutdown helper that stops TMR5, disables steppers, clears STEP pins, and disables spindle.
- [ ] Call safe shutdown from `_exit`, `__assert_func`, and the existing fault path through a weak hook where feasible.
- [ ] Keep provisional limits/control/coolant/probe capabilities consistent with the approved scope.
- [ ] Run regression tests and the full AT32F407 build.

### Task 6: Known issues and verification

**Files:**
- Create: `docs/grblhal_driver_known_issues.md`
- Modify: `docs/cnc_pin_mapping.md` only if spindle behavior needs clarification.

- [ ] Record true stream suspension, blocking delay design, and transport connection semantics as follow-ups.
- [ ] Run `.\tests\grblhal_driver_contracts.Tests.ps1` and confirm zero failures.
- [ ] Run `mingw32-make TARGET_CHIP=at32f407 BUILD=debug-rel` and confirm exit code zero.
- [ ] Run `mingw32-make TARGET_CHIP=at32f407 BUILD=debug-rel size` and record section sizes.
- [ ] Run `git diff --check` and review the final diff against every approved item.
