# Universal FOC Multi-Motor Phase 2 Implementation Plan

**Goal:** Make one `motor_handle_t` a complete, independently bindable motor
object with instance-scoped PWM, ADC, runtime, and fault state.

**Architecture:** Hardware interfaces are immutable operation tables copied into
each motor config. Every callback receives its own opaque context. The motor
layer owns validation and safe invocation; chip adapters only translate between
MDI devices and the hardware-neutral FOC interface. No global HAL registry or
heap allocation is used.

**Repository constraint:** Do not stage, commit, create branches, or push.

## Task 1: Prove instance isolation

- Add a host test with two fake hardware contexts.
- Interleave duty, enable, current reconstruction, reset, and emergency stop.
- Assert that counters, duties, currents, and fault state remain independent.

## Task 2: Introduce context-aware HAL interfaces

- Add `pContext` to PWM and ADC interfaces.
- Return explicit `foc_result_t` status for checked HAL operations.
- Remove global register/get state from `foc_hal.c`.
- Keep all wrappers null-safe and fail closed on emergency stop.

## Task 3: Make `motor_handle_t` the aggregation root

- Bind hardware through `motor_config_t` during `motor_Init()`.
- Add motor-level calibrate, sample, duty, enable, stop, and fault APIs.
- Preserve only per-instance mutable state in `motor_handle_t`.
- Make sensor and observer interfaces context-aware in preparation for Phase 3.

## Task 4: Migrate adapters and application

- Convert AT32F413 and STM32G431 MDI adapters to context callbacks.
- Export a function that fills one motor hardware binding instead of registering
  global operations.
- Route application and phase tests through motor APIs.
- Move coroutine/debug mutable state into `foc_app_t`.

## Task 5: Verify

- Run host float and fixed tests.
- Build AT32F413 float and fixed configurations.
- Compile fixed core for Cortex-M0 and inspect helper references.
- Scan `foc/` for vendor/architecture headers and run `git diff --check`.
