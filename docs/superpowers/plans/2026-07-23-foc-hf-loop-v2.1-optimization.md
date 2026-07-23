# FOC High Frequency Loop V2.1 Optimization Implementation Plan

> **For agentic workers:** Execute this plan task-by-task. Do not stage, commit,
> switch branches, or push unless the user explicitly requests it. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce high-frequency FOC-loop cost and jitter without changing the
control period, weakening safety behavior, or making the common FOC core depend
on a particular MCU, peripheral, or compiler.

**Architecture:** V2.1 adopts BAM32 as the internal electrical-angle storage,
so wrapping and LUT indexing are integer operations shared by float and fixed
numeric backends. The core remains hardware agnostic: optional CORDIC, FMAC,
DMA, and target counters stay behind target adapters. Profiling is compiled to
literal no-ops in production builds; data-layout and clamp changes are accepted
only after per-target disassembly and P99 timing evidence.

**Tech Stack:** C11, Clang/LLVM embedded Arm, Cortex-M4F where available,
portable `uint32_t` arithmetic, existing `tests/foc` Host matrix, Makefile
builds, target-specific DWT/performance-counter adapters.

**Document-only scope:** This plan changes neither source code nor target
configuration by itself. All C fragments below define intended interfaces or
test shape for a later implementation; they are not code changes in this task.

---

## V2.1 assessment and scope decision

| V2.1 proposal | Decision | Placement |
| --- | --- | --- |
| BAM32 angle storage and natural wrap | Accept | Mandatory P1.0 |
| 8-byte hot-data alignment / LDRD | Measure first | Conditional P3.2 |
| `VMAXNM` / `VMINNM` clamp | Reject as stated for Cortex-M4F | Conditional P3.3 assembly audit |
| STM32G4 FMAC filtering | Optional target extension | Conditional P5 |
| Zero-cost profiling closure | Accept | Mandatory P0 |

`VMAXNM.F32` and `VMINNM.F32` must not be used as a Cortex-M4F assumption.
The M4F VFP instruction subset does not provide them. `__builtin_fminf()` and
`__builtin_fmaxf()` are not accepted until the target disassembly proves that
they neither call a library function nor alter NaN, signed-zero, or saturation
semantics.

BAM32 is a cross-chip data-model change, not an STM32G4 feature. It is accepted
because modulo-2^32 integer wrap, LUT high-bit indexing, and signed angular
difference are portable C operations. A target CORDIC adapter may consume the
same bit pattern directly, but the generic `foc/` tree must not know that such a
peripheral exists. When speed remains float or Q-format, converting `speed * Ts`
to BAM32 is still required once at the integrator boundary; V2.1 only removes
repeated conversions from wrap, LUT, and optional trig-adapter paths.

## File structure and ownership

| File | Responsibility after V2.1 |
| --- | --- |
| `foc/math/foc_angle.h` | Public opaque angle value and angle API; expose no platform headers |
| `foc/math/foc_angle.c` | BAM32 conversion, difference, pair sin/cos dispatch, software fallback |
| `foc/math/foc_trig.h` | Internal backend contract; only scalar/BAM32 types |
| `foc/math/foc_trig_lut.c` | Software LUT backend using BAM32 high-bit indexing |
| `foc/middleware/foc_core.c` | Park/IPark consume one sin/cos pair per high-frequency step |
| `foc/motor/motor_control.c` | One pair trig call; profile data stays local until one per-instance publish |
| `foc/motor/motor_profile.h` | Public immutable HF-profile snapshot and per-motor snapshot API |
| `foc/motor/foc_hf_profile.h` | Compile-time profile no-op boundary and private enabled-build helpers |
| `target/<chip>/foc_hf_profile_port.c` | Optional target cycle-counter reader, built only by profile builds; each target may name or implement this adapter differently |
| `peripheral/<chip>/` | Optional trig/FMAC implementations; never included by generic FOC code |
| `tests/foc/test_trig.c` | BAM32 wrap, conversion, sin/cos pair, and full-turn error tests |
| `tests/foc/test_numeric.c` | Public float/fixed angle API compatibility tests |
| `tests/foc/Makefile` | Float/fixed/strict-aliasing regression matrix |

## Task 1: P0 — make profiling per-motor, coherent, and zero-cost when disabled

**Files:**
- Create: `foc/motor/motor_profile.h`
- Create: `foc/motor/foc_hf_profile.h`
- Modify: `foc/foc_config.h`
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor.c`
- Modify: `foc/motor/motor_control.c`
- Modify: `target/<chip>/...` high-frequency ISR adapter(s) that currently time the ISR
- Modify: `foc/app/foc_app.c`
- Test: `tests/foc/test_motor_control_runtime.c`
- Test: `tests/foc/Makefile`

- [ ] **Step 1: Add a disabled-by-default configuration switch**

Add this block to `foc/foc_config.h` before including feature headers:

```c
#ifndef FOC_HF_PROFILE
#define FOC_HF_PROFILE 0
#endif

#ifndef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 0
#endif

#if !FOC_HF_PROFILE
#undef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 0
#endif

#if (FOC_HF_PROFILE_LEVEL < 0) || (FOC_HF_PROFILE_LEVEL > 2)
#error "FOC_HF_PROFILE_LEVEL must be 0, 1, or 2"
#endif
```

The macro is numeric, not merely defined/undefined, so target makefiles can use
`-DFOC_HF_PROFILE=1 -DFOC_HF_PROFILE_LEVEL=1` for a total-cycle measurement,
or level 2 for a stage breakdown. Reject any level other than 0, 1, or 2 at
compile time. `FOC_HF_PROFILE=0` always forces level 0, even if a target build
accidentally supplies a different level.

- [ ] **Step 2: Define ownership, record shape, and read API before instrumentation**

Do not expose six global counters. Declare a value snapshot in
`motor_profile.h`; it represents one completed high-frequency step of exactly
one `motor_handle_t`:

```c
typedef struct {
    uint32_t wSampleSequence;
    uint32_t wTotalCycles;
    uint32_t wSampleCurrentCycles;
    uint32_t wClarkeCycles;
    uint32_t wParkCycles;
    uint32_t wIparkCycles;
    uint32_t wModulateCycles;
    uint32_t wCommitCycles;
    uint32_t wValidFlags;
    foc_result_t eResult;
} motor_hf_profile_snapshot_t;

foc_result_t motor_GetHighFrequencyProfileSnapshot(
    const motor_handle_t *ptMotor,
    motor_hf_profile_snapshot_t *ptSnapshot);
```

The public API returns a copy, never a pointer into mutable motor storage. The
application owns the motor-to-display-name/ID association; the FOC core must
not add a global motor ID, a global profile table, or a lookup to the
high-frequency path. In a non-profile build, retain a compatible API only if
needed by application code and return the project's existing disabled/not-ready
result without allocating profile storage; alternatively, gate the caller and
declaration together. Select one convention and document it consistently.

In `motor_private.h`, add private `motor_hf_profile_t` storage to
`motor_impl_t` only when `FOC_HF_PROFILE != 0`. It contains the latest complete
snapshot and any publisher bookkeeping. Extend the existing
`MOTOR_HANDLE_STORAGE_SIZE` static assertion so both no-profile and level-2
builds prove that every target's opaque handle remains large enough.

`wSampleSequence` increments once per completed publish for that motor. Define
valid bits for total time and each requested stage; when a stage is skipped or
the high-frequency step exits early, publish the result and valid-bit state
rather than accidentally retaining a timing from the preceding cycle.

- [ ] **Step 3: Define the profile macro boundary with a true disabled form**

Create `foc/motor/foc_hf_profile.h` with the following disabled path. The
disabled macros deliberately do not mention their arguments, so no expression,
temporary, counter read, `volatile` write, or symbol reference remains.

```c
#ifndef FOC_HF_PROFILE_H
#define FOC_HF_PROFILE_H

#include "foc_config.h"

#if FOC_HF_PROFILE_LEVEL >= 1
uint32_t foc_hf_profile_ReadCycles(void);
#define FOC_HF_PROFILE_TOTAL_BEGIN(name) \
    uint32_t name = foc_hf_profile_ReadCycles()
#define FOC_HF_PROFILE_TOTAL_END(name, destination) \
    do { (destination) = foc_hf_profile_ReadCycles() - (name); } while (0)
#else
#define FOC_HF_PROFILE_TOTAL_BEGIN(name)       do { } while (0)
#define FOC_HF_PROFILE_TOTAL_END(name, destination) do { } while (0)
#endif

#if FOC_HF_PROFILE_LEVEL >= 2
#define FOC_HF_PROFILE_STAGE_BEGIN(name) \
    uint32_t name = foc_hf_profile_ReadCycles()
#define FOC_HF_PROFILE_STAGE_END(name, destination) \
    do { (destination) = foc_hf_profile_ReadCycles() - (name); } while (0)
#else
#define FOC_HF_PROFILE_STAGE_BEGIN(name)       do { } while (0)
#define FOC_HF_PROFILE_STAGE_END(name, destination) do { } while (0)
#endif

#endif /* FOC_HF_PROFILE_H */
```

- [ ] **Step 4: Publish one complete per-motor record, not stage globals**

Replace every `#if defined(MOTOR_PROFILE_CYCLES)` block in
`foc/motor/motor_control.c` with `#if FOC_HF_PROFILE`, and use the total macros
only around the whole high-frequency step. Use stage macros around sampling,
Clarke, Park, IPark, modulation, and commit only when
`FOC_HF_PROFILE_LEVEL >= 2`. In a profile build, place the intermediate
timestamps and stage deltas in automatic locals inside
`motor_HighFrequencyStep()`. At the normal or error exit, assemble one
`motor_hf_profile_snapshot_t` and publish it to that motor's private
`motor_hf_profile_t` exactly once. Thus level 1 has exactly two counter reads
for the step total, while level 2 adds the stage counter reads deliberately.

Do not create `g_wSampleCurrentCycles`, `g_wClarkeCycles`, `g_wParkCycles`,
`g_wIparkCycles`, `g_wModulateCycles`, `g_wCommitCycles`, `g_wHFStepCycles`, or
equivalent file/global singleton state. `volatile` would prevent selected
compiler optimizations but cannot make six separately written values a coherent
record, and a second motor would overwrite the first motor's data.

The high-frequency writer takes no lock: the established motor contract gives
each instance one non-reentrant high-frequency writer, and it performs only the
final structure assignment/publication. The low-rate
`motor_GetHighFrequencyProfileSnapshot()` reader uses that motor's existing
`motor_sync_if_t` for the short copy, with the adapter supplying any required
compiler/hardware ordering. This is portable across supported chips and avoids
assuming C11 atomics are lock-free. Do not put a lock, callback, global lookup,
or logging operation into the normal high-frequency loop. A later
target-validated double-buffer/seqlock reader may be considered only if
snapshot-read latency is shown to be a problem; it must still operate per motor
and publish a whole record.

- [ ] **Step 5: Gate target total timing and application reporting**

Move any target-owned ISR total counter, start timestamp, counter reads, probe
math, and profile-only assignments behind `#if FOC_HF_PROFILE`. A generic ISR
adapter must call the application high-frequency entry exactly as before when
profiling is disabled; target-specific ADC/PWM flag policy stays outside this
feature.

In `foc/app/foc_app.c`, remove all `extern` declarations for profile counters.
For each registered application motor, call
`motor_GetHighFrequencyProfileSnapshot()` at low rate and log its own copied
snapshot with the application's label. It must never combine fields read from
separate globals. The application can compare several motors, but this happens
outside every motor's high-frequency step.

- [ ] **Step 6: Prove zero residual cost and multi-motor record isolation**

Run the normal target build, then inspect the ELF:

```powershell
mingw32-make BUILD=debug-rel TARGET_CHIP=stm32g431 FOC_NUMERIC=float
D:\software\llvm_for_arm\bin\llvm-nm.exe build\template.elf |
    Select-String 'g_wHFStepCycles|g_wParkCycles|foc_hf_profile_ReadCycles|motor_hf_profile'
```

Expected: the second command prints no matches. Repeat with
`FOC_HF_PROFILE=1` at levels 1 and 2; profile-only symbols may then be present.
Also disassemble `motor_HighFrequencyStep()` in the no-profile image: it must
contain no cycle-counter read or profile publication path.

Add a host test with two independent motor handles and mock ADC/PWM adapters.
Interleave completed steps `A, B, A`, copy both snapshots, and assert that each
snapshot has its own sequence, result, valid bits, and stage/total timings. The
test must fail if a record can be overwritten by the other motor or if a reader
can observe a mixture of two publications. Report no-profile, level-1, and
level-2 P99 separately: level-2 instrumentation overhead is calibration data,
not a production performance baseline.

## Task 2: P1.0 — introduce the hardware-neutral BAM32 angle contract

**Files:**
- Modify: `foc/math/foc_angle.h`
- Modify: `foc/math/foc_angle.c`
- Modify: `foc/math/foc_math.c`
- Modify: `foc/control/foc_pll.c`
- Modify: `foc/motor/motor_control.c`
- Modify: `foc/motor/motor_position.c`
- Modify: `foc/observer/foc_hall.c`
- Modify: `foc/observer/foc_hfi.c`
- Modify: `foc/optimization/foc_cogging.c`
- Modify: `foc/optimization/foc_optimization.c`
- Modify: `tests/foc/test_trig.c`
- Modify: `tests/foc/test_numeric.c`

- [ ] **Step 1: Write failing public-API tests before changing representation**

Replace direct construction of `foc_angle_t` in `tests/foc/test_trig.c` with
public constructors. Add the following test function and call it from
`test_trig()` for both numeric backends:

```c
static void test_bam32_wrap_and_difference(void)
{
    foc_angle_t near_wrap = foc_angle_from_turns(0.99999f);
    foc_angle_t step = foc_angle_from_turns(0.00002f);
    foc_angle_t wrapped = foc_angle_add(near_wrap, step);

    TEST_NEAR(foc_angle_to_turns(wrapped), 0.00001f, 4e-5f);
    TEST_NEAR(foc_to_float(foc_angle_diff(
                  foc_angle_from_turns(0.05f),
                  foc_angle_from_turns(0.95f))),
              0.10f, 4e-5f);
}
```

Run the existing matrix before modifying production code. Expected: it fails to
compile because `foc_angle_add()` does not yet exist.

```powershell
mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
```

- [ ] **Step 2: Replace scalar storage with BAM32 and define only explicit operations**

In `foc/math/foc_angle.h`, replace the public member with an unsigned BAM field
and add APIs that make arithmetic explicit:

```c
typedef struct {
    uint32_t wBam32;
} foc_angle_t;

foc_angle_t foc_angle_from_turns(float fTurns);
foc_angle_t foc_angle_from_scalar(foc_scalar_t qTurns);
float foc_angle_to_turns(foc_angle_t tAngle);
foc_angle_t foc_angle_add(foc_angle_t tLeft, foc_angle_t tRight);
foc_angle_t foc_angle_add_scalar(foc_angle_t tAngle,
                                 foc_scalar_t qTurns);
foc_angle_t foc_angle_wrap(foc_angle_t tAngle);
```

`foc_angle_wrap()` becomes an inline identity function or a documented identity
implementation. Do not retain a modulo or comparison in it. If control speed is
still represented by `foc_scalar_t`, only `foc_angle_add_scalar()` converts the
per-cycle increment to BAM32; LUT and backend dispatch receive BAM32 directly.

- [ ] **Step 3: Implement conversions and modular difference without implementation-defined casts**

In `foc/math/foc_angle.c`, use a common conversion constant and an unsigned
subtraction for the modulo operation. The conversion functions are permitted at
the float/scalar boundary, not inside LUT indexing or hardware adapters.

```c
#define FOC_BAM32_PER_TURN 4294967296.0f

foc_angle_t foc_angle_add(foc_angle_t tLeft, foc_angle_t tRight)
{
    return (foc_angle_t){ .wBam32 = tLeft.wBam32 + tRight.wBam32 };
}

foc_scalar_t foc_angle_diff(foc_angle_t tTarget, foc_angle_t tActual)
{
    uint32_t wDelta = tTarget.wBam32 - tActual.wBam32;
    int32_t nDelta = (wDelta < 0x80000000U) ?
                     (int32_t)wDelta :
                     -(int32_t)(0U - wDelta);
    return foc_scalar_from_bam32_delta(nDelta);
}
```

Define `foc_scalar_from_bam32_delta()` for each numeric backend with explicit
rounding and saturation. Do not convert an arbitrary `uint32_t >= 0x80000000`
directly to `int32_t`.

- [ ] **Step 4: Migrate every direct `.qTurns` use through the new API**

Migrate all current direct-field users listed in this task. Examples:

```c
/* before */
ptHfi->tPhase.qTurns = foc_add_sat(ptHfi->tPhase.qTurns,
                                   ptHfi->tParams.qPhaseStep);
ptHfi->tPhase = foc_angle_wrap(ptHfi->tPhase);

/* after */
ptHfi->tPhase = foc_angle_add_scalar(ptHfi->tPhase,
                                      ptHfi->tParams.qPhaseStep);
```

For cogging-table interpolation, add a helper that returns integer table index
and fraction from `wBam32`; the index must derive from integer multiplication
and shifting, not float multiplication. For example, with `N` table entries:

```c
uint64_t wPosition = (uint64_t)tAngle.wBam32 * hwCount;
hwIndex = (uint16_t)(wPosition >> 32);
wFraction = (uint32_t)wPosition;
```

- [ ] **Step 5: Run float, fixed, and strict-aliasing matrices**

```powershell
mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc `
  STRICT_ALIAS_CFLAGS='-O2 -fstrict-aliasing -Wstrict-aliasing=2' clean all
```

Expected: all matrices report `FOC tests: PASS (0 failures)`. Do not continue
to LUT or CORDIC integration until both numeric backends pass.

## Task 3: P1.1 — make SinCos a single backend transaction

**Files:**
- Modify: `foc/math/foc_trig.h`
- Modify: `foc/math/foc_trig_lut.c`
- Modify: `foc/math/foc_angle.c`
- Modify: `foc/middleware/foc_core.c`
- Modify: `foc/motor/motor_control.c`
- Modify: `tests/foc/test_trig.c`
- Modify: `tests/foc/test_transform.c`

- [ ] **Step 1: Add pair-result tests**

Add this public test to `tests/foc/test_trig.c`:

```c
static void test_sincos_pair(float fTurns)
{
    foc_scalar_t qSin;
    foc_scalar_t qCos;

    foc_angle_sincos(foc_angle_from_turns(fTurns), &qSin, &qCos);
    TEST_NEAR(foc_to_float(qSin), sinf(fTurns * 6.283185307179586f), 4e-5f);
    TEST_NEAR(foc_to_float(qCos), cosf(fTurns * 6.283185307179586f), 4e-5f);
}
```

Call it for 0, 0.125, 0.25, 0.5, 0.75, and 0.99999 turns. Compile it before
adding `foc_angle_sincos`; expected result is a compile failure.

- [ ] **Step 2: Define the pair API and software-backend contract**

Add the following declaration to `foc/math/foc_angle.h`:

```c
void foc_angle_sincos(foc_angle_t tAngle,
                      foc_scalar_t *pqSin,
                      foc_scalar_t *pqCos);
```

Change `foc_trig_backend_t` to receive BAM32 rather than `foc_scalar_t` turns:

```c
typedef struct {
    void (*fnSinCosBam32)(uint32_t wBam32,
                           foc_scalar_t *pqSin,
                           foc_scalar_t *pqCos);
    foc_angle_t (*fnAtan2)(foc_scalar_t qY, foc_scalar_t qX);
} foc_trig_backend_t;
```

No backend interface may contain an STM32 type or vendor header.

- [ ] **Step 3: Replace float LUT phase conversion with integer indexing**

Implement the software LUT phase using BAM32 high bits. For the existing
512-entry quarter-wave table, derive quadrant, index, and interpolation fraction
from the BAM value; no `float fPhase = turns * 4.0f` is allowed in this path.

```c
uint32_t wQuadrant = wBam32 >> 30;
uint32_t wQuarter = wBam32 & 0x3FFFFFFFU;
uint32_t wIndex = wQuarter >> 21;
uint32_t wFraction = (wQuarter >> 5) & 0xFFFFU;
```

Use the same quadrant reduction for cosine, with a quarter-turn BAM offset.
Clamp only the table endpoint index; do not reintroduce full-angle wrap logic.

- [ ] **Step 4: Consume one pair in the high-frequency path**

Replace these two lines in `foc/motor/motor_control.c`:

```c
foc_scalar_t qSinTheta = foc_angle_sin(angle);
foc_scalar_t qCosTheta = foc_angle_cos(angle);
```

with:

```c
foc_scalar_t qSinTheta;
foc_scalar_t qCosTheta;
foc_angle_sincos(angle, &qSinTheta, &qCosTheta);
```

Leave `foc_park_cached()` and `foc_ipark_cached()` unchanged. Update the public
single-result functions to call the pair implementation, preserving their
existing external behavior.

- [ ] **Step 5: Verify numerical and transform behavior**

```powershell
mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
```

Expected: both numeric binaries print `FOC tests: PASS (0 failures)`. The
maximal float LUT error must remain at or below `2e-6`; fixed must remain within
its existing `4e-5` tolerance.

## Task 4: P3.2/P3.3 — admit layout and clamp changes only with evidence

**Files:**
- Modify only if the admission check passes: `foc/motor/motor_control_types.h`,
  `foc/modulation/foc_modulation.c`, `foc/math/foc_numeric.c`
- Test: `tests/foc/test_modulation.c`, `tests/foc/test_control.c`
- Inspect: target ELF disassembly and profile report

- [ ] **Step 1: Capture a post-P1 optimized disassembly and timing baseline**

Build the reference target in no-profile and profile configurations:

```powershell
mingw32-make BUILD=debug-rel TARGET_CHIP=stm32g431 FOC_NUMERIC=float
D:\software\llvm_for_arm\bin\llvm-objdump.exe -d build\template.elf |
  Select-String 'motor_HighFrequencyStep|foc_svpwm|foc_sat' -Context 0,80
```

Capture 100000 cycles for the no-profile control image using the target profile
procedure. Record mean, P99, max, and overrun count before changing layout or
clamps.

- [ ] **Step 2: Apply layout changes only if loads/stores dominate an identified hot block**

If the disassembly and stage counters identify repeated stack spills or adjacent
hot fields, group those fields by access pattern in `motor_control_t` or its
V2.1 replacement. Use natural 4-byte alignment first. Add `_Alignas(8)` only
when all three conditions are true:

1. `sizeof()` growth is accepted by the motor handle static assertion;
2. target disassembly changes to fewer accesses or a valid paired transfer;
3. P99 improves by at least 1% without increasing max latency.

Add `_Static_assert` checks for the selected offsets, for example:

```c
_Static_assert(offsetof(motor_hf_runtime_t, tCurrent) % 4U == 0U,
               "D/Q current must retain word alignment");
```

Do not assert an LDRD instruction or a fixed cycle count in portable code.

- [ ] **Step 3: Audit clamp code generation without changing semantics first**

Compile `foc_modulation.c` and `foc_numeric.c` for float and fixed, inspect the
clamp path, and preserve the existing `foc_sat()` behavior for NaN, range, and
saturation. Do not introduce `__builtin_fminf` or `__builtin_fmaxf` until the
target assembly proves no call to `fminf`/`fmaxf` and the tests below pass.

```powershell
mingw32-make BUILD=debug-rel TARGET_CHIP=stm32g431 FOC_NUMERIC=float
D:\software\llvm_for_arm\bin\llvm-nm.exe build\template.elf |
  Select-String 'fminf|fmaxf'
```

Expected: no output before accepting a builtin-based implementation. For fixed,
evaluate `__SSAT`/`__USAT` only for ranges that exactly match the intrinsic's
semantics; arbitrary PID bounds remain on the proven generic path.

- [ ] **Step 4: Re-run modulation and controller regression tests**

```powershell
mingw32-make -C tests/foc SHELL=cmd.exe CC=gcc clean all
```

Expected: `FOC tests: PASS (0 failures)`. Reject the change if float/fixed
outputs, duty bounds, or P99 regress.

## Task 5: P5 — optional target accelerator adapters, not core dependencies

**Files:**
- Create only for a target that passes the admission check:
  `peripheral/<chip>/foc_trig_accel_adapter.c`
- Create only when filter latency is accepted:
  `peripheral/<chip>/foc_filter_accel_adapter.c`
- Modify: the relevant `target/<chip>/target.mk`
- Test: target-specific adapter tests plus `tests/foc` software backend matrix

- [ ] **Step 1: Keep CORDIC integration behind the BAM32 backend contract**

An accelerator adapter may implement:

```c
static void target_trig_SinCosBam32(uint32_t wBam32,
                                    foc_scalar_t *pqSin,
                                    foc_scalar_t *pqCos);
```

It may program hardware at initialization or when changing function mode, but
must not expose register types through `foc_trig.h`. It must return both values
from one transaction and must have a software-backend fallback selected by the
target build.

- [ ] **Step 2: Admit FMAC only after proving one-sample latency is acceptable**

Before creating an FMAC adapter, document the filter equation, coefficient
format, Q1.15 quantization error, ownership, input/output buffer depth, DMA
channel, interrupt priority, and whether the observer accepts one-cycle delay.
Reject FMAC for a path that needs its result before the same ISR can compute
PWM, unless a measured polling transaction is faster than the software filter.

- [ ] **Step 3: Compare accelerator and software backends on the target**

For the same 100000-cycle workload, collect a paired report containing backend
name, numerical maximum error against software reference, mean, P99, max,
overruns, DMA interrupts, and filter latency. Accept the accelerator only if
P99 improves, max does not regress, and no new ISR is introduced on the
high-frequency critical path. A target without such an accelerator remains on
the software backend and is still a fully supported configuration.

## Task 6: Final acceptance report

**Files:**
- Modify: `docs/foc_app_HighFrequencyISR具体函数的资源占用优化方案（第二版）.md`
- Create: `docs/foc_app_HighFrequencyISR高频环路V2.1验证报告.md`

- [ ] **Step 1: Record exact build and workload metadata**

The report must contain chip, clock, compiler version, `BUILD`, numeric backend,
trig backend, observer mode, profile enable/level, motor count, sample count,
mean, P99, max, overruns, ADC errors, and PWM commit failures. A level-2 report
must also state its measured instrumentation overhead relative to the no-profile
image; it is not a substitute for the production timing result.

- [ ] **Step 2: Apply the V2.1 acceptance gates**

Accept only if all are true:

```text
float + fixed Host matrices: PASS
strict-aliasing matrix: PASS
production ELF profile symbols: absent
two-motor interleave snapshot test: PASS
each profile snapshot is attributable to one motor and one completed sample
P99: no worse than V2 baseline
max: no worse than V2 baseline
overruns / ADC errors / PWM failures: 0
generic foc/ tree: no vendor headers or target register access
targets without CORDIC/FMAC: software backend matrix passes unchanged
```

- [ ] **Step 3: Update V2 with evidence, not estimates**

Promote BAM32, profile closure, layout changes, clamp changes, or accelerator
results into the V2 document only when the exact report data satisfies Task 6
Step 2. Retain rejected options in the report with their measured reason rather
than claiming a theoretical cycle saving.
