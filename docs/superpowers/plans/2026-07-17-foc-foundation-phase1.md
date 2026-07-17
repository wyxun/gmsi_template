# Universal FOC Foundation Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the architecture-independent float/Q16.15 numeric foundation,
coordinate transforms, modulation, basic controllers, filters, PLL, LTD, and
feedforward modules required by later multi-motor and observer phases.

**Architecture:** Select one numeric backend per build while exposing one typed
API. Keep all mutable state in caller-owned structs. High-frequency per-unit
operations use bounded 32-bit fixed-point arithmetic; initialization-only wide
operations are explicitly separated.

**Tech Stack:** C11, GNU Make, LLVM Embedded Toolchain for Arm, native C test
harness, AT32F413 Cortex-M4F target.

**Repository constraint:** Do not run `git add`, `git commit`, branch creation,
or push commands. The user will manage source-control operations.

---

## File Map

Create these focused modules:

- `foc/math/foc_numeric.h`: backend selection, scalar constants and conversions.
- `foc/math/foc_numeric.c`: checked conversion and slow/wide arithmetic.
- `foc/math/foc_angle.h`: distinct normalized electrical-angle type.
- `foc/math/foc_angle.c`: wrap, difference, sine/cosine and atan2 conversion.
- `foc/control/foc_pid.h/.c`: PID with limits and anti-windup.
- `foc/control/foc_filter.h/.c`: first-order and configurable biquad filters.
- `foc/control/foc_pll.h/.c`: phase tracking loop.
- `foc/control/foc_ltd.h/.c`: linear tracking differentiator.
- `foc/control/foc_feedforward.h/.c`: current and velocity feedforward terms.
- `foc/modulation/foc_modulation.h/.c`: SVPWM, SPWM and third-harmonic SPWM.
- `tests/foc/test_foc.c`: native test runner and assertions.
- `tests/foc/test_numeric.c`: numeric and angle tests.
- `tests/foc/test_transform.c`: Clarke/Park/iPark tests.
- `tests/foc/test_modulation.c`: modulation tests.
- `tests/foc/test_control.c`: controller, filter, PLL, LTD and feedforward tests.
- `tests/foc/Makefile`: float and fixed native builds.

Modify these files:

- `Makefile`: expose `FOC_NUMERIC=float|fixed`, new include paths and sources.
- `foc/foc_config.h`: remove hard-coded FPU selection.
- `foc/math/foc_math_types.h`: compatibility aliases onto the new backend.
- `foc/math/foc_math_port.h`: pure-C fast arithmetic wrappers.
- `foc/math/foc_math.h/.c`: use normalized angles and backend operations.
- `foc/middleware/foc_core.h/.c`: keep transforms, move modulation out.
- `foc/foc.h`: export the new public modules.
- `target/at32f413/target.mk`: include all phase-one sources through wildcards.

## Task 1: Build-Time Numeric Selection and Native Test Harness

**Files:**

- Modify: `Makefile`
- Modify: `foc/foc_config.h`
- Create: `tests/foc/Makefile`
- Create: `tests/foc/test_foc.c`

- [ ] **Step 1: Add a failing backend-selection smoke test**

Create a runner whose suites return failure counts:

```c
#include <stdio.h>

int test_numeric(void);
int test_transform(void);
int test_modulation(void);
int test_control(void);

int main(void)
{
    int nFailures = 0;
    nFailures += test_numeric();
    nFailures += test_transform();
    nFailures += test_modulation();
    nFailures += test_control();
    printf("FOC tests: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures == 0 ? 0 : 1;
}
```

Create suite functions in the listed test files. `test_numeric()` includes
`foc_numeric.h` and checks `FOC_ZERO`; the remaining suites return zero until
their concrete tests are added by Tasks 3 through 5.

- [ ] **Step 2: Add native float and fixed build targets**

`tests/foc/Makefile` must compile the same production sources twice:

```make
CC ?= clang
CFLAGS = -std=c11 -Wall -Wextra -Werror -I../../foc -I../../foc/math \
         -I../../foc/middleware -I../../foc/control -I../../foc/modulation
TEST_SRCS = test_foc.c test_numeric.c test_transform.c \
            test_modulation.c test_control.c
FOC_SRCS = ../../foc/math/foc_numeric.c ../../foc/math/foc_angle.c \
           ../../foc/math/foc_math.c ../../foc/middleware/foc_core.c \
           ../../foc/modulation/foc_modulation.c \
           ../../foc/control/foc_pid.c ../../foc/control/foc_filter.c \
           ../../foc/control/foc_pll.c ../../foc/control/foc_ltd.c \
           ../../foc/control/foc_feedforward.c

.PHONY: all float fixed clean
all: float fixed
float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -lm -o foc_test_float
	./foc_test_float
fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -o foc_test_fixed
	./foc_test_fixed
clean:
	$(RM) foc_test_float foc_test_fixed
```

- [ ] **Step 3: Expose numeric selection in the root Makefile**

Add validation and definitions:

```make
FOC_NUMERIC ?= float
ifeq ($(FOC_NUMERIC),float)
    C_DEFS += -DFOC_NUMERIC_FLOAT=1
else ifeq ($(FOC_NUMERIC),fixed)
    C_DEFS += -DFOC_NUMERIC_FIXED=1
else
    $(error FOC_NUMERIC must be float or fixed)
endif
```

Replace `FOC_USE_FPU_HARDWARE` in `foc_config.h` with a compile-time check that
exactly one backend is selected. Do not infer the backend from CPU architecture.

- [ ] **Step 4: Run the tests and confirm the intentional failure**

Run:

```powershell
mingw32-make -C tests/foc float
mingw32-make -C tests/foc fixed
```

Expected: both targets fail because `foc_numeric.h` and the phase-one production
sources do not exist yet. This proves the harness cannot pass before the new
numeric layer is implemented.

## Task 2: Scalar and Angle Backends

**Files:**

- Create: `foc/math/foc_numeric.h`
- Create: `foc/math/foc_numeric.c`
- Create: `foc/math/foc_angle.h`
- Create: `foc/math/foc_angle.c`
- Modify: `foc/math/foc_math_types.h`
- Modify: `foc/math/foc_math_port.h`
- Modify: `foc/math/foc_math.h`
- Modify: `foc/math/foc_math.c`
- Modify: `tests/foc/test_numeric.c`

- [ ] **Step 1: Write numeric and angle tests**

Test these public invariants:

```c
CHECK_NEAR(foc_to_float(FOC_SCALAR(0.5f)), 0.5f, tolerance);
CHECK(foc_to_float(foc_sat(FOC_SCALAR(2.0f), FOC_NEG_ONE,
                           FOC_ONE)) <= 1.0f);
CHECK_NEAR(foc_to_float(foc_mul_pu(FOC_SCALAR(0.5f),
                                    FOC_SCALAR(-0.25f))),
           -0.125f, tolerance);
CHECK_NEAR(foc_angle_to_turns(foc_angle_from_turns(1.25f)),
           0.25f, angle_tolerance);
CHECK_NEAR(foc_to_float(foc_angle_sin(foc_angle_from_turns(0.25f))),
           1.0f, trig_tolerance);
CHECK_NEAR(foc_angle_to_turns(
               foc_angle_atan2(FOC_ONE, FOC_ZERO)),
           0.25f, angle_tolerance);
```

Also test negative wrapping, shortest signed angle difference, zero square root,
division-by-zero rejection and fixed conversion saturation.

- [ ] **Step 2: Implement one backend-selected scalar API**

Expose these exact concepts from `foc_numeric.h`:

```c
#if defined(FOC_NUMERIC_FLOAT)
typedef float foc_scalar_t;
#elif defined(FOC_NUMERIC_FIXED)
typedef int32_t foc_scalar_t;
#else
#error "Select FOC_NUMERIC_FLOAT or FOC_NUMERIC_FIXED"
#endif

#define FOC_Q_FRACTION_BITS 15

foc_scalar_t foc_from_float(float fValue);
float foc_to_float(foc_scalar_t qValue);
foc_scalar_t foc_add_sat(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_sub_sat(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_mul_pu(foc_scalar_t qA, foc_scalar_t qB);
foc_scalar_t foc_mul_wide(foc_scalar_t qA, foc_scalar_t qB);
bool foc_div_checked(foc_scalar_t qNumerator,
                     foc_scalar_t qDenominator,
                     foc_scalar_t *pqResult);
foc_scalar_t foc_sat(foc_scalar_t qValue,
                     foc_scalar_t qMinimum,
                     foc_scalar_t qMaximum);
```

For fixed `foc_mul_pu`, require both operands in `[-1, 1]`, use a 32-bit product,
round before shifting, and saturate. Use 64-bit arithmetic only in
`foc_mul_wide`, checked division and conversion helpers.

- [ ] **Step 3: Implement a distinct normalized angle type**

Use a wrapper to prevent accidental scalar/angle mixing:

```c
typedef struct {
    foc_scalar_t qTurns;
} foc_angle_t;

foc_angle_t foc_angle_from_turns(float fTurns);
float foc_angle_to_turns(foc_angle_t tAngle);
foc_angle_t foc_angle_wrap(foc_angle_t tAngle);
foc_scalar_t foc_angle_diff(foc_angle_t tTarget,
                            foc_angle_t tActual);
foc_scalar_t foc_angle_sin(foc_angle_t tAngle);
foc_scalar_t foc_angle_cos(foc_angle_t tAngle);
foc_angle_t foc_angle_atan2(foc_scalar_t qY,
                            foc_scalar_t qX);
```

Float trigonometry converts turns to radians at the backend boundary. Fixed
trigonometry indexes a quarter-wave table with normalized turns and never treats
the scalar as radians.

- [ ] **Step 4: Preserve compatibility names without preserving old semantics**

In `foc_math_types.h`, alias `q_type` to `foc_scalar_t` and `_Q(v)` to
`FOC_SCALAR(v)`. Keep `Q_ZERO`, `Q_HALF`, and `Q_ONE`. Mark direct angle use of
`q_type` as deprecated in comments; transform APIs are changed in Task 3.

- [ ] **Step 5: Run numeric tests**

Run both native targets. Expected: numeric suite passes in both builds; the
remaining suites contain no assertions until their corresponding tasks add them.

## Task 3: Coordinate Transforms and Modulation

**Files:**

- Modify: `foc/middleware/foc_core.h`
- Modify: `foc/middleware/foc_core.c`
- Create: `foc/modulation/foc_modulation.h`
- Create: `foc/modulation/foc_modulation.c`
- Modify: `tests/foc/test_transform.c`
- Modify: `tests/foc/test_modulation.c`

- [ ] **Step 1: Write transform round-trip tests**

For angles 0, 0.125, 0.25, 0.5 and 0.875 turns, verify:

```c
foc_ab_t tAB = { FOC_SCALAR(0.35f), FOC_SCALAR(-0.2f) };
foc_dq_t tDQ;
foc_ab_t tRoundTrip;
foc_park(&tAB, foc_angle_from_turns(fTurns), &tDQ);
foc_ipark(&tDQ, foc_angle_from_turns(fTurns), &tRoundTrip);
CHECK_NEAR(foc_to_float(tRoundTrip.qAlpha), 0.35f, tolerance);
CHECK_NEAR(foc_to_float(tRoundTrip.qBeta), -0.2f, tolerance);
```

Test balanced Clarke input and null-pointer rejection through status return
codes.

- [ ] **Step 2: Replace overloaded AB/DQ field names**

Define distinct types:

```c
typedef struct { foc_scalar_t qAlpha; foc_scalar_t qBeta; } foc_ab_t;
typedef struct { foc_scalar_t qD; foc_scalar_t qQ; } foc_dq_t;

foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB);
foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ);
foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB);
```

Use shared sine/cosine results and backend multiply helpers. Do not call
`sinf/cosf` directly from transform code.

- [ ] **Step 3: Write modulation tests**

Verify zero vector produces 50% duty, every duty stays in `[0, 1]`, phase order
is correct for six sector vectors, and small input changes do not create a duty
step larger than the expected tolerance.

- [ ] **Step 4: Implement modulation as a separate component**

Expose:

```c
typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_duty_abc_t;

foc_result_t foc_svpwm(const foc_ab_t *ptVoltage,
                       foc_duty_abc_t *ptDuty);
foc_result_t foc_spwm(const foc_ab_t *ptVoltage,
                      foc_duty_abc_t *ptDuty);
foc_result_t foc_third_harmonic_spwm(const foc_ab_t *ptVoltage,
                                     foc_duty_abc_t *ptDuty);
```

Inputs are already normalized to bus voltage; remove the unused `qVbus`
parameter from the old SVPWM interface. Clamp at the modulation boundary.

- [ ] **Step 5: Run transform and modulation tests**

Expected: numeric, transform and modulation suites pass for float and fixed.

## Task 4: PID and Filters

**Files:**

- Create: `foc/control/foc_pid.h`
- Create: `foc/control/foc_pid.c`
- Create: `foc/control/foc_filter.h`
- Create: `foc/control/foc_filter.c`
- Modify: `tests/foc/test_control.c`

- [ ] **Step 1: Write PID behavior tests**

Cover proportional response, integral accumulation, output clamping, conditional
integration anti-windup, reset and two-instance isolation:

```c
foc_pid_t tPidA;
foc_pid_t tPidB;
foc_pid_Init(&tPidA, &tParams);
foc_pid_Init(&tPidB, &tParams);
(void)foc_pid_Step(&tPidA, FOC_SCALAR(0.5f), FOC_ZERO);
CHECK(foc_to_float(tPidA.qIntegrator) != 0.0f);
CHECK(foc_to_float(tPidB.qIntegrator) == 0.0f);
```

- [ ] **Step 2: Implement caller-owned PID state**

Use:

```c
typedef struct {
    foc_scalar_t qKp;
    foc_scalar_t qKiTs;
    foc_scalar_t qKdOverTs;
    foc_scalar_t qOutputMin;
    foc_scalar_t qOutputMax;
    foc_scalar_t qIntegratorMin;
    foc_scalar_t qIntegratorMax;
} foc_pid_params_t;

typedef struct {
    foc_pid_params_t tParams;
    foc_scalar_t qIntegrator;
    foc_scalar_t qPreviousError;
} foc_pid_t;
```

`Init` validates limits and uses already-discretized gains. `Step` conditionally
integrates when the saturated output would otherwise wind up further.

- [ ] **Step 3: Write filter tests**

Test first-order step convergence and three biquad coefficient sets. Verify reset,
bounded output for bounded input, and independent state for two instances.

- [ ] **Step 4: Implement first-order and biquad filters**

Use transposed direct form II for the biquad:

```c
qOutput = foc_add_sat(foc_mul_pu(qB0, qInput), qState1);
qState1 = foc_add_sat(foc_sub_sat(foc_mul_pu(qB1, qInput),
                                  foc_mul_pu(qA1, qOutput)),
                     qState2);
qState2 = foc_sub_sat(foc_mul_pu(qB2, qInput),
                      foc_mul_pu(qA2, qOutput));
```

Coefficient generation from cutoff, damping and sample time is a slow-path
initialization function. Also allow applications to supply precomputed
coefficients for fixed-only targets.

- [ ] **Step 5: Run control tests**

Expected: PID and filter tests pass in both numeric builds.

## Task 5: PLL, LTD, and Feedforward

**Files:**

- Create: `foc/control/foc_pll.h`
- Create: `foc/control/foc_pll.c`
- Create: `foc/control/foc_ltd.h`
- Create: `foc/control/foc_ltd.c`
- Create: `foc/control/foc_feedforward.h`
- Create: `foc/control/foc_feedforward.c`
- Modify: `tests/foc/test_control.c`

- [ ] **Step 1: Write PLL tracking tests**

Feed a wrapped constant-speed electrical angle for at least 500 samples. Verify
the estimated angle converges within configured turn tolerance, estimated speed
has the correct sign, reset clears validity, and discontinuous input causes a
temporary invalid state rather than a full-turn error.

- [ ] **Step 2: Implement PLL with normalized phase error**

The PLL owns a PID, estimated angle, normalized speed, lock counter and validity
flag. Phase error must use `foc_angle_diff`; never subtract wrapped angle storage
directly.

- [ ] **Step 3: Write LTD and feedforward tests**

LTD tests cover bounded target ramp and derivative sign. Feedforward tests cover
PMSM decoupling terms at zero and nonzero speed and verify configurable limits.

- [ ] **Step 4: Implement LTD and feedforward**

LTD is a stateful component with position, velocity, acceleration limit and
sample-period coefficients. Feedforward is stateless after initialization and
uses precomputed normalized PMSM coefficients:

```text
Vd_ff = -omega_e * Lq * Iq
Vq_ff =  omega_e * (Ld * Id + flux)
```

All products use bounded or explicitly wide helpers according to coefficient
range. Outputs are clamped before addition to controller voltage commands.

- [ ] **Step 5: Run all native tests**

Run `mingw32-make -C tests/foc clean all` and expect both executables to report
`FOC tests: PASS (0 failures)`.

## Task 6: Public API, AT32F413 Builds, and Portability Checks

**Files:**

- Modify: `foc/foc.h`
- Modify: `target/at32f413/target.mk`
- Modify: `peripheral/at32f413/foc_hal_mdi_adapter.c`
- Modify: existing `foc/app` call sites required by typed angle/DQ APIs
- Modify: `Makefile`

- [ ] **Step 1: Export phase-one modules**

Add control and modulation include paths to the root Makefile and include the
public headers from `foc/foc.h`. Keep compatibility aliases only where their
semantics are unchanged.

- [ ] **Step 2: Update existing call sites**

Replace overloaded `foc_ab_t` DQ usage with `foc_dq_t`, construct normalized
angles with `foc_angle_from_turns`, and call the new SVPWM interface after voltage
normalization. Do not add chip-specific conditionals to `foc/`.

- [ ] **Step 3: Build AT32F413 float configuration**

Run:

```powershell
mingw32-make TARGET_CHIP=at32f413 FOC_NUMERIC=float `
    BUILD_DIR=build/at32f413-float
```

Expected: ELF, BIN and HEX are generated; no compile or link errors.

- [ ] **Step 4: Build AT32F413 fixed configuration**

Run:

```powershell
mingw32-make TARGET_CHIP=at32f413 FOC_NUMERIC=fixed `
    BUILD_DIR=build/at32f413-fixed
```

Expected: ELF, BIN and HEX are generated; no float-only API leakage prevents the
fixed build.

- [ ] **Step 5: Check architecture boundaries and software helpers**

Run repository searches proving `foc/` does not include vendor headers. Inspect
fixed high-frequency objects with `llvm-nm` and `llvm-objdump`; fail the check if
transform, modulation or PID objects reference 64-bit division helpers or
unexpected 64-bit multiplication helpers.

- [ ] **Step 6: Record phase-one verification**

Document exact commands, pass/fail result, binary sizes, fixed helper-symbol
inspection and any remaining hardware-only limitations in the implementation
handoff. Do not claim motor-control performance without hardware testing.
