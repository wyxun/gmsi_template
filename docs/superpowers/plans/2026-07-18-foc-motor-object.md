# FOC Motor Object Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the FOC motor into a statically allocated opaque object with a small command API, an internal `fsm_rt_t` lifecycle, unified position sources, safe open-loop-to-closed-loop transfer, and stable diagnostics.

**Architecture:** Keep application policy in `foc/app`, lifecycle and control orchestration in `foc/motor`, and reusable position algorithms in `foc/observer`. The public handle is aligned opaque storage; a private implementation owns state, references, hardware bindings, position sources, transition data, and diagnostics. Main-loop FSM work is separated from deterministic high- and low-frequency steps.

**Tech Stack:** C11, MODUS `fsm_rt_t`/perf-counter protothreads, existing FOC numeric abstraction, Makefile/mingw32-make host tests, target builds through `make.bat`.

---

## Working rules

- Do not stage, commit, switch branches, or push; project instructions require explicit user authorization.
- Preserve unrelated worktree and submodule changes.
- Use `Makefile`/`make.bat` for target builds and `tests/foc/Makefile` for host tests.
- Keep public enum typedefs suffixed `_e` and struct typedefs suffixed `_t`.
- Keep public lines within 86 characters where practical.
- Run float and fixed host tests after every task that changes FOC code.

## File map

**Create**

- `foc/motor/motor_private.h`: private object layout and internal helpers; never included by app code.
- `foc/motor/motor_position.h`: unified public position-source interface and validity flags.
- `foc/motor/motor_position.c`: source validation, mechanical/electrical conversion, qualification, and angle blending.
- `foc/motor/motor_fsm.c`: lifecycle FSM and accepted-command processing.
- `tests/foc/test_motor_position.c`: position-source and wraparound transition tests.
- `tests/foc/test_motor_encapsulation.c`: public API and multi-instance behavioral tests.
- `tests/foc/compile_fail_motor_member_access.c`: negative compile fixture proving members are private.

**Modify**

- `foc/motor/motor_types.h`: public opaque handle, enums, configs, snapshot, faults.
- `foc/motor/motor.h`: small public lifecycle/reference/snapshot API.
- `foc/motor/motor.c`: init/reset, references, snapshot, safety, private access.
- `foc/motor/motor_control_types.h`: internal control-mode types only; remove public runtime layout.
- `foc/motor/motor_control.h`: internal high/low control entry points.
- `foc/motor/motor_control.c`: deterministic control calculation against private implementation.
- `foc/observer/foc_observer.h/.c`: adapt observer interface to unified position source.
- `foc/observer/foc_hall.h/.c`: expose Hall through unified position source.
- `foc/middleware/observer_lib.h/.c`: remove the obsolete parallel sensor abstraction.
- `foc/foc.h`: export the new public motor/position interfaces only.
- `foc/app/foc_app.h/.c`: remove motor internals and FOC math from the app FSM.
- `foc/app/phase_test.c`: stop bypassing motor ownership; retain only safe diagnostics or disable it.
- `tests/foc/test_motor.c`: migrate existing motor tests to public behavior.
- `tests/foc/test_observer.c`: assert unified observer/Hall adapters.
- `tests/foc/test_foc.c`: register new test functions.
- `tests/foc/Makefile`: add sources and negative compile check.
- `foc/README.md`: replace direct-member examples and document four supported run patterns.

### Task 1: Establish the baseline and encapsulation test harness

**Files:**
- Create: `tests/foc/compile_fail_motor_member_access.c`
- Create: `tests/foc/test_motor_encapsulation.c`
- Modify: `tests/foc/test_foc.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Record the existing host-test baseline**

Run:

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc all
```

Expected: both `foc_test_float` and `foc_test_fixed` build and report zero
failures. If the baseline fails, stop and record the pre-existing failure before
changing production code.

- [ ] **Step 2: Add a negative compile fixture**

Create this exact fixture:

```c
#include "motor.h"

int main(void)
{
    motor_handle_t tMotor;

    tMotor.tRt.wFaults = 0U;
    return 0;
}
```

It deliberately compiles now and must fail after Task 2 because `tRt` is no
longer public.

- [ ] **Step 3: Add the negative compile target without putting it in `all` yet**

Add to `tests/foc/Makefile`:

```make
.PHONY: encapsulation

encapsulation:
	@if $(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 \
	    compile_fail_motor_member_access.c -c -o compile_fail.o; then \
	    $(RM) compile_fail.o; \
	    echo "FAIL: motor internals are publicly accessible"; \
	    exit 1; \
	else \
	    $(RM) compile_fail.o; \
	    echo "PASS: motor internals are opaque"; \
	fi
```

- [ ] **Step 4: Add a public multi-instance test shell**

Add `int test_motor_encapsulation(void)` with two fake hardware contexts. For
now assert only that both `motor_Init()` calls succeed; register it in
`test_foc.c`. This test becomes the migration destination for checks that
currently inspect `tMotor.tHal`, `tMotor.tRt`, or `tMotor.tControl`.

- [ ] **Step 5: Re-run the baseline**

Run `mingw32-make -C tests/foc all`.

Expected: PASS. Do not run `encapsulation` yet; it is expected to fail until
Task 2.

### Task 2: Make the public motor object opaque

**Files:**
- Create: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/test_motor_encapsulation.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Change behavioral tests to request snapshots**

Replace direct state assertions with the intended API shape:

```c
motor_snapshot_t tSnapshotA;
motor_snapshot_t tSnapshotB;

TEST_CHECK(motor_GetSnapshot(&tMotorA, &tSnapshotA) == FOC_RESULT_OK);
TEST_CHECK(motor_GetSnapshot(&tMotorB, &tSnapshotB) == FOC_RESULT_OK);
TEST_CHECK(tSnapshotA.eState == MOTOR_STATE_FAULT);
TEST_CHECK((tSnapshotA.wFaults & MOTOR_FAULT_HARDWARE) != 0U);
TEST_CHECK(tSnapshotB.eState == MOTOR_STATE_IDLE);
```

- [ ] **Step 2: Run the tests and verify the new API is missing**

Run `mingw32-make -C tests/foc float`.

Expected: compile failure naming `motor_snapshot_t` or `motor_GetSnapshot`.

- [ ] **Step 3: Define the public storage and snapshot types**

In `motor_types.h`, expose aligned bytes instead of business members. Use a
single documented capacity constant and C11 alignment:

```c
#define MOTOR_HANDLE_STORAGE_SIZE 512U

typedef union {
    max_align_t tAlignment;
    uint8_t achStorage[MOTOR_HANDLE_STORAGE_SIZE];
} motor_handle_t;

typedef enum {
    MOTOR_STATE_UNINITIALIZED = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_FAULT,
} motor_state_e;

typedef struct {
    motor_state_e eState;
    motor_control_mode_e eControlMode;
    uint32_t wFaults;
    bool bPwmEnabled;
    bool bCurrentCalibrated;
    foc_angle_t tElectricalAngle;
    q_type qElectricalSpeed;
    foc_dq_t tCurrentReference;
    foc_dq_t tCurrent;
    foc_dq_t tVoltage;
    foc_duty_abc_t tDuty;
} motor_snapshot_t;
```

Keep the numeric size private to the library contract and add a comment that
changing it is an ABI change.

- [ ] **Step 4: Move the current object layout into `motor_private.h`**

Define `motor_impl_t` with the old fields first, then add:

```c
static inline motor_impl_t *motor_private_Get(motor_handle_t *ptMotor)
{
    return (motor_impl_t *)(void *)ptMotor->achStorage;
}

static inline const motor_impl_t *motor_private_GetConst(
    const motor_handle_t *ptMotor)
{
    return (const motor_impl_t *)(const void *)ptMotor->achStorage;
}

_Static_assert(sizeof(motor_impl_t) <= MOTOR_HANDLE_STORAGE_SIZE,
               "MOTOR_HANDLE_STORAGE_SIZE is too small");
_Static_assert(_Alignof(motor_handle_t) >= _Alignof(motor_impl_t),
               "motor_handle_t alignment is insufficient");
```

- [ ] **Step 5: Convert `motor.c` to private access and implement snapshot copy**

Every public function must validate its handle, obtain `motor_impl_t *`, and
operate only on the private type. `motor_GetSnapshot()` must not return a torn
multi-field snapshot. Add this optional portable synchronization binding to
`motor_config_t`, because FOC currently has no platform-neutral critical
section:

```c
typedef struct {
    void *pContext;
    uintptr_t (*fnEnter)(void *pContext);
    void (*fnExit)(void *pContext, uintptr_t wState);
} motor_sync_if_t;
```

Targets that run realtime steps in interrupts must bind both callbacks. Host
tests may omit them only when every call is single-threaded. The motor library
must not include CMSIS or vendor interrupt headers.

- [ ] **Step 6: Enable the encapsulation target in `all`**

Change:

```make
all: encapsulation float fixed
```

Run `mingw32-make -C tests/foc clean all`.

Expected: negative member-access compilation prints its expected compiler
error, `PASS: motor internals are opaque`, then both numeric suites pass.

### Task 3: Introduce the unified position-source contract

**Files:**
- Create: `foc/motor/motor_position.h`
- Create: `foc/motor/motor_position.c`
- Create: `tests/foc/test_motor_position.c`
- Modify: `foc/observer/foc_observer.h/.c`
- Modify: `foc/observer/foc_hall.h/.c`
- Modify: `foc/middleware/observer_lib.h/.c`
- Modify: `foc/foc.h`
- Modify: `tests/foc/test_observer.c`
- Modify: `tests/foc/test_foc.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Write failing interface-adapter tests**

Cover these exact behaviors:

- invalid interface with null `fnStep` is rejected;
- a fake magnetic encoder returns mechanical angle and motor conversion applies
  direction, pole pairs, mechanical zero, and electrical offset;
- Hall adapter returns valid electrical angle/speed and propagates invalid code;
- SMO/NLFO adapter accepts the common input and returns confidence/valid flags;
- output timestamp is copied for freshness checks.

Run `mingw32-make -C tests/foc float` and expect missing
`foc_position_source_if_t` symbols.

- [ ] **Step 2: Define one common interface**

Add validity flags, input, output, and interface:

```c
typedef enum {
    FOC_POSITION_VALID_ELECTRICAL_ANGLE = 1U << 0,
    FOC_POSITION_VALID_ELECTRICAL_SPEED = 1U << 1,
    FOC_POSITION_VALID_MECHANICAL_ANGLE = 1U << 2,
    FOC_POSITION_VALID_MECHANICAL_SPEED = 1U << 3,
    FOC_POSITION_VALID_MULTI_TURN       = 1U << 4,
    FOC_POSITION_INDEX_FOUND            = 1U << 5,
} foc_position_valid_flag_e;

typedef struct {
    foc_ab_t tCurrent;
    foc_ab_t tVoltage;
    uint32_t wTimestamp;
    q_type qSamplePeriod;
} foc_position_source_input_t;

typedef struct {
    foc_angle_t tElectricalAngle;
    q_type qElectricalSpeed;
    foc_angle_t tMechanicalAngle;
    q_type qMechanicalSpeed;
    int32_t lMechanicalTurns;
    q_type qConfidence;
    uint32_t wValidFlags;
    uint32_t wFaults;
    uint32_t wTimestamp;
} foc_position_source_output_t;

typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(
        void *pContext,
        const foc_position_source_input_t *ptInput,
        foc_position_source_output_t *ptOutput);
} foc_position_source_if_t;
```

- [ ] **Step 3: Implement validation and conversion helpers**

`motor_position.c` must provide focused internal functions for interface
validation, stepping, mechanical-to-electrical conversion, shortest signed angle
error, and blending. Do not place lifecycle state transitions in this file.

- [ ] **Step 4: Adapt existing observers and Hall**

Retain algorithm-specific `Init/Reset/Step` APIs. Add adapter constructors that
return `foc_position_source_if_t`. Delete `sensor_interface_t` and the
`observer_interface_t` alias after all references migrate; do not keep two
parallel public abstractions.

- [ ] **Step 5: Run both numeric suites**

Run `mingw32-make -C tests/foc clean all`.

Expected: encapsulation, float, and fixed tests PASS.

### Task 4: Add the command API and lifecycle FSM

**Files:**
- Create: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor.h`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`
- Modify: `tests/foc/Makefile`

- [ ] **Step 1: Write failing lifecycle tests**

Use fake time/calibration/PWM callbacks and assert:

- `motor_Start()` in IDLE accepts a valid command but does not enable PWM;
- `motor_RunFSM()` performs calibration before enable;
- calibration failure invokes emergency stop and enters FAULT;
- startup delay is non-blocking and returns `fsm_rt_on_going`;
- `motor_Stop()` requests STOPPING and FSM disables PWM before IDLE;
- a second start while STARTING or RUNNING returns a busy/invalid-state result;
- `motor_ClearFault()` fails unless PWM is off and state is FAULT.

- [ ] **Step 2: Add the minimal run configuration**

Define `motor_run_config_t` exactly as approved in the design. Validate the four
supported source combinations and reject two different non-null sources in the
first release.

- [ ] **Step 3: Add an internal one-command mailbox**

The private implementation stores command kind, copied run configuration, and
pending status. `motor_Start()` and `motor_Stop()` validate and enqueue intent;
they do not assign lifecycle states directly. Emergency stop remains immediate.

- [ ] **Step 4: Implement `motor_RunFSM()`**

Use `PERFC_PT_BEGIN/ENTRY/WAIT_UNTIL/DELAY_MS/YIELD/END` consistently with the
existing MODUS style. Keep the private startup phases:

```c
typedef enum {
    MOTOR_START_CALIBRATION = 0,
    MOTOR_START_DELAY,
    MOTOR_START_OPEN_LOOP,
    MOTOR_START_QUALIFY_SOURCE,
    MOTOR_START_BLEND_ANGLE,
    MOTOR_START_COMPLETE,
} motor_start_state_e;
```

The FSM owns lifecycle state, PWM enable/disable, calibration sequencing, and
transition timeout. It delegates realtime qualification/blending samples to the
high-frequency path.

- [ ] **Step 5: Run tests**

Run `mingw32-make -C tests/foc clean all`.

Expected: all lifecycle cases pass under float and fixed backends.

### Task 5: Move open-loop and cascaded control inside motor

**Files:**
- Modify: `foc/motor/motor_control_types.h`
- Modify: `foc/motor/motor_control.h`
- Modify: `foc/motor/motor_control.c`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor.c`
- Modify: `tests/foc/test_motor.c`

- [ ] **Step 1: Write failing control-path tests**

Assert these observable results through fake HAL plus `motor_GetSnapshot()`:

- voltage + open-loop angle produces changing duty and advancing angle;
- current + open-loop angle runs Id/Iq controllers instead of copying voltage;
- speed mode requires a valid speed controller and position source;
- position mode requires position and speed controllers plus mechanical position;
- high-frequency calls outside STARTING/RUNNING do not energize PWM;
- any sample/transform/modulation/set-duty error triggers emergency stop.

- [ ] **Step 2: Rename public control APIs and enum typedefs**

Replace `motor_ControlStart/Stop/Set*/HighFrequencyStep/LowFrequencyStep` with
the approved `motor_Start/Stop/Set*/HighFrequencyStep/LowFrequencyStep`. Remove
the old API rather than retaining undocumented aliases, because breaking the
member/API contract was explicitly approved.

- [ ] **Step 3: Implement deterministic high-frequency ordering**

Use this exact ownership order: reference snapshot, current sample, position
source update, angle choice/blend, Clarke/Park, current loop, inverse Park,
modulation, duty write, snapshot update. Advance open-loop angle from the
configured sample period, never from `get_system_ms()` in app code.

- [ ] **Step 4: Implement low-frequency cascades**

Position generates speed reference; speed generates Iq reference; current loops
remain high frequency. Reject missing controller bindings at `motor_Start()` so
the realtime path does not discover configuration errors after PWM enable.

- [ ] **Step 5: Run tests**

Run `mingw32-make -C tests/foc clean all`.

Expected: both backends pass and fake PWM remains disabled in invalid states.

### Task 6: Implement qualified and blended open-to-closed transfer

**Files:**
- Modify: `foc/motor/motor_position.c`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `foc/motor/motor_control.c`
- Modify: `foc/motor/motor_types.h`
- Modify: `tests/foc/test_motor_position.c`
- Modify: `tests/foc/test_motor.c`

- [ ] **Step 1: Write failing qualification tests**

Use a scripted fake source to verify that transition does not begin until valid,
fresh, confident, minimum-speed, direction-consistent, bounded-angle-error
samples persist for the configured consecutive count.

- [ ] **Step 2: Write failing wraparound blend tests**

Test `0.99 -> 0.01` and `0.01 -> 0.99`. Assert movement follows the shortest
signed circular path and does not rotate almost a full turn.

- [ ] **Step 3: Add bounded transition configuration**

Add qualification count, confidence threshold, minimum speed, maximum angle
error, blend sample count, and timeout to `motor_config_t` defaults. Keep these
library configuration values out of every `motor_Start()` call.

- [ ] **Step 4: Implement qualification and blend ownership**

The high-frequency path updates counters and blended angle. The FSM only starts,
completes, times out, or faults the transition. Source invalidation during blend
must emergency-stop in the first release; do not implement automatic fallback.

- [ ] **Step 5: Preserve control continuity**

Keep current references unchanged during angle transfer. If startup targets
speed/position, complete angle takeover first, then initialize the outer-loop
output to the current Iq before enabling that outer loop. Add a test that bounds
the one-sample Iq and duty delta.

- [ ] **Step 6: Run tests**

Run `mingw32-make -C tests/foc clean all`.

Expected: all qualification, timeout, wraparound, and continuity tests PASS.

### Task 7: Add stable diagnostics without exposing internals

**Files:**
- Modify: `foc/motor/motor_types.h`
- Modify: `foc/motor/motor_private.h`
- Modify: `foc/motor/motor.c`
- Modify: `foc/motor/motor_fsm.c`
- Modify: `tests/foc/test_motor.c`

- [ ] **Step 1: Write failing snapshot/event tests**

Assert that a snapshot exposes lifecycle state, startup phase, control mode,
active/candidate source validity, open/active/candidate angles, angle error,
blend factor, D/Q references/feedback, voltage, phase current, duty, calibration,
and faults. Assert an event is appended for command rejection, state transition,
source validity change, transition completion/timeout, and emergency stop.

- [ ] **Step 2: Add a fixed-size private event ring**

Use a compile-time capacity and overwrite-oldest policy. Store numeric event
records only: timestamp, from/to state, reason, and faults. Do not format strings
or call logging from realtime code.

- [ ] **Step 3: Complete `motor_GetSnapshot()`**

Copy one coherent snapshot through the configured `motor_sync_if_t`. Do not
return pointers to position outputs, controllers, calibration, HAL, or private
storage. Return `FOC_RESULT_INVALID_ARGUMENT` when concurrent target use is
configured but the synchronization interface is incomplete.

- [ ] **Step 4: Add the event-read API**

Add and test this API:

```c
bool motor_DebugReadEvent(motor_handle_t *ptMotor,
                          motor_event_t *ptEvent);
```

Test FIFO order and overwrite-oldest behavior. `foc_app` may drain events for
logs; normal control must never depend on consuming the ring.

- [ ] **Step 5: Run tests**

Run `mingw32-make -C tests/foc clean all` and expect PASS.

### Task 8: Migrate the FOC application and safe diagnostic path

**Files:**
- Modify: `foc/app/foc_app.h`
- Modify: `foc/app/foc_app.c`
- Modify: `foc/app/phase_test.c`
- Modify: target-specific FOC interrupt files only where they schedule high/low steps

- [ ] **Step 1: Add a compile check for forbidden app dependencies**

Before migration, use:

```powershell
rg -n "->tRt|->tControl|->tCurrent|->tParams|foc_ipark|foc_svpwm|motor_Enable|motor_SetDuty|motor_SampleCurrent" foc/app
```

Expected: current violations are listed. Save the command as a documented review
check; after migration it must return no matches except explicitly gated
diagnostic code.

- [ ] **Step 2: Reduce `foc_app_RunFSM()` to application orchestration**

It may submit product commands and return `motor_RunFSM(ptMotor)`. Remove open-loop
angle, calibration, PWM, current reconstruction, Park, modulation, and motor
state assignments.

- [ ] **Step 3: Convert button and Shell code**

Use `motor_GetSnapshot()` for decisions and printing. Use `motor_Start()` with a
single product-owned `motor_run_config_t`, `motor_Stop()`, and reference setters
for commands. Print command rejection and motor faults distinctly.

- [ ] **Step 4: Schedule realtime calls at their actual rates**

Ensure the PWM/ADC control interrupt calls `motor_HighFrequencyStep()` for its
own instance. Ensure a documented lower-rate scheduler calls
`motor_LowFrequencyStep()`. Do not run the high-frequency algorithm from the
main-loop FSM.

- [ ] **Step 5: Remove unsafe phase-test bypasses**

Disable direct fixed-duty tests from normal initialization. If hardware bring-up
still requires them, place the minimum code behind the existing experimental or
new diagnostic build flag, enforce IDLE/no-fault/timeout/limit checks, and access
hardware only through a narrow motor diagnostic API.

- [ ] **Step 6: Run forbidden-access and host checks**

Run the `rg` command from Step 1 and `mingw32-make -C tests/foc clean all`.

Expected: no app-layer member/control-algorithm violations; all host tests PASS.

### Task 9: Update the public documentation and extension notes

**Files:**
- Modify: `foc/README.md`
- Modify: `foc/foc.h`
- Review: `docs/superpowers/specs/2026-07-18-foc-motor-object-design.md`

- [ ] **Step 1: Replace every direct-member example**

Run:

```powershell
rg -n "\.tRt|\.tControl|\.tCurrent|\.tParams|motor_Control" foc/README.md
```

Replace each result with public API and snapshot usage. No example may rely on
private layout.

- [ ] **Step 2: Document four complete usage examples**

Provide compilable examples for D/Q voltage open loop, open-angle D/Q current
loop, sensored direct closed loop, and open-loop-to-observer transfer. For each,
show initialization, run config, main FSM call, high/low scheduler calls,
snapshot diagnostics, stop, and error handling.

- [ ] **Step 3: Document unified source adapters**

Explain how Hall, AB/ABZ, optical, magnetic, and observer implementations fill
valid flags; clarify mechanical-to-electrical conversion ownership and MDI/HAL
boundaries.

- [ ] **Step 4: Preserve bounded extension notes**

Document the trigger and implementation direction for flying start, position
manager, closed-loop degradation, runtime mode handover, online tuning, and
expanded diagnostic FSM. Do not add dormant enums, states, or APIs for them.

- [ ] **Step 5: Verify public header consistency**

Ensure `foc/foc.h` exports the new interfaces and no obsolete
`observer_lib.h`/old motor-control API. Re-run README searches and expect no
direct-member results.

### Task 10: Full verification

**Files:**
- Verify only; fix failures in the owning files from Tasks 1-9.

- [ ] **Step 1: Run formatting and stale-symbol checks**

```powershell
git diff --check
rg -n "motor_control_mode_t|motor_state_t|sensor_interface_t|observer_interface_t|motor_Control(Start|Stop|Set|High|Low)|->tRt|->tControl|->tCurrent" foc tests
```

Expected: no stale public names or app/test direct access. Matches in private
implementation are acceptable only for private fields with deliberately retained
names.

- [ ] **Step 2: Run the complete host matrix**

```powershell
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc all
```

Expected: encapsulation check, float suite, and fixed suite PASS.

- [ ] **Step 3: Run the default target build**

```powershell
.\make.bat clean
.\make.bat
```

Expected: debug build succeeds for the Makefile default target (`at32f407`).

- [ ] **Step 4: Build the FOC targets affected by adapters**

```powershell
.\make.bat clean TARGET_CHIP=at32f413
.\make.bat BUILD=debug TARGET_CHIP=at32f413 FOC_NUMERIC=float
.\make.bat clean TARGET_CHIP=stm32g431
.\make.bat BUILD=debug TARGET_CHIP=stm32g431 FOC_NUMERIC=float
```

Expected: both target builds succeed without vendor headers leaking into
business or FOC core code.

- [ ] **Step 5: Review the final diff against the design**

Confirm every public operation is API-based, state transitions are FSM-owned,
realtime paths are deterministic, the two-source limit is enforced, transition
failure defaults to stop, diagnostics remain readable, and deferred features
exist only as documented extension notes.
