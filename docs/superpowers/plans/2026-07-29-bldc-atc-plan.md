# BLDC Spindle Mechanical Socket ATC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a BLDC spindle motor reverse-rotation mechanical socket ATC system with FG pulse closed-loop sensing and fallback delay mode on the AT32F407 grblHAL controller.

**Architecture:** Update `grblhal_driver.h` and `grblhal_motion.c` to drive BLDC Enable, Direction, PWM, and capture FG pulse inputs. Refactor `atc_plugin.c` to replace pneumatic solenoid drawbar calls with low-RPM CCW (unscrew/release) and CW (screw/lock) spindle state transitions, governed by FG pulse feedback or fallback timing.

**Tech Stack:** C (AT32F407 BSP / Standard Peripheral Library), grblHAL Core API, MODUS framework, mingw32-make / make.bat.

---

## File Structure

- Modify: `grblhal_adapt/grblhal_driver.h` - Pin mapping for `SPINDLE_DIR` (PA5), `SPINDLE_EN` (PB0), `SPINDLE_FG` (PB1), `SPINDLE_ALARM` (PC5).
- Modify: `grblhal_adapt/grblhal_motion.c` - GPIO initialization, FG pulse interrupt/counter, `dc_spindle_set_state` updates for EN and DIR pins.
- Modify: `grblhal_adapt/atc_plugin.h` - Structure definition for updated ATC settings `$900` ~ `$909`.
- Modify: `grblhal_adapt/atc_plugin.c` - Rewritten ATC state machine implementing CCW unclamp & CW clamp with FG pulse sensing.

---

### Task 1: Update Hardware Pin Mapping and Spindle Driver (`grblhal_driver.h` & `grblhal_motion.c`)

**Files:**
- Modify: `grblhal_adapt/grblhal_driver.h`
- Modify: `grblhal_adapt/grblhal_motion.c`

- [ ] **Step 1: Add Spindle EN, DIR, FG, and ALARM Pin Definitions to `grblhal_driver.h`**

```c
/* Spindle & BLDC Control Pins */
#define SPINDLE_PWM_PORT        GPIOA
#define SPINDLE_PWM_PIN         GPIO_PINS_6
#define SPINDLE_DIR_PORT        GPIOA
#define SPINDLE_DIR_PIN         GPIO_PINS_5
#define SPINDLE_EN_PORT         GPIOB
#define SPINDLE_EN_PIN          GPIO_PINS_0
#define SPINDLE_FG_PORT         GPIOB
#define SPINDLE_FG_PIN          GPIO_PINS_1
#define SPINDLE_ALARM_PORT      GPIOC
#define SPINDLE_ALARM_PIN       GPIO_PINS_5
```

- [ ] **Step 2: Add FG Pulse Tracker & Control Helpers to `grblhal_motion.c`**

Add global pulse counter variables and EXTI / GPIO init in `grblhal_motion.c`:

```c
static volatile uint32_t s_wFgPulseCount = 0;
static volatile uint32_t s_wLastFgPulseTicks = 0;

uint32_t grblhal_spindle_get_fg_count(void) {
    return s_wFgPulseCount;
}

void grblhal_spindle_reset_fg_count(void) {
    s_wFgPulseCount = 0;
    s_wLastFgPulseTicks = grblhal_get_ticks();
}

uint32_t grblhal_spindle_get_fg_idle_time_ms(void) {
    return grblhal_get_ticks() - s_wLastFgPulseTicks;
}

bool grblhal_spindle_get_alarm(void) {
    return gpio_input_data_bit_read(SPINDLE_ALARM_PORT, SPINDLE_ALARM_PIN) == RESET;
}
```

- [ ] **Step 3: Update `dc_spindle_set_state` to Control EN and DIR Pins**

```c
static void dc_spindle_set_state(spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    s_tSpindleState.value = 0;
    s_tSpindleState.on = state.on;
    s_tSpindleState.ccw = state.ccw;

    /* PA5 (DIR): LOW for CW, HIGH for CCW */
    gpio_bits_write(SPINDLE_DIR_PORT, SPINDLE_DIR_PIN, (state.on && state.ccw) ? TRUE : FALSE);
    
    /* PB0 (EN): HIGH when enabled, LOW when stopped */
    gpio_bits_write(SPINDLE_EN_PORT, SPINDLE_EN_PIN, state.on ? TRUE : FALSE);

    /* PA6 (PWM): Output PWM duty cycle */
    pwm_spindle_update_pwm(spindle, state.on ? spindle->get_pwm(spindle, rpm) : 0);
}
```

- [ ] **Step 4: Configure GPIO for EN, DIR, FG, and ALARM in `grblhal_spindle_config`**

Configure GPIO modes for PA5 (DIR Out), PB0 (EN Out), PB1 (FG In / EXTI), PC5 (ALARM In Pull-up).

- [ ] **Step 5: Verify Build**

Run: `.\make.bat`
Expected: Build succeeds with 0 errors.

- [ ] **Step 6: Commit Task 1**

```bash
git add grblhal_adapt/grblhal_driver.h grblhal_adapt/grblhal_motion.c
git commit -m "feat(spindle): add BLDC DIR, EN, FG, ALARM GPIO control and pulse sensing"
```

---

### Task 2: Update ATC Plugin Settings and State Machine (`atc_plugin.h` & `atc_plugin.c`)

**Files:**
- Modify: `grblhal_adapt/atc_plugin.h`
- Modify: `grblhal_adapt/atc_plugin.c`

- [ ] **Step 1: Update `atc_settings_t` Data Structure in `atc_plugin.h`**

```c
typedef struct {
    uint8_t n_pockets;
    float pocket_pitch;
    float rack_origin_x;
    float rack_origin_y;
    float z_clear_height;
    float z_pickup_depth;
    float spindle_rpm;      /* $906: RPM during clamp/unclamp (default 500) */
    uint16_t timeout_ms;    /* $907: Timeout/delay in ms (default 2000) */
    uint16_t fg_target;     /* $908: Target FG pulses for unclamp (default 30) */
    union {
        uint8_t value;
        struct {
            uint8_t use_fg       :1, /* Bit 0: 1=FG closed loop, 0=Timeout delay */
                    restore_xy   :1, /* Bit 1: 1=Restore XY position after tool change */
                    unused       :6;
        };
    } flags;                /* $909: ATC Mode Flags */
} atc_settings_t;
```

- [ ] **Step 2: Register Settings `$900` - `$909` in `atc_plugin.c`**

Map settings `$900` to `$909` in `atc_settings[]` table with appropriate getters, setters, ranges, and formats.

- [ ] **Step 3: Implement `atc_unclamp_spindle()` and `atc_clamp_spindle()`**

Replace `atc_drawbar()` and `atc_air_blast()` with:

```c
static bool atc_unclamp_spindle(void)
{
    /* Check spindle alarm */
    if (grblhal_spindle_get_alarm()) return false;

    spindle_state_t state = { .on = 1, .ccw = 1 };
    hal.spindle.set_state(hal.spindle.get_context(), state, s_atc.spindle_rpm);
    grblhal_spindle_reset_fg_count();

    uint32_t start_ticks = grblhal_get_ticks();
    bool ok = true;

    if (s_atc.flags.use_fg) {
        /* FG Mode: Wait until target pulses reached or timeout */
        while (grblhal_spindle_get_fg_count() < s_atc.fg_target) {
            if (grblhal_get_ticks() - start_ticks > s_atc.timeout_ms) {
                ok = false; break;
            }
            if (grblhal_spindle_get_alarm()) { ok = false; break; }
            grblhal_delay_ms(10, NULL);
        }
    } else {
        /* Fallback delay mode */
        grblhal_delay_ms(s_atc.timeout_ms, NULL);
    }

    spindle_all_off(false);
    return ok;
}

static bool atc_clamp_spindle(void)
{
    if (grblhal_spindle_get_alarm()) return false;

    spindle_state_t state = { .on = 1, .ccw = 0 };
    hal.spindle.set_state(hal.spindle.get_context(), state, s_atc.spindle_rpm);
    grblhal_spindle_reset_fg_count();

    uint32_t start_ticks = grblhal_get_ticks();
    bool ok = true;

    if (s_atc.flags.use_fg) {
        /* FG Mode: Wait for initial pulse then wait for stall (pulse idle > 200ms) */
        grblhal_delay_ms(100, NULL);
        while (grblhal_spindle_get_fg_idle_time_ms() < 200) {
            if (grblhal_get_ticks() - start_ticks > s_atc.timeout_ms) {
                ok = false; break;
            }
            if (grblhal_spindle_get_alarm()) { ok = false; break; }
            grblhal_delay_ms(10, NULL);
        }
    } else {
        /* Fallback delay mode */
        grblhal_delay_ms(s_atc.timeout_ms, NULL);
    }

    spindle_all_off(false);
    return ok;
}
```

- [ ] **Step 4: Update `atc_tool_change` to Use Unclamp & Clamp Helper Functions**

Replace step 3 (return old tool) and step 4 (pickup new tool) pneumatic drawbar logic in `atc_tool_change` with `atc_unclamp_spindle()` and `atc_clamp_spindle()`.

- [ ] **Step 5: Verify Build**

Run: `.\make.bat`
Expected: Build succeeds with 0 errors.

- [ ] **Step 6: Commit Task 2**

```bash
git add grblhal_adapt/atc_plugin.h grblhal_adapt/atc_plugin.c
git commit -m "feat(atc): refactor ATC tool change state machine for BLDC reverse unclamp and clamp"
```

---

### Task 3: Full Project Build & Section Verification

**Files:** None

- [ ] **Step 1: Perform Clean Build**

Run: `.\make.bat clean`
Run: `.\make.bat`
Expected: Clean build compiles all modules into `build/template.elf` without warnings or errors.

- [ ] **Step 2: Check ELF Sizes**

Run: `mingw32-make size`
Expected: Display ELF memory footprints.

- [ ] **Step 3: Commit Final Verification**

```bash
git commit --allow-empty -m "build(atc): verify full build of BLDC mechanical socket ATC feature"
```
