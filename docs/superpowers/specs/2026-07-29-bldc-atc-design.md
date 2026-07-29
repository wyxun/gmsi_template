# BLDC Spindle Mechanical Socket ATC Specification

## Overview
This specification details the design for replacing the legacy pneumatic drawbar ATC scheme with a Brushless DC (BLDC) Spindle Motor reverse-rotation mechanical socket ATC scheme on the AT32F407 grblHAL controller.

The tool changing process is accomplished by lowering the spindle into a fixed mechanical wrench socket/pocket that restrains the toolholder nut, then running the spindle motor in CCW (reverse) at controlled low RPM to unscrew/release the tool, or in CW (forward) at low RPM to screw in and lock the toolholder.

The system features **FG Pulse Closed-Loop Sensing** as the primary mode (detecting rotation pulses and stall/tightening condition), with a **Fallback Delay Mode** for operating without FG pulse feedback.

---

## Hardware IO Pin Mapping

The BLDC motor driver interface is mapped to the MCU (AT32F407) as follows:

| Signal Name | Pin Assignment | Direction | Description |
| :--- | :--- | :--- | :--- |
| `SPINDLE_PWM` | `PA6` (TMR3_CH1) | Output | PWM output for 0-10V analog speed control (via external RC filter & op-amp) |
| `SPINDLE_DIR` | `PA5` | Output | Spindle Direction control (`LOW` = CW Forward, `HIGH` = CCW Reverse) |
| `SPINDLE_EN` | `PB0` | Output | Spindle Driver Enable/Run (`HIGH` = Enabled, `LOW` = Disabled) |
| `SPINDLE_FG` | `PB1` (EXTI Line 1) | Input | Tachometer / FG pulse input for RPM measurement and stall detection |
| `SPINDLE_ALARM` | `PC5` | Input | Driver Fault/Alarm input (Active `LOW`) |

---

## Architecture & Tool Change State Machine (`atc_plugin.c`)

When `M6 T<n>` is triggered, the `atc_tool_change` state machine executes:

### 1. Tool Unclamp (CCW Reverse Unscrewing)
1. Spindle moves in Z axis down into the fixed mechanical wrench socket.
2. MCU sets `SPINDLE_DIR` to CCW (`HIGH`) and enables spindle at `ATC Spindle RPM` (Settings `$906`, default 500 RPM).
3. **FG Mode Enabled (`$909` Bit 0 = 1)**:
   - MCU counts incoming FG pulses on `PB1`.
   - When pulse count reaches target unscrew pulses (`$908`), or pulse rate drops indicating free disengagement, spindle is stopped.
4. **Fallback Mode (`$909` Bit 0 = 0)**:
   - Spindle runs CCW for specified `ATC Timeout` duration (`$907`, default 2000 ms), then stops.
5. Z axis raises to clear height, leaving the old tool in the socket.

### 2. Tool Clamp (CW Forward Screwing & Tightening)
1. Spindle moves to new tool pocket slot, Z axis lowers to engage new tool shank.
2. MCU sets `SPINDLE_DIR` to CW (`LOW`) and enables spindle at `ATC Spindle RPM` (`$906`).
3. **FG Mode Enabled (`$909` Bit 0 = 1)**:
   - MCU monitors FG pulse interval on `PB1`.
   - As the thread tightens, rotation slows down and stops. When no FG pulse is received for > 200 ms (motor stall / thread fully torqued), lock is confirmed and spindle is immediately disabled.
4. **Fallback Mode (`$909` Bit 0 = 0)**:
   - Spindle runs CW for specified `ATC Timeout` duration (`$907`), then stops.
5. Z axis raises to clear height, completing tool change.

---

## Configuration Settings ($900 - $909)

| Setting ID | Name | Unit | Default | Description |
| :--- | :--- | :--- | :--- | :--- |
| `$900` | ATC Pockets | - | 4 | Number of tool pockets |
| `$901` | ATC Pocket Pitch | mm | 50.0 | Pitch distance between tool pockets |
| `$902` | ATC Rack Origin X | mm | 0.0 | X coordinate of Pocket 1 |
| `$903` | ATC Rack Origin Y | mm | 0.0 | Y coordinate of Pocket 1 |
| `$904` | ATC Z Clear | mm | 50.0 | Z axis safe clear height |
| `$905` | ATC Z Pickup | mm | -10.0 | Z axis depth for socket engagement |
| `$906` | ATC Spindle RPM | RPM | 500 | Spindle speed during clamp / unclamp |
| `$907` | ATC Timeout Delay | ms | 2000 | Timeout / delay for clamp and unclamp operations |
| `$908` | ATC FG Target Pulses | - | 30 | Target FG pulses for unscrewing (0 = automatic disengage) |
| `$909` | ATC Options Bitfield | - | 1 | Bit 0: Enable FG Closed Loop (1=FG, 0=Timeout Delay)<br>Bit 1: Restore XY Position after tool change |

---

## Safety & Error Handling

- **Spindle Alarm Input (`PC5`)**: If `SPINDLE_ALARM` triggers during tool change (e.g. driver over-current), `atc_tool_change` immediately halts spindle and raises a G-code Tool Error (`Status_GCodeToolError`).
- **Timeout Protection**: If FG mode is active but motor fails to reach target pulses or stall within `$907` timeout, tool change aborts safely.
