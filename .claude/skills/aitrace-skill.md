# AITrace Debug Skill

Use `aitrace.exe` (at `./tools/aitrace.exe`) to debug MCU firmware.

## Prerequisites

OpenOCD must be running with RTT enabled. Start from modus_template root:
```powershell
.\make.bat rtt
```

This exposes TCP 9090 (RTT Shell/Logs), TCP 9091 (Waveform), TCP 4444 (OpenOCD Telnet).

## Intrusion Levels

**Default: Passive.** Always try this first.

| Level | Method | CPU Impact | Confirmation Needed |
|-------|--------|------------|---------------------|
| A. Passive | `aitrace shell ...`, `aitrace wave ...` | None | No |
| B. Halt | `aitrace ocd halt/regs/peek/mdw/stack` | Paused briefly | **Yes** |
| C. GDB | `aitrace gdb connect/break/step` | Full debug control | **Yes** |

## Core Commands

### Passive (always safe)

```powershell
aitrace shell regs / peek <addr> / stack [n] / cfsr / list
aitrace shell log -E -W -I -D
aitrace wave capture 5                  # 5s CSV to stdout
aitrace wave capture 10 --output w.csv  # To file
aitrace wave start / stop / rate <n>
aitrace serial --port COM3 [--baud 115200] [--duration 5] [--hex]  # Listen to serial port (passive, safe)
```

### Halt-Based (requires confirmation)

```powershell
aitrace ocd halt / resume / regs
aitrace ocd peek <addr> / mdw <addr> [n] / stack [n]
```

### GDB (requires confirmation)

```powershell
aitrace gdb connect --elf build/template.elf
aitrace gdb break main.c:100
aitrace gdb continue / step / bt
aitrace gdb print g_wTickCounter
aitrace gdb detach
```

### Analysis

```powershell
aitrace map resolve build/template.elf 0x08001234 0x08005678
aitrace map info build/template.map
aitrace crash report --pc=0x... --lr=0x... --sp=0x... --elf=build/template.elf
```

## Workflows

### HardFault Analysis

1. Observe RTT Ch0 output — firmware auto-dumps exception frame + CFSR on fault
2. Extract PC, LR, SP from the dump
3. `aitrace crash report --pc=<PC> --lr=<LR> --sp=<SP> --elf=build/template.elf`
4. Interpret: PC = faulting instruction, LR = caller return address, CFSR = fault type

### Runtime Behavior Analysis

1. `aitrace wave capture 5 > wave.csv` — capture waveform
2. `aitrace shell regs` + `aitrace shell list` — check MCU state
3. Analyze CSV: look for anomalies, compare with expected ranges

### Serial Port Capturing & Interactive Transmit (Passive, safe)

Use `aitrace serial` to write commands to the MCU and monitor logs/feedback in a closed loop:
- Read-only capturing: `.\tools\aitrace.exe serial --port COM3 --baud 115200 --duration 5`
- Interactive Closed Loop (Send -> Listen -> Exit): 
  `.\tools\aitrace.exe serial --port COM3 --send "help" --duration 3`
  `.\tools\aitrace.exe serial --port COM3 --send-hex "A5 5A 01" --duration 2 --hex`
- Supports `--hex` to print hex, and `--ascii` to print clean text (default).

### Variable Inspection

- Passive: `aitrace shell peek <addr>`
- Halt: `aitrace ocd halt` → `aitrace ocd peek <addr>` → `aitrace ocd resume` (requires confirmation)
- GDB: `aitrace gdb connect --elf build/template.elf` → `aitrace gdb print <var>` (requires confirmation)

## Safety Rules

- ALWAYS prefer passive commands first — the MCU may be driving a motor/power stage
- Before halt/resume or GDB: explain WHY to the engineer, get confirmation, remind them it interrupts real-time control
- Never modify source code or re-flash firmware without confirmation

## Double-Track Debugging Strategy

To maximize debugging safety and efficiency, follow the **Double-Track Debugging** workflow:

| Scenario | Mode | Compiler Optimization | Recommended Tools | Advantages |
| :--- | :--- | :--- | :--- | :--- |
| **Logic & Code Flow** | `.\make.bat` (`BUILD=debug`) | `-O0` (No optimization) | VS Code Graphical F5 (Cortex-Debug) | Precise step-by-step debug, values of local variables are 100% visible (no `<optimized out>`). |
| **Tuning, Waveforms & Timing** | `.\make.bat auto` (`BUILD=debug-rel`) | `-Oz` (Strict size/speed opt) | `mstudio` / `aitrace` (Passive wave/shell) | 100% identical physical timing to the release build, preventing FOC MOS-blown risks due to timing jitter. |

## FAQ & Troubleshooting

### 1. `Failed to write memory` or `can't assert SRST`
If OpenOCD fails to flash or halt the CPU, the SWD debug pins might be disabled by firmware or entering low-power modes.
- **Solution**: Press and **hold** the physical **RESET button** on the MCU board, execute `.\make.bat auto` (or aitrace flash command), and **release** the RESET button the exact moment the terminal prints `CMSIS-DAP: Interface ready`.

### 2. How to use `.\make.bat auto` Workflow
The `.\make.bat auto` command automates the entire loop:
1. Performs `make clean`.
2. Compiles the firmware with `BUILD=debug-rel` (`-Oz`).
3. Flashes the firmware into the MCU.
4. Spawns an OpenOCD RTT server in the background, mapping RTT Ch1 Waveform data to `127.0.0.1:9091` automatically.
- **Result**: Once finished, you do **NOT** need to run any extra commands. Simply open **`mstudio`** and connect to `127.0.0.1:9091` to view FOC waveforms.

