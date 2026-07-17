# AITrace Debug Skill

This file is the project's authoritative MCU debugging procedure. Use
`.\tools\aitrace.exe` as the single AI-facing debug entry point. Do not use
`tools/dev_debug.ps1` except when explicitly requested for legacy compatibility.

All commands are run from the repository root. Build modes and flashing commands
remain defined by the root `Makefile` and `make.bat`; this skill does not duplicate
their implementation.

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
.\tools\aitrace.exe shell regs / peek <addr> / stack [n] / cfsr / list
.\tools\aitrace.exe shell log -E -W -I -D
.\tools\aitrace.exe wave capture 5                  # 5s CSV to stdout
.\tools\aitrace.exe wave capture 10 --output w.csv  # To file
.\tools\aitrace.exe wave start / stop / rate <n>
.\tools\aitrace.exe serial --port COM3 --baud 115200 --duration 5
```

### Halt-Based (requires confirmation)

```powershell
.\tools\aitrace.exe ocd halt / resume / regs
.\tools\aitrace.exe ocd peek <addr> / mdw <addr> [n] / stack [n]
```

### GDB (requires confirmation)

```powershell
.\tools\aitrace.exe gdb connect --elf build/template.elf
.\tools\aitrace.exe gdb break main.c:100
.\tools\aitrace.exe gdb continue / step / bt
.\tools\aitrace.exe gdb print g_wTickCounter
.\tools\aitrace.exe gdb detach
```

### Analysis

```powershell
.\tools\aitrace.exe map resolve build/template.elf 0x08001234 0x08005678
.\tools\aitrace.exe map info build/template.map
.\tools\aitrace.exe crash report --pc=0x... --lr=0x... --sp=0x... --elf=build/template.elf
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
| **Tuning, Waveforms & Timing** | `mingw32-make BUILD=debug-rel` followed by the required flash/RTT commands | `-Oz` (Strict size/speed opt) | `mstudio` / `aitrace` (Passive wave/shell) | Release-like code generation while retaining debug modules. |

## FAQ & Troubleshooting

### 1. `Failed to write memory` or `can't assert SRST`
If OpenOCD fails to flash or halt the CPU, the SWD debug pins might be disabled by firmware or entering low-power modes.
- **Solution**: Press and **hold** the physical **RESET button** on the MCU board, execute `.\make.bat auto`, and **release** the RESET button when the terminal prints `CMSIS-DAP: Interface ready`.

### 2. How to use `.\make.bat auto` Workflow
The `.\make.bat auto` command automates the entire loop:
1. Performs `make clean`.
2. Compiles the firmware with `BUILD=debug` (`-O0`).
3. Flashes the firmware into the MCU.
4. Spawns an OpenOCD RTT server in the background, mapping RTT Ch1 Waveform data to `127.0.0.1:9091` automatically.
- **Result**: Once finished, you do **NOT** need to run any extra commands. Simply open **`mstudio`** and connect to `127.0.0.1:9091` to view FOC waveforms.

