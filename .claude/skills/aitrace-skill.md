# AITrace Debug Skill

Use `aitrace.exe` (at `E:\Project\mstudio\aitrace\aitrace.exe`) to debug MCU firmware.

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

### Variable Inspection

- Passive: `aitrace shell peek <addr>`
- Halt: `aitrace ocd halt` → `aitrace ocd peek <addr>` → `aitrace ocd resume` (requires confirmation)
- GDB: `aitrace gdb connect --elf build/template.elf` → `aitrace gdb print <var>` (requires confirmation)

## Safety Rules

- ALWAYS prefer passive commands first — the MCU may be driving a motor/power stage
- Before halt/resume or GDB: explain WHY to the engineer, get confirmation, remind them it interrupts real-time control
- Never modify source code or re-flash firmware without confirmation
