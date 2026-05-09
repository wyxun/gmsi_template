# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Debug Commands

```powershell
.\make.bat                       # build (debug mode, -O0)
.\make.bat clean                 # clean build artifacts
.\make.bat flash                 # flash via OpenOCD
.\make.bat rtt                   # start OpenOCD RTT server (shell=9090, waveform=9091)
.\make.bat auto                  # clean -> build(debug-rel) -> flash -> rtt (AI default)
mingw32-make BUILD=debug-rel   # release-close debug (-Oz, debug modules on)
mingw32-make BUILD=release     # production build (-Oz, debug modules stripped)
mingw32-make TARGET_CHIP=stm32g431
mingw32-make TARGET_CHIP=at32f421
mingw32-make size                # show ELF section sizes
```

### Build Modes

| Mode | Command | Opt | -g | MSHELL | MWAVEFORM | MLOG | Use Case |
|------|---------|-----|-----|--------|-----------|------|----------|
| **debug** | `make` | -O0 | yes | on | on | on | Daily development, breakpoint-friendly |
| **debug-rel** | `make debug-rel` | -Oz | yes | on | on | on | AI debugging, matches release codegen |
| **release** | `make release` | -Oz | yes | off | off | off | Production firmware (bin stripped) |

### AI Debugging Workflow

```
.\make.bat auto                              # 1. Build(debug-rel) + flash + RTT
.\tools\dev_debug.ps1 halt                    # 2. Halt CPU when anomaly detected
.\tools\dev_debug.ps1 regs                    # 3. Read PC/SP/LR/core regs
.\tools\dev_debug.ps1 stack 32               # 4. Dump stack region
.\tools\dev_debug.ps1 var build/template.elf g_wTickCounter  # 5. Read specific variable
.\tools\dev_debug.ps1 resume                  # 6. Resume execution
```

### Firmware Shell Commands (RTT/mshell)

| Command | Description |
|---------|-------------|
| `help` | List all registered commands |
| `regs` | Dump MSP, PSP, CONTROL, PRIMASK, FAULTMASK, BASEPRI, LR |
| `peek <hex_addr>` | Read uint32 at memory address |
| `poke <hex_addr> <hex_val>` | Write uint32 to memory address |
| `stack [depth]` | Dump stack memory around current SP, mark likely LR/PC |
| `cfsr` | Show fault status registers (CFSR/HFSR/MMAR/BFAR) with decode |
| `log [-E][-W][-I][-D]` | Toggle MLOG levels at runtime |
| `wave start\|stop\|rate\|list\|drop` | Control real-time waveform |
| `list` | List all MODUS objects and event masks |
| `post <id_hex> <event_hex>` | Post event to an object |
| `ver` | Show MODUS version |

### dev_debug.ps1 (OpenOCD telnet wrapper)

```
.\tools\dev_debug.ps1 halt           # Halt CPU
.\tools\dev_debug.ps1 resume         # Resume CPU
.\tools\dev_debug.ps1 reset          # Reset MCU and halt
.\tools\dev_debug.ps1 regs           # Dump all core registers
.\tools\dev_debug.ps1 peek <addr>    # Read uint32 at address
.\tools\dev_debug.ps1 mdw <addr> [n] # Dump n words
.\tools\dev_debug.ps1 stack [n]      # Dump stack around SP
.\tools\dev_debug.ps1 disasm <addr> [n]  # Disassemble
.\tools\dev_debug.ps1 var <elf> <name>   # Read global/static variable value
```

### HardFault Auto-Dump

Fault handlers (`HardFault_Handler`, `MemManage_Handler`, `BusFault_Handler`, `UsageFault_Handler`) in `vendor/cortex-m/core_debug/core_debug_cm_fault.c` override the weak defaults. On any fault, the stacked exception frame (R0-R3, R12, LR, PC, xPSR) and fault status registers are printed via RTT automatically — **no debugger needed**.

To disable and use custom handlers, define `CORE_DEBUG_OVERRIDE_FAULT_HANDLER` in userconfig.h.

## Architecture

**Dual-target bare-metal template** for ARM Cortex-M4, supporting STM32G431 (with FPU) and AT32F421 (no FPU).

### Layer Stack

```
src/main.c                 — App entry, calls peripheral_Init() + modus_Init()
class/                     — Business logic objects, OOP-C style
peripheral/                — MDI adaptation layer ("moat")
  stm32g431/port_mdi.c     — STM32G4 MDI implementations
  at32f421/port_mdi.c      — AT32F421 MDI implementations
foc/                       — FOC motor control (STM32G4 only)
vendor/                    — Third-party vendor SDKs (git submodules)
  cortex-m/
    cmsis_core/            — ARM CMSIS Core headers
    cmsis_device_g4/       — STM32G4 CMSIS device headers
    stm32g4xx_hal_driver/  — STM32G4 HAL/LL drivers
    AT32F421_Firmware_Library/ — AT32F421 standard peripheral library
    core_debug/            — Cortex-M on-chip debug (regs/peek/poke/stack/fault)
  riscv/                   — Reserved for RISC-V
modus/                     — MODUS framework (git submodule)
tools/
  mstudio/                 — PC debug workbench (waveform + shell + regs + vars + map)
  dev_debug.ps1            — OpenOCD telnet debug wrapper
```

### Key Patterns

- **MDI (Modular Device Interface)**: All hardware access goes through `HW` global in `mdi_hw.h` using `MDI_Read/Write/Toggle` macros. Business code MUST NOT include vendor headers or call vendor HAL directly.
- **MODUS Object Auto-Registration**: Use `MODUS_DECLARE_OBJECT(type, name, ...)` in any .c file — the object is automatically discovered and initialized by `modus_Init()` via the `init_infos` linker section. No manual registration in `main.c`.
- **Millisecond Co-routines**: Use `perf_counter` FSM macros (`PERFC_PT_BEGIN/END`, `PERFC_PT_DELAY_MS`, `PERFC_PT_WAIT_UNTIL`) instead of raw `switch-case` state machines.
- **Shell Commands**: Use `MODUS_SHELL_CMD(name, handler, help_str)` anywhere in any .c file — automatically registered.

### Adding a New Target Chip

1. Create `peripheral/<chip>/port_mdi.c`, `port_mdi.h`, `mdi_hw.h`
2. Create `target/<chip>/target.mk` with CPU flags, linker script, vendor sources
3. Build with `mingw32-make TARGET_CHIP=<chip>`

### Naming Convention

- Variables prefixed by type: `ch`=uint8_t, `hw`=uint16_t, `w`=uint32_t, `f`=float, `b`=bool, `pt`=struct pointer
- Static vars: `s_` prefix, Global vars: `g_` prefix
- Functions: `module_Action()` style (e.g., `template_Init`, `mbase_EventPend`)
- Types: `_t` suffix, all lowercase with underscores
- Macros: `UPPER_CASE`
- Line width limit: 86 characters
