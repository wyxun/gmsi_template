# AGENTS.md

Shared project instructions for AI coding agents.

## Build and Debug Commands

```powershell
.\make.bat                       # Default build (-Os, debug modules on)
.\make.bat clean                 # Clean build artifacts
.\make.bat flash                 # Flash via OpenOCD
.\make.bat rtt                   # RTT servers: shell 9090, waveform 9091
.\make.bat auto                  # Clean, default build, flash, start RTT
.\make.bat rttv                  # Open the RTT log viewer
mingw32-make BUILD=debug         # Unoptimized build (-O0)
mingw32-make BUILD=release       # Production build
mingw32-make TARGET_CHIP=<chip>  # Select a target explicitly
mingw32-make size                # Display ELF section sizes
```

`Makefile` and `make.bat` are authoritative. Available targets are the
subdirectories under `target/`; `Makefile` currently defaults to `at32f413`.

## AI Debugging

The authoritative procedure is `.agents/skills/aitrace/SKILL.md`.
Use `.\tools\aitrace.exe` as the single AI-facing debugging entry point.

- Start with passive RTT shell, waveform, or USB CDC capture.
- Obtain explicit confirmation before CPU halt, reset, or GDB operations.
- Build and flash only through commands defined by `Makefile` or `make.bat`.
- `tools/dev_debug.ps1` is deprecated; do not recreate or use it as the default.

## Architecture

```text
src/                  Application entry
class/                Business objects
peripheral/<chip>/    MDI hardware adaptation
peripheral/driver/    Chip-agnostic device drivers (e.g. AS5600 encoder)
foc/                  STM32G4 motor control
target/<chip>/        Target build and OpenOCD configuration
vendor/               Vendor libraries and submodules
modus/                MODUS framework submodule
tools/                AITrace, RTT viewer, and development utilities
```

Business code must access hardware through the MDI layer and must not include
vendor headers or call vendor HAL functions directly.

Use MODUS facilities instead of parallel local implementations:

- `MODUS_DECLARE_OBJECT` for object registration.
- `MODUS_SHELL_CMD` and mdebug for shell/logging support.
- `perf_counter` FSM and timing facilities for delays and cycle conversion.

## Coding Conventions

- Prefer high cohesion and low coupling.
- Keep target-specific code inside `peripheral/<chip>/` or `target/<chip>/`.
- Functions use `module_Action()` naming; types use the `_t` suffix.
- Macros use uppercase names.
- Keep lines within 86 characters where practical.
- Preserve unrelated worktree and submodule changes.
- Do not stage, commit, switch branches, or push unless explicitly requested.
