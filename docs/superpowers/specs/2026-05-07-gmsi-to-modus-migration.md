# Design Spec: Migration from GMSI to MODUS

## 1. Overview
The `gmsi` library has been rebranded to `modus`. This migration involves updating the project to use the new naming conventions, directory structure, and the provided `modus.mk` build system integration.

## 2. Infrastructure Changes

### 2.1 Directory Structure
- The `gmsi/` submodule has been renamed/moved to `modus/`.
- Core library source files are now located in `modus/src/`.

### 2.2 Build System (Makefile)
- Define `MODUS_ROOT = modus`.
- Enable the following modules in `makefile`:
  - `MSHELL_ENABLE = 1`
  - `MSTORAGE_ENABLE = 1`
  - `MBLINFO_ENABLE = 1`
  - `MWAVEFORM_ENABLE = 1`
  - `MODUS_USE_LOG = 1` (conditional on build mode)
- Include `$(MODUS_ROOT)/modus.mk`.
- Remove manual listing of GMSI source files and include paths.
- Add `$(MODUS_SRCS)` to `C_SOURCES`.
- Add `$(MODUS_INCLUDES)` to `C_INCLUDES`.
- Add `$(MODUS_CFLAGS)` to `CFLAGS`.

## 3. Code Changes (Global Search & Replace)

### 3.1 Macros and Constants
- `GMSI_` -> `MODUS_`
- `GBASE_` -> `MBASE_`
- `GDI_` -> `MDI_`
- `GLOG` -> `MLOG`
- `GLOGF` -> `MLOGF`

### 3.2 Types and Functions
- `gmsi_t` -> `modus_t`
- `gmsi_Init` -> `modus_Init`
- `gmsi_Run` -> `modus_Run`
- `gmsi_Clock` -> `modus_Clock`
- `gstorage_` -> `mstorage_`
- `gshell_` -> `mshell_`
- `gwaveform_` -> `mwaveform_`
- `gblinfo_` -> `mblinfo_`
- `gcoroutine_` -> `mcoroutine_`
- `gringbuf_` -> `mringbuf_`

### 3.3 Headers
- `gmsi.h` -> `modus.h`
- `gstorage.h` -> `mstorage.h`
- `gdi/gdi.h` -> `mdi/mdi.h`
- `gdebug/gshell.h` -> `mdebug/mshell.h`
- `gdebug/gwaveform.h` -> `mdebug/mwaveform.h`

## 4. Porting and Peripheral Layer
- Update `peripheral/stm32g431/port_gdi.c` (or rename to `port_mdi.c`) internal references.
- Update `#include "gdi_hw.h"` references to `#include "mdi_hw.h"`.

## 5. Verification Plan
1. `make clean`
2. `make` (build for default target)
3. Verify compilation success.
4. (Optional) Run `autotest` workflow if hardware is available.
