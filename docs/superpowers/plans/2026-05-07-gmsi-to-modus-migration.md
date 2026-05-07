# GMSI to MODUS Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the project from legacy `gmsi` to the new `modus` framework.

**Architecture:** Use `modus.mk` for build system integration and perform global API renaming.

**Tech Stack:** C, Make, MODUS Framework.

---

### Task 1: Makefile Refactoring

**Files:**
- Modify: `makefile`

- [ ] **Step 1: Define MODUS_ROOT and include modus.mk**
Remove old GMSI variables and source lists.
```makefile
MODUS_ROOT = modus
MSHELL_ENABLE    = 1
MWAVEFORM_ENABLE = 1
MSTORAGE_ENABLE  = 1
MBLINFO_ENABLE   = 1
MODUS_USE_LOG    = 1
include $(MODUS_ROOT)/modus.mk

# Replace GMSI_SOURCES with MODUS_SRCS in C_SOURCES
# Replace GMSI_INCLUDES with MODUS_INCLUDES in C_INCLUDES
# Add MODUS_CFLAGS to CFLAGS
```

- [ ] **Step 2: Update Defines**
```diff
- -DGMSI_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""
+ -DMODUS_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""
```

- [ ] **Step 3: Commit Makefile changes**
`git add makefile; git commit -m "build: refactor makefile to use modus.mk"`

### Task 2: Global API Renaming (Core)

**Files:**
- Modify: `src/main.c`, `peripheral/peripheral.h`, `peripheral/stm32g431/port_gdi.c`

- [ ] **Step 1: Replace Headers and Types**
  - `gmsi.h` -> `modus.h`
  - `gmsi_t` -> `modus_t`
  - `gmsi_Init` -> `modus_Init`
  - `gmsi_Run` -> `modus_Run`
  - `gmsi_Clock` -> `modus_Clock`

- [ ] **Step 2: Replace Log Macros**
  - `GLOG` -> `MLOG`
  - `GLOGF` -> `MLOGF`

- [ ] **Step 3: Commit Core Changes**
`git commit -am "refactor: rename core GMSI APIs to MODUS"`

### Task 3: Peripheral and Porting Layer (GDI to MDI)

**Files:**
- Rename: `peripheral/stm32g431/gdi_hw.h` -> `peripheral/stm32g431/mdi_hw.h`
- Rename: `peripheral/stm32g431/port_gdi.c` -> `peripheral/stm32g431/port_mdi.c`
- Modify: `peripheral/peripheral.h`, `src/main.c`

- [ ] **Step 1: Rename files and update internal macros**
  - `GDI_` -> `MDI_`
  - `gdi_` -> `mdi_`

- [ ] **Step 2: Commit Peripheral Changes**
`git commit -am "refactor: migrate GDI port to MDI"`

### Task 4: Cleanup and Verification

- [ ] **Step 1: Clean and Build**
Run: `make clean; make TARGET_CHIP=stm32g431`
Expected: Successful compilation.

- [ ] **Step 2: Final Commit**
`git commit -m "chore: complete migration to modus"`
