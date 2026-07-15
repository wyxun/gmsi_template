# Decouple grblHAL from root makefile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove all grblHAL-specific compilation logic, file includes, and definitions from the root `makefile` and encapsulate them in a new `grblhal_adapt/grblhal_adapt.mk` file.

**Architecture:**
1. Create `grblhal_adapt/grblhal_adapt.mk` containing all grblHAL sources, include paths, settings macros, and custom RTT stream logic.
2. Modify `target/at32f407/target.mk` and `target/stm32g431/target.mk` to include `grblhal_adapt/grblhal_adapt.mk` if `GRBLHAL_ENABLE` is defined.
3. Clean the root `makefile` to completely remove all grblHAL sections.

**Tech Stack:** Make, C

---

### Task 1: Create grblhal_adapt.mk Helper Makefile

**Files:**
- Create: `grblhal_adapt/grblhal_adapt.mk`

- [ ] **Step 1: Write grblhal_adapt.mk**
Create the new file containing all grblHAL compilation settings, source files, include paths, compiler flags, and the RTT stream build rule.

### Task 2: Update target.mk for at32f407 and stm32g431

**Files:**
- Modify: `target/at32f407/target.mk`
- Modify: `target/stm32g431/target.mk`

- [ ] **Step 1: Update at32f407 target.mk**
Include `grblhal_adapt/grblhal_adapt.mk` if `GRBLHAL_ENABLE` is set, and remove the now-redundant local grblHAL class and settings definitions.

- [ ] **Step 2: Update stm32g431 target.mk**
Include `grblhal_adapt/grblhal_adapt.mk` if `GRBLHAL_ENABLE` is set, and remove the local grblHAL class and settings definitions.

### Task 3: Clean Root makefile

**Files:**
- Modify: `makefile`

- [ ] **Step 1: Remove all grblHAL sections**
Remove all `ifdef GRBLHAL_ENABLE` blocks and custom compile targets/source checks for grblHAL in the root `makefile`.

### Task 4: Verify Builds

**Files:**
- Test: Run builds for both at32f407 and stm32g431 targets in debug and release modes.

- [ ] **Step 1: Verify at32f407 build**
Compile with `mingw32-make.exe TARGET_CHIP=at32f407 BUILD=release clean all` and ensure it compiles successfully.

- [ ] **Step 2: Verify stm32g431 build**
Compile with `mingw32-make.exe TARGET_CHIP=stm32g431 BUILD=release clean all` and ensure it compiles successfully.
