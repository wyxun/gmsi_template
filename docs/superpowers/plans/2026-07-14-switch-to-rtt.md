# Switch to RTT for grblHAL on AT32F407 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure grblHAL on AT32F407 to use the SEGGER RTT channel 0 virtual serial port for testing and debugging, replacing the physical UART stream.

**Architecture:** Change `#ifdef GRBLHAL_STREAM_UART` to `#if GRBLHAL_STREAM_UART` in `grblhal_stream.c`, then update `target/at32f407/target.mk` to define `GRBLHAL_STREAM_UART=0`, disabling UART stream and automatically enabling RTT stream fallback.

**Tech Stack:** C, Make

---

### Task 1: Update grblhal_stream.c Preprocessor Directives

**Files:**
- Modify: `grblhal_adapt/grblhal_stream.c:8`
- Modify: `grblhal_adapt/grblhal_stream.c:81`
- Modify: `grblhal_adapt/grblhal_stream.c:147`

- [ ] **Step 1: Modify grblhal_stream.c preprocessor conditions**
Change `#ifdef GRBLHAL_STREAM_UART` to `#if GRBLHAL_STREAM_UART` and `#endif /* GRBLHAL_STREAM_UART */` comments to match, enabling conditional evaluation of its numeric value.

### Task 2: Configure AT32F407 Target Makefile

**Files:**
- Modify: `target/at32f407/target.mk:14`

- [ ] **Step 1: Set GRBLHAL_STREAM_UART to 0**
Modify the `C_DEFS` in `target/at32f407/target.mk` to specify `-DGRBLHAL_STREAM_UART=0` instead of `-DGRBLHAL_STREAM_UART=1`.

### Task 3: Build Verification

**Files:**
- Test: Build project for AT32F407 target and check size/linking

- [ ] **Step 1: Run compilation**
Run compile command for at32f407 in debug mode to verify clean compilation and linking with RTT symbols.
Command: `mingw32-make.exe TARGET_CHIP=at32f407 BUILD=debug clean all`
Expected: Output successfully links `template.elf` and prints size.
