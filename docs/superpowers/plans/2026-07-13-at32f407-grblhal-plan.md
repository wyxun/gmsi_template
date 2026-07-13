# AT32F407 grblHAL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add AT32F407 target (AT-START-F407 board) with full grblHAL CNC controller over physical USART1, using interrupt-driven TX/RX ringbuf MDI adapters. No flash trimming; all hardware drivers stubbed; zero regression on existing targets.

**Architecture:** New `vendor/cortex-m/AT32F403A_407_Firmware_Library` submodule for chip drivers, new `target/at32f407/` with target.mk/conf/linkerscript/it.c/openocd, new `peripheral/at32f407/` with MDI mdi_hw/port_mdi/port_sys. Two shared files feature-gated (`grblhal_stream.c` via `GRBLHAL_STREAM_UART`, `grblhal_config.h` via `GRBLHAL_FULL_FEATURES`). All other files untouched.

**Tech Stack:** C11 (gnu11), Clang/LLVM for ARM, AT32F403A_407 Standard Peripheral Library, grblHAL core, MODUS framework

---

### Task 1: Add submodule — AT32F403A_407 Firmware Library

**Files:**
- Modify: `.gitmodules` (add 1 block)
- Create: (none — git submodule add writes to `.git/config`)

- [ ] **Step 1: Add submodule entry to `.gitmodules`**

Append this block at the end of `.gitmodules`:

```
[submodule "vendor/cortex-m/AT32F403A_407_Firmware_Library"]
	path = vendor/cortex-m/AT32F403A_407_Firmware_Library
	url = https://github.com/ArteryTek/AT32F403A_407_Firmware_Library.git
	ignore = untracked
```

- [ ] **Step 2: Add the submodule via git**

```bash
git submodule add --name "vendor/cortex-m/AT32F403A_407_Firmware_Library" \
  https://github.com/ArteryTek/AT32F403A_407_Firmware_Library.git \
  vendor/cortex-m/AT32F403A_407_Firmware_Library
```

- [ ] **Step 3: Verify submodule was added**

```bash
git submodule status vendor/cortex-m/AT32F403A_407_Firmware_Library
```
Expected: a commit hash followed by the path (not a leading `-`).

- [ ] **Step 4: Verify library structure**

```bash
ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/src/
ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/inc/
ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/
```

Expected: `.c` files in `src/`, `.h` files in `inc/`, `startup_at32f403a_407.s` and `system_at32f403a_407.c` in `device_support/`.

---

### Task 2: Create target/at32f407/ — build system

**Files:**
- Create: `target/at32f407/target.mk`
- Create: `target/at32f407/at32f407_conf.h`
- Create: `target/at32f407/AT32F407xC_FLASH.ld`
- Create: `target/at32f407/at32f407_it.c`
- Create: `target/at32f407/openocd.cfg`

- [ ] **Step 1: Create `target/at32f407/target.mk`**

Create the file with this content:

```makefile
# =============================================================================
# target/at32f407/target.mk
# AT32F407VGT7 — Cortex-M4F, single-precision FPU (AT-START-F407)
# =============================================================================

# grblHAL CNC controller — full features, UART stream
GRBLHAL_ENABLE = 1

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER
C_DEFS += -DGRBLHAL_FULL_FEATURES=1 -DGRBLHAL_STREAM_UART=1

# CMSIS / peripheral library paths
CHIPLIB_ROOT = vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/at32f407/AT32F407xC_FLASH.ld
STARTUP_S    = $(CMSIS_DEV)/startup/gcc/startup_at32f403a_407.s
SYSTEM_C     = $(CMSIS_DEV)/system_at32f403a_407.c
IT_C         = target/at32f407/at32f407_it.c

# Chip-specific include paths
TARGET_INCLUDES = -I$(DRV_INC) -Itarget/at32f407

# ------------------------------------------------------------------
# AT32F407 peripheral driver selection
# ------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_USART  ?= 1
USE_DRV_DEBUG  ?= 1
USE_DRV_TMR    ?= 0
USE_DRV_ADC    ?= 0
USE_DRV_DMA    ?= 0
USE_DRV_EXINT  ?= 0
USE_DRV_SPI    ?= 0
USE_DRV_I2C    ?= 0
USE_DRV_CAN    ?= 0
USE_DRV_WDT    ?= 0
USE_DRV_WWDT   ?= 0
USE_DRV_PWC    ?= 0
USE_DRV_RTC    ?= 0
USE_DRV_BPR    ?= 0
USE_DRV_CRC    ?= 0
USE_DRV_SDIO   ?= 0
USE_DRV_USB    ?= 0

DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_flash.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_usart.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_debug.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_tmr.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_adc.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_dma.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_exint.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_i2c.c
endif
ifeq ($(USE_DRV_CAN),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_can.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_wwdt.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_pwc.c
endif

CHIP_SOURCES = $(DRV_SOURCES)

# grblHAL class source
CLASS_SOURCES += class/grblhal.c

# Compile grblHAL settings.c with __SETTINGS_C__ defined
build/settings.o: CFLAGS += -D__SETTINGS_C__

# Enable MODUS default perf_counter port
MODUS_USE_DEFAULT_PERFC_PORT = 1

# OpenOCD (AT32 custom)
OPENOCD_BIN     = $(SW_ROOT)/msys64/mingw64/bin/openocd-at32.exe
OPENOCD_SCRIPTS = $(SW_ROOT)/msys64/mingw64/share/openocd/scripts
```

Note: The actual driver `.c` filenames in the AT32F403A_407 library may use the pattern `at32f403a_407_<periph>.c`. After the submodule is checked out, verify the exact filenames with `ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/src/` and adjust the `DRV_SOURCES` entries accordingly. The pattern shown above is the expected one based on the library naming convention.

- [ ] **Step 2: Create `target/at32f407/at32f407_conf.h`**

Create the file with this content:

```c
/**
 * @file   at32f407_conf.h
 * @brief  AT32F407 peripheral library configuration (project-local copy).
 *         Enable only the modules used in this project.
 */

#ifndef __AT32F407_CONF_H
#define __AT32F407_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Clock values -------------------------------------------------------- */
#if !defined(HEXT_VALUE)
#define HEXT_VALUE               ((uint32_t)8000000)   /* 8 MHz external crystal */
#endif
#define HEXT_STARTUP_TIMEOUT     ((uint16_t)0x3000)
#define HICK_VALUE               ((uint32_t)8000000)   /* 8 MHz internal RC */

/* ---- Module enable ------------------------------------------------------- */
#define CRM_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define DEBUG_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define MISC_MODULE_ENABLED

/* ---- Includes ------------------------------------------------------------ */
#ifdef CRM_MODULE_ENABLED
    #include "at32f403a_407_crm.h"
#endif
#ifdef GPIO_MODULE_ENABLED
    #include "at32f403a_407_gpio.h"
#endif
#ifdef USART_MODULE_ENABLED
    #include "at32f403a_407_usart.h"
#endif
#ifdef DEBUG_MODULE_ENABLED
    #include "at32f403a_407_debug.h"
#endif
#ifdef FLASH_MODULE_ENABLED
    #include "at32f403a_407_flash.h"
#endif
#ifdef MISC_MODULE_ENABLED
    #include "at32f403a_407_misc.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __AT32F407_CONF_H */
```

- [ ] **Step 3: Create `target/at32f407/AT32F407xC_FLASH.ld`**

Create the linkerscript. Clone from `target/at32f413/AT32F413xC_FLASH.ld` and change only the memory sizes and top-of-stack:

```ld
/*
*****************************************************************************
**
**  File        : AT32F407xC_FLASH.ld
**
**  Abstract    : Linker script for AT32F407VGT7 Device with
**                1024KByte FLASH, 96KByte SRAM
**
**  Target      : Artery Tek AT32
**
**  Environment : Arm gcc toolchain
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20018000;    /* end of 96KB SRAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 1024K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 96K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .init_infos :
  {
    . = ALIGN(4);
    __start_init_infos = .;
    KEEP(*(init_infos))
    KEEP(*(init_infos*))
    __stop_init_infos = .;
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
```

- [ ] **Step 4: Create `target/at32f407/at32f407_it.c`**

Create the file with this content:

```c
/**
 * @file   at32f407_it.c
 * @brief  Interrupt handlers (AT32F407 AT-START-F407)
 *
 * SysTick_Handler drives perf_counter + modus + USART 1ms tick.
 * USART1 is the only active peripheral IRQ.
 */

#define CORE_DEBUG_OVERRIDE_FAULT_HANDLER
#include "at32f403a_407.h"
#include "perf_counter.h"
#include "mdebug_cm.h"

/* Exported by main.c */
extern void modus_Clock(void);
extern void peripheral_Clock(void);

/* USART private instance (defined in port_mdi.c) */
#include "port_mdi.h"
extern at32_usart_priv_t s_tUsart1Priv;

/* --------------------------------------------------------------------------
 *  Cortex-M4 Core Exceptions
 * -------------------------------------------------------------------------- */
void NMI_Handler(void)              { while(1); }
#ifndef MDEBUG_CM_FAULT_HANDLERS_ACTIVE
void HardFault_Handler(void)        { while(1); }
void MemManage_Handler(void)        { while(1); }
void BusFault_Handler(void)         { while(1); }
void UsageFault_Handler(void)       { while(1); }
#endif
void SVC_Handler(void)              {}
void DebugMon_Handler(void)         {}
void PendSV_Handler(void)           {}

void SysTick_Handler(void)
{
    perfc_port_insert_to_system_timer_insert_ovf_handler();
    modus_Clock();
    at32_usart_timer_1ms(&s_tUsart1Priv);
}

/* --------------------------------------------------------------------------
 *  AT32F407 Peripheral Interrupts
 * -------------------------------------------------------------------------- */

/* ---- USART1 (grblHAL serial I/O) ---- */
void USART1_IRQHandler(void)        { at32_usart_irq_handler(&s_tUsart1Priv); }

/* ---- Stubs (all other IRQs) ---- */
void WWDT_IRQHandler(void)                  {}
void PVM_IRQHandler(void)                   {}
void TAMPER_IRQHandler(void)                {}
void RTC_IRQHandler(void)                   {}
void FLASH_IRQHandler(void)                 {}
void CRM_IRQHandler(void)                   {}
void EXINT0_IRQHandler(void)                {}
void EXINT1_IRQHandler(void)                {}
void EXINT2_IRQHandler(void)                {}
void EXINT3_IRQHandler(void)                {}
void EXINT4_IRQHandler(void)                {}
void EXINT9_5_IRQHandler(void)              {}
void EXINT15_10_IRQHandler(void)            {}
void DMA1_Channel1_IRQHandler(void)         {}
void DMA1_Channel2_IRQHandler(void)         {}
void DMA1_Channel3_IRQHandler(void)         {}
void DMA1_Channel4_IRQHandler(void)         {}
void DMA1_Channel5_IRQHandler(void)         {}
void DMA1_Channel6_IRQHandler(void)         {}
void DMA1_Channel7_IRQHandler(void)         {}
void DMA2_Channel1_IRQHandler(void)         {}
void DMA2_Channel2_IRQHandler(void)         {}
void DMA2_Channel3_IRQHandler(void)         {}
void DMA2_Channel4_5_IRQHandler(void)       {}
void DMA2_Channel6_7_IRQHandler(void)       {}
void TMR1_BRK_TMR9_IRQHandler(void)         {}
void TMR1_OVF_TMR10_IRQHandler(void)        {}
void TMR1_TRG_HALL_TMR11_IRQHandler(void)   {}
void TMR1_CH_IRQHandler(void)               {}
void TMR2_GLOBAL_IRQHandler(void)           {}
void TMR3_GLOBAL_IRQHandler(void)           {}
void TMR4_GLOBAL_IRQHandler(void)           {}
void TMR5_GLOBAL_IRQHandler(void)           {}
void TMR8_BRK_IRQHandler(void)              {}
void TMR8_OVF_IRQHandler(void)              {}
void TMR8_TRG_HALL_IRQHandler(void)         {}
void TMR8_CH_IRQHandler(void)               {}
void ADC1_2_IRQHandler(void)                {}
void USART2_IRQHandler(void)                {}
void USART3_IRQHandler(void)                {}
void UART4_IRQHandler(void)                 {}
void UART5_IRQHandler(void)                 {}
void SPI1_IRQHandler(void)                  {}
void SPI2_IRQHandler(void)                  {}
void I2C1_EVT_IRQHandler(void)              {}
void I2C1_ERR_IRQHandler(void)              {}
void I2C2_EVT_IRQHandler(void)              {}
void I2C2_ERR_IRQHandler(void)              {}
void USBFS_H_CAN1_TX_IRQHandler(void)       {}
void USBFS_L_CAN1_RX0_IRQHandler(void)      {}
void CAN1_RX1_IRQHandler(void)              {}
void CAN1_SE_IRQHandler(void)               {}
void CAN2_TX_IRQHandler(void)               {}
void CAN2_RX0_IRQHandler(void)              {}
void CAN2_RX1_IRQHandler(void)              {}
void CAN2_SE_IRQHandler(void)               {}
void SDIO1_IRQHandler(void)                 {}
void RTCAlarm_IRQHandler(void)              {}
void USBFSWakeUp_IRQHandler(void)           {}
void ACC_IRQHandler(void)                   {}
void USBFS_MAPH_IRQHandler(void)            {}
void USBFS_MAPL_IRQHandler(void)            {}
```

Note: The exact set of IRQ handlers depends on the AT32F407 interrupt vector table in the startup file. After the submodule is in place, check `vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f403a_407.s` for the complete vector table and adjust stub names if they differ from what's used here.

- [ ] **Step 5: Create `target/at32f407/openocd.cfg`**

Create the file with this content:

```
source [find interface/cmsis-dap.cfg]
cmsis_dap_backend hid
source [find target/at32f407xx.cfg]
```

---

### Task 3: Create peripheral/at32f407/ — MDI hardware adapters

**Files:**
- Create: `peripheral/at32f407/mdi_hw.h`
- Create: `peripheral/at32f407/port_mdi.h`
- Create: `peripheral/at32f407/port_mdi.c`
- Create: `peripheral/at32f407/port_sys.c`

- [ ] **Step 1: Create `peripheral/at32f407/mdi_hw.h`**

Create the file with this content:

```c
/**
 * @file mdi_hw.h
 * @brief Global peripheral resource definition (MDI hardware pool) — AT32F407
 *
 * Provides a unified hardware structure for the application layer,
 * avoiding exposure of chip-specific headers.
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/mdi.h"

/*============================================================================
 * Project hardware resource pool
 *===========================================================================*/

typedef struct {
    /* ---------- LED / Status ---------- */
    mdi_gpio_t   *ptLedStatus;    /**< PD13, active-low (AT-START-F407 LED2) */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< USART1 PA9/PA10 (on-board ST-Link VCP) */

} mdi_hardware_t;

extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
```

- [ ] **Step 2: Create `peripheral/at32f407/port_mdi.h`**

Create the file with this content:

```c
#ifndef __PORT_MDI_H__
#define __PORT_MDI_H__

#include "at32f403a_407.h"
#include "mringbuf.h"

typedef struct {
    usart_type *ptUsart;
    mringbuf_t tTxQueue;
    mringbuf_t tRxQueue;
    volatile uint8_t chTXFlag;
    volatile uint8_t chRXFlag;
    volatile uint8_t chRXFinishTime;
} at32_usart_priv_t;

/* Generic USART Handlers */
void at32_usart_init(at32_usart_priv_t *ptPriv,
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize);

void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv);
void at32_usart_irq_handler(at32_usart_priv_t *ptPriv);

/* Instance Wrappers */
void at32_usart1_init(void);

#endif /* __PORT_MDI_H__ */
```

- [ ] **Step 3: Create `peripheral/at32f407/port_mdi.c`**

Create the file with this content. This is a near-clone of `peripheral/at32f413/port_mdi.c`, stripped of FOC peripherals (PWM, ADC, comparator, button) and adapted for AT-START-F407 pinout (USART1 on PA9/PA10, LED on PD13):

```c
/**
 * @file   port_mdi.c
 * @brief  AT32F407 MDI adapter — binds peripheral hardware to MDI objects
 *
 * USART1 interrupt-driven TX/RX ringbuf (same pattern as AT32F413).
 * GPIO wrappers for status LED.
 */

#include "mdi_hw.h"
#include "at32f403a_407.h"
#include "mdi/mdi.h"

#include "port_mdi.h"

/*============================================================================
 * AT32F407 USART1 adapter (same ringbuf pattern as AT32F413)
 *===========================================================================*/
#define USART1_TX_BUFFER_SIZE  256
#define USART1_RX_BUFFER_SIZE  256

#define USART_RXFLAG_IDLE   0
#define USART_RXFLAG_BUSY   1
#define USART_RXFLAG_FINISH 2

#define USART_TXFLAG_IDLE   0
#define USART_TXFLAG_BUSY   1

#define USART_DELAYTIME     5

static uint8_t s_chUsart1TxBuf[USART1_TX_BUFFER_SIZE];
static uint8_t s_chUsart1RxBuf[USART1_RX_BUFFER_SIZE];

/* ---- GPIO wrappers ---- */

static int32_t at32_gpio_Set(void *pPriv, mdi_gpio_level_t eLevel)
{
    void **ap = (void **)pPriv;
    gpio_bits_write((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1], (confirm_state)eLevel);
    return 0;
}

static int32_t at32_gpio_Get(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_HIGH : MDI_GPIO_LOW;
}

static int32_t at32_gpio_Toggle(void *pPriv)
{
    void **ap = (void **)pPriv;
    gpio_bits_toggle((gpio_type *)ap[0], (uint16_t)(uintptr_t)ap[1]);
    return 0;
}

static int32_t at32_gpio_Get_ActiveLow(void *pPriv)
{
    void **ap = (void **)pPriv;
    uint16_t hwPins = (uint16_t)(uintptr_t)ap[1];
    return ((gpio_type *)ap[0])->idt & hwPins ? MDI_GPIO_LOW : MDI_GPIO_HIGH;
}

/* ---- LED GPIO instance (active-low, PD13 = AT-START-F407 LED2) ---- */

static void *s_apvLedStatPriv[] = { GPIOD, (void *)(uintptr_t)GPIO_PINS_13 };
static mdi_gpio_t s_tLedStatus = {
    .pPriv = s_apvLedStatPriv, .fnSet = at32_gpio_Set,
    .fnGet = at32_gpio_Get_ActiveLow, .fnToggle = at32_gpio_Toggle,
};

/* ---- USART1 ringbuf logic (reused from AT32F413) ---- */

at32_usart_priv_t s_tUsart1Priv = { .ptUsart = NULL };

void at32_usart1_init(void)
{
    if (s_tUsart1Priv.ptUsart != NULL) return;
    at32_usart_init(&s_tUsart1Priv, USART1,
                    s_chUsart1TxBuf, USART1_TX_BUFFER_SIZE,
                    s_chUsart1RxBuf, USART1_RX_BUFFER_SIZE);
}

void at32_usart_init(at32_usart_priv_t *ptPriv,
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize)
{
    if (NULL == ptPriv || NULL == ptUsart) return;

    ptPriv->ptUsart = ptUsart;
    mringbuf_Init(&ptPriv->tTxQueue, pchTxBuf, (uint16_t)wTxBufSize);
    mringbuf_Init(&ptPriv->tRxQueue, pchRxBuf, (uint16_t)wRxBufSize);
    ptPriv->chTXFlag = USART_TXFLAG_IDLE;
    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    ptPriv->chRXFinishTime = 0;
}

void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv) return;
    if (ptPriv->chRXFlag == USART_RXFLAG_BUSY) {
        if (ptPriv->chRXFinishTime > 0) {
            ptPriv->chRXFinishTime--;
        } else {
            ptPriv->chRXFlag = USART_RXFLAG_FINISH;
        }
    }
}

static int32_t at32_stream_Write(void *pPriv, const uint8_t *pchData, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i;

    for (i = 0; i < wLen; i++) {
        if (mringbuf_Write(&ptPriv->tTxQueue, pchData[i]) == 0) break;
    }

    if (ptPriv->chTXFlag == USART_TXFLAG_IDLE
        && mringbuf_GetUsed(&ptPriv->tTxQueue) > 0) {
        uint8_t chData;
        if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
            usart_data_transmit(ptPriv->ptUsart, chData);
            ptPriv->chTXFlag = USART_TXFLAG_BUSY;
        }
    }
    return i;
}

static int32_t at32_stream_Read(void *pPriv, uint8_t *pchBuf, uint32_t wLen)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    uint32_t i = 0;
    uint16_t hwRead;

    if (ptPriv->chRXFlag != USART_RXFLAG_FINISH) return 0;

    do {
        hwRead = mringbuf_Read(&ptPriv->tRxQueue, &pchBuf[i]);
        if (hwRead > 0) i++;
        if (i >= wLen) break;
    } while (hwRead > 0);

    ptPriv->chRXFlag = USART_RXFLAG_IDLE;
    return i;
}

static int32_t at32_stream_IsBusy(void *pPriv)
{
    at32_usart_priv_t *ptPriv = (at32_usart_priv_t *)pPriv;
    return (ptPriv->chTXFlag == USART_TXFLAG_BUSY) ? 1 : 0;
}

static mdi_stream_t s_tStreamSerial = {
    .pPriv    = &s_tUsart1Priv,
    .fnWrite  = at32_stream_Write,
    .fnRead   = at32_stream_Read,
    .fnIsBusy = at32_stream_IsBusy,
};

void at32_usart_irq_handler(at32_usart_priv_t *ptPriv)
{
    if (NULL == ptPriv || NULL == ptPriv->ptUsart) return;
    uint8_t chData;
    usart_type *ptHW = ptPriv->ptUsart;

    if (ptHW->ctrl1_bit.rdbfien != RESET) {
        if (usart_interrupt_flag_get(ptHW, USART_RDBF_FLAG) != RESET) {
            uint8_t ch = usart_data_receive(ptHW);
            mringbuf_Write(&ptPriv->tRxQueue, ch);
            ptPriv->chRXFinishTime = USART_DELAYTIME;
            ptPriv->chRXFlag = USART_RXFLAG_BUSY;
        }
    }

    if (ptHW->ctrl1_bit.tdcien != RESET) {
        if (usart_flag_get(ptHW, USART_TDC_FLAG) != RESET) {
            usart_flag_clear(ptHW, USART_TDC_FLAG);
            if (mringbuf_Read(&ptPriv->tTxQueue, &chData) == 1) {
                usart_data_transmit(ptHW, chData);
                ptPriv->chTXFlag = USART_TXFLAG_BUSY;
            } else {
                ptPriv->chTXFlag = USART_TXFLAG_IDLE;
            }
        }
    }
}

/*============================================================================
 * Global hardware resource pool
 *===========================================================================*/

const mdi_hardware_t HW = {
    .ptLedStatus = &s_tLedStatus,   /* PD13 — primary heartbeat LED */
    .ptSerial    = &s_tStreamSerial,
};
```

- [ ] **Step 4: Create `peripheral/at32f407/port_sys.c`**

Create the file with this content. Clock config targets 240 MHz (AT32F407 max) from 8 MHz HSE. USART1 is initialized on PA9/PA10 (no remap needed — these are the default USART1 pins on AT32F407, directly connected to the ST-Link VCP on AT-START-F407):

```c
/**
 * @file   port_sys.c
 * @brief  AT32F407 AT-START-F407 system-level init (clock + USART1 + LED + SysTick)
 *
 * Clock: HEXT 8MHz → PLL ×30 → 240MHz SYSCLK (AT32F407 max)
 *        HCLK = 240MHz
 *        APB2 = 120MHz (PCLK2)
 *        APB1 =  60MHz (PCLK1)
 */

#include "peripheral.h"
#include "at32f403a_407.h"
#include "port_mdi.h"

/* --------------------------------------------------------------------------
 *  System clock: HEXT 8MHz / 2 × 60 = 240MHz (or as close as PLL allows)
 *               Fallback: HICK 8MHz (no PLL) if HEXT fails
 * -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
    /* Reset CRM */
    crm_reset();

    /* Enable HICK (needed as fallback; 8 MHz internal RC) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);

    /* Try HEXT (8MHz external crystal on AT-START-F407) */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (crm_hext_stable_wait() == ERROR) {
        /* No HEXT — stay on HICK 8MHz, skip PLL */
        crm_ahb_div_set(CRM_AHB_DIV_1);
        crm_apb2_div_set(CRM_APB2_DIV_1);
        crm_apb1_div_set(CRM_APB1_DIV_1);
        system_core_clock_update();
        return;
    }

    /* PLL source = HEXT/2 = 4MHz, ×60 = 240MHz, range > 72MHz */
    crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_60,
                   CRM_PLL_OUTPUT_RANGE_GT72MHZ);

    /* Enable PLL */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

    /* AHB / APB dividers */
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_apb1_div_set(CRM_APB1_DIV_4);

    /* Auto step mode for smooth clock switch */
    crm_auto_step_mode_enable(TRUE);

    /* Switch system clock to PLL */
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL);

    crm_auto_step_mode_enable(FALSE);

    /* Update global SystemCoreClock */
    system_core_clock_update();
}

/* --------------------------------------------------------------------------
 *  USART1 init — PA9=TX, PA10=RX, 115200-8-N-1
 *  AT-START-F407 connects these to the on-board ST-Link VCP.
 * -------------------------------------------------------------------------- */
static void halusart_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* USART1 TX — PA9 */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins      = GPIO_PINS_9;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* USART1 RX — PA10 */
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio_init_struct);

    nvic_irq_enable(USART1_IRQn, 8, 0);

    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);

    usart_flag_clear(USART1, USART_TDBE_FLAG);
    usart_flag_clear(USART1, USART_TDC_FLAG);
    usart_flag_clear(USART1, USART_RDBF_FLAG);
    usart_interrupt_enable(USART1, USART_TDC_INT, TRUE);

    /* Bind USART1 to MDI stream instance */
    at32_usart1_init();
}

/* --------------------------------------------------------------------------
 *  Status LED init — PD13, push-pull, active-low
 * -------------------------------------------------------------------------- */
static void halled_Init(void)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode      = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins      = GPIO_PINS_13;
    gpio_init_struct.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOD, &gpio_init_struct);

    /* LED off by default (active-low, set HIGH = off) */
    gpio_bits_set(GPIOD, GPIO_PINS_13);
}

/* --------------------------------------------------------------------------
 *  peripheral_Init — main() calls this first
 * -------------------------------------------------------------------------- */
void peripheral_Init(void)
{
    /* NVIC: 4-bit preemption priority, 0-bit sub-priority */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    SystemClock_Config();

    halled_Init();
    halusart_Init();

    /* SysTick 1ms interrupt */
    SysTick_Config(SystemCoreClock / 1000U);
}

/* --------------------------------------------------------------------------
 *  System clock query
 * -------------------------------------------------------------------------- */
uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}

/* --------------------------------------------------------------------------
 *  Unused required symbols
 * -------------------------------------------------------------------------- */
void peripheral_Clock(void) {}
void peripheral_EnableIRQ(void) {}
void peripheral_DisableIRQ(void) {}
```

Note: Some register-level field names may differ slightly between the AT32F403A_407 library and the AT32F413/AT32F421 libraries (e.g. `GPIO_DRIVE_STRENGTH_STRONGER` vs a different name, or `ctrl1_bit.rdbfien` accessor). After the submodule is checked out, verify these against `vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/inc/at32f403a_407_gpio.h` and `at32f403a_407_usart.h`. Adjust the code if the library uses different naming.

---

### Task 4: Feature-gate grblhal_adapt/ — UART stream + full features

**Files:**
- Modify: `grblhal_adapt/grblhal_stream.c` (add `#ifdef GRBLHAL_STREAM_UART` path)
- Modify: `grblhal_adapt/grblhal_config.h` (add `#ifdef GRBLHAL_FULL_FEATURES` path)

- [ ] **Step 1: Modify `grblhal_adapt/grblhal_stream.c`**

Replace the entire file with a feature-gated version. The existing STM32G431 RTT path is kept exactly as-is inside the `#else` block. The new `#ifdef GRBLHAL_STREAM_UART` block uses `MDI_Read`/`MDI_Write` through `HW.ptSerial`.

Replace the file content with:

```c
/**
 * @file   grblhal_stream.c
 * @brief  grblHAL stream I/O — RTT (STM32G431) or MDI UART (AT32F407)
 */

#include "grblhal_driver.h"

#ifdef GRBLHAL_STREAM_UART

/* =========================================================================
 *  AT32F407 path: stream I/O via MDI UART (HW.ptSerial = USART1)
 * ========================================================================= */
#include "mdi_hw.h"
#include "mdi/mdi.h"
#include <string.h>

static void uart_write_n(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0) {
        MDI_Write(HW.ptSerial, data, (uint32_t)length);
    }
}

static bool uart_write_char(const uint8_t c)
{
    return MDI_Write(HW.ptSerial, &c, 1) == 1;
}

static bool uart_is_connected(void)
{
    return true;
}

static int32_t uart_read(void)
{
    uint8_t ch;
    int32_t n = MDI_Read(HW.ptSerial, &ch, 1);
    return (n > 0) ? (int32_t)ch : -1;
}

static void uart_reset_read_buffer(void)
{
    /* Drain any pending RX data */
    uint8_t ch;
    while (MDI_Read(HW.ptSerial, &ch, 1) > 0) {}
}

static bool uart_suspend_read(bool suspend)
{
    (void)suspend;
    return true;
}

static bool uart_enqueue_rt_command(uint8_t c)
{
    (void)c;
    return false;
}

static uint16_t uart_get_rx_buffer_available(void)
{
    /* MDI stream doesn't expose pending count — poll one byte */
    return 0;
}

static io_stream_t s_grblhal_uart = {
    .type                  = StreamType_Serial,
    .is_connected          = uart_is_connected,
    .read                  = uart_read,
    .reset_read_buffer     = uart_reset_read_buffer,
    .suspend_read          = uart_suspend_read,
    .enqueue_rt_command    = uart_enqueue_rt_command,
    .get_rx_buffer_count   = uart_get_rx_buffer_available,
    .write_n               = uart_write_n,
    .write_char            = uart_write_char,
};

void grblhal_stream_init(void)
{
    hal.stream = s_grblhal_uart;
}

#else /* !GRBLHAL_STREAM_UART — original STM32G431 RTT path (unchanged) */

/* =========================================================================
 *  STM32G431 path: stream I/O via SEGGER RTT channel 0
 * ========================================================================= */
#include "SEGGER_RTT.h"

static void rtt_write_n(const uint8_t *data, uint16_t length);
static bool rtt_write_char(const uint8_t c);
static bool rtt_is_connected(void);
static int32_t rtt_read(void);
static void rtt_reset_read_buffer(void);
static bool rtt_suspend_read(bool suspend);
static bool rtt_enqueue_rt_command(uint8_t c);
static uint16_t rtt_get_rx_buffer_available(void);

static io_stream_t s_grblhal_rtt = {
    .type                  = StreamType_Serial,
    .is_connected          = rtt_is_connected,
    .read                  = rtt_read,
    .reset_read_buffer     = rtt_reset_read_buffer,
    .suspend_read          = rtt_suspend_read,
    .enqueue_rt_command    = rtt_enqueue_rt_command,
    .get_rx_buffer_count   = rtt_get_rx_buffer_available,
    .write_n               = rtt_write_n,
    .write_char            = rtt_write_char,
};

static void rtt_write_n(const uint8_t *data, uint16_t length)
{
    if (data != NULL && length > 0) {
        SEGGER_RTT_Write(0, data, (unsigned)length);
    }
}

static bool rtt_write_char(const uint8_t c)
{
    SEGGER_RTT_PutChar(0, (char)c);
    return true;
}

static bool rtt_is_connected(void) { return true; }

static int32_t rtt_read(void)
{
    uint8_t ch;
    if (SEGGER_RTT_Read(0, &ch, 1) == 1) {
        return (int32_t)ch;
    }
    return -1;
}

static void rtt_reset_read_buffer(void)    { /* no-op for RTT */ }
static bool rtt_suspend_read(bool s)       { (void)s; return true; }
static bool rtt_enqueue_rt_command(uint8_t c) { (void)c; return false; }
static uint16_t rtt_get_rx_buffer_available(void)
{
    uint8_t ch;
    return (SEGGER_RTT_Read(0, &ch, 1) == 1) ? 1 : 0;
}

void grblhal_stream_init(void)
{
    hal.stream = s_grblhal_rtt;
}

#endif /* GRBLHAL_STREAM_UART */
```

- [ ] **Step 2: Modify `grblhal_adapt/grblhal_config.h`**

Replace the entire file with a feature-gated version. The existing STM32G431 trimmed config is kept exactly as-is in the `#else` block. The new `#ifdef GRBLHAL_FULL_FEATURES` block enables all features with no trimming:

Replace the file content with:

```c
/**
 * @file   grblhal_config.h
 * @brief  grblHAL compile-time configuration overrides for modus_template
 *
 * Two paths:
 *   GRBLHAL_FULL_FEATURES (AT32F407) — full feature set, no trimming
 *   default (STM32G431) — trimmed for 128KB flash
 */

#ifndef __GRBLHAL_CONFIG_H__
#define __GRBLHAL_CONFIG_H__

#ifdef GRBLHAL_FULL_FEATURES

/* =========================================================================
 *  AT32F407: full features — 1024KB flash, no trimming needed
 * ========================================================================= */
#define COMPATIBILITY_LEVEL             10

/* ---- Machine geometry ---- */
#define N_AXIS          3       /* X, Y, Z — 3-axis CNC */
#define HOMING_CYCLE_0  (1 << 0)
#define HOMING_CYCLE_1  (1 << 1)
#define HOMING_CYCLE_2  (1 << 2)

/* ---- Buffer sizes ---- */
#define RX_BUFFER_SIZE      256
#define TX_BUFFER_SIZE      256
#define BLOCK_BUFFER_SIZE   36
#define NVS_BUFFER_SIZE     512

/* ---- Feature toggles (all enabled — stubs for now) ---- */
#define SPINDLE_ENABLE      1
#define COOLANT_ENABLE      1
#define PROBE_ENABLE        1
#define LIMITS_ENABLE       1
#define CONTROL_ENABLE      1
#define ENABLE_SAFETY_DOOR_INPUT_PIN  1
#define SDCARD_ENABLE       0
#define EEPROM_ENABLE       0
#define BLUETOOTH_ENABLE    0
#define ETHERNET_ENABLE     0
#define WIFI_ENABLE         0
#define ENABLE_SOFTWARE_DEBOUNCE  0

/* ---- Full feature set (no trimming) ---- */
#define NGC_PARAMETERS_ENABLE           1
#define NVSDATA_BUFFER_ENABLE           1
#define ENABLE_RESTORE_NVS_WIPE_ALL     1
#define ENABLE_RESTORE_NVS_DEFAULT_SETTINGS 1
#define ENABLE_RESTORE_NVS_CLEAR_PARAMETERS 1
#define ENABLE_RESTORE_NVS_DRIVER_PARAMETERS 1
#define SETTINGS_RESTORE_DEFAULTS       1
#define SETTINGS_RESTORE_PARAMETERS     1
#define SETTINGS_RESTORE_STARTUP_LINES  1
#define SETTINGS_RESTORE_BUILD_INFO     1
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 1

/* No trimming macros — everything stays in */

#ifdef __SETTINGS_C__
#include <stdint.h>
#include "errors.h"
#include "settings.h"
#endif

#else /* !GRBLHAL_FULL_FEATURES — original STM32G431 trimmed config (unchanged) */

/* =========================================================================
 *  STM32G431: trimmed for 128KB flash
 * ========================================================================= */
#define COMPATIBILITY_LEVEL             10

/* ---- Machine geometry ---- */
#define N_AXIS          3       /* X, Y, Z — 3-axis CNC */
#define HOMING_CYCLE_0  (1 << 0)
#define HOMING_CYCLE_1  (1 << 1)
#define HOMING_CYCLE_2  (1 << 2)

/* ---- Buffer sizes ---- */
#define RX_BUFFER_SIZE      256
#define TX_BUFFER_SIZE      256
#define BLOCK_BUFFER_SIZE   36
#define NVS_BUFFER_SIZE     512

/* ---- Feature toggles (stub driver — minimal capabilities) ---- */
#define SPINDLE_ENABLE      0
#define COOLANT_ENABLE      0
#define PROBE_ENABLE        0
#define LIMITS_ENABLE       0
#define CONTROL_ENABLE      0
#define ENABLE_SAFETY_DOOR_INPUT_PIN  0
#define SDCARD_ENABLE       0
#define EEPROM_ENABLE       0
#define BLUETOOTH_ENABLE    0
#define ETHERNET_ENABLE     0
#define WIFI_ENABLE         0
#define ENABLE_SOFTWARE_DEBOUNCE  0

/* ---- Footprint optimization (disable unused heavy features) ---- */
#define NGC_PARAMETERS_ENABLE           0
#define NVSDATA_BUFFER_ENABLE           0
#define ENABLE_RESTORE_NVS_WIPE_ALL     0
#define ENABLE_RESTORE_NVS_DEFAULT_SETTINGS 0
#define ENABLE_RESTORE_NVS_CLEAR_PARAMETERS 0
#define ENABLE_RESTORE_NVS_DRIVER_PARAMETERS 0
#define SETTINGS_RESTORE_DEFAULTS       0
#define SETTINGS_RESTORE_PARAMETERS     0
#define SETTINGS_RESTORE_STARTUP_LINES  0
#define SETTINGS_RESTORE_BUILD_INFO     0
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 0

#define NO_SAFETY_DOOR_SUPPORT          1
#define NO_SETTINGS_DESCRIPTIONS        1
#define NO_ERROR_DESCRIPTIONS           1
#define NO_TOOL_CHANGE_SUPPORT          1

#ifdef __SETTINGS_C__
#ifdef NO_SAFETY_DOOR_SUPPORT
#include <stdint.h>
#include "errors.h"
#include "settings.h"
static inline status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value) { return Status_OK; }
static inline status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value) { return Status_OK; }
#endif
#endif

#endif /* GRBLHAL_FULL_FEATURES */

#endif /* __GRBLHAL_CONFIG_H__ */
```

---

### Task 5: Build and verify

**Files:** none (verification only)

- [ ] **Step 1: Build AT32F407 target**

```bash
mingw32-make TARGET_CHIP=at32f407 clean
mingw32-make TARGET_CHIP=at32f407
```

Expected: build succeeds with no errors. If the AT32F403A_407 library uses different filenames or register names than assumed, fix the build errors by adjusting `target.mk` driver source names, `port_sys.c` register/field names, or `port_mdi.c` register access patterns.

- [ ] **Step 2: Check ELF size**

```bash
mingw32-make TARGET_CHIP=at32f407 size
```

Expected: `text` + `data` well under 1024K (1024 × 1024 = 1,048,576 bytes). With full grblHAL features and no trimming, expect roughly 150-200KB.

- [ ] **Step 3: Verify STM32G431 still builds (no regression)**

```bash
mingw32-make TARGET_CHIP=stm32g431 clean
mingw32-make TARGET_CHIP=stm32g431
```

Expected: build succeeds, RTT stream path active (no change from before).

- [ ] **Step 4: Verify AT32F413 still builds (no regression)**

```bash
mingw32-make TARGET_CHIP=at32f413 clean
mingw32-make TARGET_CHIP=at32f413
```

Expected: build succeeds, no grblHAL involvement (FOC-only build).

- [ ] **Step 5: Verify AT32F421 still builds (no regression)**

```bash
mingw32-make TARGET_CHIP=at32f421 clean
mingw32-make TARGET_CHIP=at32f421
```

Expected: build succeeds.

---

### Task 6: Fixups (library naming adaptation)

After the submodule is cloned, the actual filenames and register macros in the AT32F403A_407 library may differ slightly from what was assumed above. This task addresses those differences.

**Files:**
- Possibly modify: `target/at32f407/target.mk` (driver .c filenames)
- Possibly modify: `peripheral/at32f407/port_sys.c` (clock/PLL/GPIO macros)
- Possibly modify: `peripheral/at32f407/port_mdi.c` (USART register accessor names)
- Possibly modify: `target/at32f407/at32f407_it.c` (IRQ handler names)

- [ ] **Step 1: Inspect actual driver filenames**

```bash
ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/src/
```

Compare against the filenames used in `target.mk` DRV_SOURCES. If any filenames differ (e.g., `at32f403a_407_crm.c` vs `at32f403a_407_crm.c`), update the corresponding entries in `target/at32f407/target.mk`.

- [ ] **Step 2: Inspect actual CMSIS header filename**

```bash
ls vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/
```

Look for the main chip header (e.g., `at32f403a_407.h`). If the filename differs, update the `#include` in `at32f407_conf.h`, `at32f407_it.c`, `port_mdi.c`, and `port_mdi.h`.

- [ ] **Step 3: Inspect GPIO register/struct names**

```bash
grep -n "gpio_init_type\|gpio_type\|GPIO_DRIVE_STRENGTH\|GPIO_PINS_" \
  vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/inc/at32f403a_407_gpio.h | head -20
```

Compare against the names used in `port_sys.c` and `port_mdi.c`. If the library uses different names (e.g., `gpio_init_type` → `gpio_init_struct_type`, or `GPIO_DRIVE_STRENGTH_STRONGER` → `GPIO_DRIVE_STRENGTH_HIGH`), update accordingly.

- [ ] **Step 4: Inspect USART register accessor names**

```bash
grep -n "ctrl1_bit\|rdbfien\|tdcien\|USART_RDBF_FLAG\|USART_TDC_FLAG\|USART_TDBE_FLAG" \
  vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/inc/at32f403a_407_usart.h | head -20
```

Compare against the names used in `port_mdi.c`'s `at32_usart_irq_handler`. If the library uses different bitfield or flag names (e.g., `ctrl1_bit.rdbfien` → `ctrl1.rdbfien`), update accordingly.

- [ ] **Step 5: Inspect CRM/PLL macro names**

```bash
grep -n "CRM_PLL_MULT\|CRM_CLOCK_SOURCE\|CRM_PLL_SOURCE\|CRM_AHB_DIV\|CRM_APB\|crm_hext_stable_wait\|crm_auto_step_mode" \
  vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/drivers/inc/at32f403a_407_crm.h | head -30
```

Compare against the names used in `port_sys.c`. The AT32F403A_407 CRM module may have different multiplier constants (e.g., `CRM_PLL_MULT_60` may not exist — the library may use direct register writes instead). Adjust `SystemClock_Config` accordingly.

- [ ] **Step 6: Inspect startup file for IRQ handler names**

```bash
grep -i "word\|handler\|irq" \
  vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f403a_407.s | head -80
```

Compare IRQ handler names (e.g., `USART1_IRQHandler`, `SysTick_Handler`) against the names used in `at32f407_it.c`. Adjust stub handler names if the startup file uses different names.

- [ ] **Step 7: Rebuild and verify after fixups**

```bash
mingw32-make TARGET_CHIP=at32f407 clean
mingw32-make TARGET_CHIP=at32f407
```

Expected: clean build after all naming fixups applied.

- [ ] **Step 8: Final regression check on existing targets**

```bash
mingw32-make TARGET_CHIP=stm32g431 clean && mingw32-make TARGET_CHIP=stm32g431 && \
mingw32-make TARGET_CHIP=at32f413 clean && mingw32-make TARGET_CHIP=at32f413 && \
mingw32-make TARGET_CHIP=at32f421 clean && mingw32-make TARGET_CHIP=at32f421
```

Expected: all three build without errors.
