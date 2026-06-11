# =============================================================================
# target/ch592/target.mk
# WeAct Studio CH592F — RISC-V 32-bit QingKe V4C 内核
# =============================================================================

# CPU architecture flags for RISC-V RV32IMAC
CPU_FLAGS = -march=rv32imac -mabi=ilp32

# Chip preprocessor defines
C_DEFS += -DCH592 -D__riscv

# CMSIS / Peripheral Library Paths
CHIPLIB_ROOT = vendor/riscv/ch592/EVT/EXAM/SRC
SYS_INC_PATH = D:/software/llvm_for_arm/lib/clang-runtimes/arm-none-eabi/armv7m_soft_nofp/include
TARGET_INCLUDES = -I$(CHIPLIB_ROOT)/StdPeriphDriver/inc -I$(CHIPLIB_ROOT)/RVMSIS -Itarget/ch592 -I$(SYS_INC_PATH)

# Linker script, startup, interrupt handler, perf_counter port
LDSCRIPT     = target/ch592/CH592_FLASH.ld
STARTUP_S    = $(CHIPLIB_ROOT)/Startup/startup_CH592.S target/ch592/ch592_exception.S
IT_C         = target/ch592/ch592_it.c

# Overwrite Cortex-M debug sources to empty (RISC-V debug is inside modus framework core)
CORE_DEBUG_SOURCES =

# Chip core sources (scanned automatically via PERIPHERAL_SOURCES)
CHIP_SOURCES =

# OpenOCD (WCH Custom OpenOCD path or standard openocd with wlink support)
# WCH MounRiver Studio typically installs openocd.exe supporting wlink
OPENOCD_BIN     ?= $(SW_ROOT)/msys64/mingw64/bin/openocd.exe
OPENOCD_SCRIPTS ?= $(SW_ROOT)/msys64/mingw64/share/openocd/scripts

# Use LLVM LLD linker for cross-architecture ELF linking, disable standard C library dependency
STD_LIBS =
LDFLAGS += -fuse-ld=lld -nostdlib

# ------------------------------------------------------------------------------
# Flash configuration for CH592 (prioritizes community 'wchisp', falls back to official 'WCHISPTool_CMD')
# ------------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
    WCHISP_PATH := $(shell where wchisp 2>nul)
    WCHCMD_PATH := $(shell where WCHISPTool_CMD 2>nul)
    
    ifneq ($(WCHISP_PATH),)
        FLASH_CMD = wchisp flash $<
    else ifneq ($(WCHCMD_PATH),)
        CONFIG_INI = target/ch592/config.ini
        ifeq ($(wildcard $(CONFIG_INI)),)
            FLASH_CMD = @echo [ERROR] WCHISPTool_CMD requires target/ch592/config.ini && \
                        echo Please open WchIspStudio GUI, select CH592, configure and click 'File -> Save UI Config' to save as 'target/ch592/config.ini' && \
                        exit 1
        else
            FLASH_CMD = WCHISPTool_CMD -p USB -c $(CONFIG_INI) -o program -f $<
        endif
    else
        FLASH_CMD = @echo [ERROR] No flashing tool found. Please download 'wchisp.exe' (https://github.com/ch32-rs/wchisp) and add it to your PATH, or download official 'WCHISPTool_CMD.exe' and add it to your PATH. && \
                    exit 1
    endif
else
    # Linux / macOS
    WCHISP_PATH := $(shell which wchisp 2>/dev/null)
    WCHCMD_PATH := $(shell which WCHISPTool_CMD 2>/dev/null)
    
    ifneq ($(WCHISP_PATH),)
        FLASH_CMD = wchisp flash $<
    else ifneq ($(WCHCMD_PATH),)
        CONFIG_INI = target/ch592/config.ini
        ifeq ($(wildcard $(CONFIG_INI)),)
            FLASH_CMD = @echo "[ERROR] WCHISPTool_CMD requires target/ch592/config.ini" && \
                        echo "Please open WchIspStudio GUI, select CH592, configure and click 'File -> Save UI Config' to save as 'target/ch592/config.ini'" && \
                        exit 1
        else
            # Linux USB node defaults to /dev/ch37x
            FLASH_CMD = WCHISPTool_CMD -p /dev/ch37x -c $(CONFIG_INI) -o program -f $<
        endif
    else
        FLASH_CMD = @echo "[ERROR] No flashing tool found (wchisp or WCHISPTool_CMD)" && \
                    exit 1
    endif
endif

