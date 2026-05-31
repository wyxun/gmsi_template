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
