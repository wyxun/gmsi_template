# ==============================================================================
# Project Makefile — Multi-chip support via target/$(TARGET_CHIP)/
# Toolchain : LLVM Embedded Toolchain for Arm (Windows)
# Usage     : make [BUILD=debug|release] [TARGET_CHIP=stm32g431] ...
# ==============================================================================
SW_ROOT ?= D:/software
MSYS2_BIN = $(SW_ROOT)/msys64/mingw64/bin
MAKE      = $(MSYS2_BIN)/mingw32-make.exe

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
TARGET = template
TARGET_CHIP ?= stm32g431

include target/$(TARGET_CHIP)/target.mk

# ------------------------------------------------------------------------------
# Toolchain  (Windows LLVM path)
# ------------------------------------------------------------------------------
LLVM_PATH = $(SW_ROOT)/llvm_for_arm/bin/
CC  = $(LLVM_PATH)clang
AS  = $(LLVM_PATH)clang
CP  = $(LLVM_PATH)llvm-objcopy
SZ  = $(LLVM_PATH)llvm-size
LD  = $(LLVM_PATH)clang
AR  = $(LLVM_PATH)llvm-ar

# ------------------------------------------------------------------------------
# CPU / Architecture
# ------------------------------------------------------------------------------
TARGET_TRIPLE = --target=armv7em-none-eabi

# ------------------------------------------------------------------------------
# Directories
# ------------------------------------------------------------------------------
BUILD_DIR = build

ifeq ($(OS),Windows_NT)
    SHELL = cmd.exe
    MKDIR = if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)
    RMDIR = if exist $(BUILD_DIR) rd /s /q $(BUILD_DIR)
else
    MKDIR = mkdir -p $(BUILD_DIR)
    RMDIR = rm -rf $(BUILD_DIR)
endif

# ------------------------------------------------------------------------------
# MODUS framework
# ------------------------------------------------------------------------------
MODUS_ROOT = modus

# Module switches
MSHELL_ENABLE    = 1
MWAVEFORM_ENABLE = 1
MSTORAGE_ENABLE  = 1
MBLINFO_ENABLE   = 1
MODUS_USE_LOG    = 1

include $(MODUS_ROOT)/modus.mk

LIB_PERF_DIR  = $(MODUS_ROOT)/lib/perf_counter
LIB_PLOOC_DIR = $(MODUS_ROOT)/lib/plooc

# MODUS sources are automatically handled via MODUS_SRCS from modus.mk
PERIF_LIB_SOURCES = \
    $(LIB_PERF_DIR)/perf_counter.c

PERIPHERAL_SOURCES = $(wildcard peripheral/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/$(TARGET_CHIP)/*.c)
CLASS_SOURCES      = $(wildcard class/*.c) $(wildcard class/*/*.c)

# FOC framework — defined per-chip in target.mk (not all chips support FOC)
FOC_SOURCES ?=

# ------------------------------------------------------------------------------
# All C sources  (chip-specific vars set by target.mk)
# ------------------------------------------------------------------------------
C_SOURCES = \
    src/main.c \
    $(PERFC_PORT_C) \
    $(IT_C) \
    $(SYSTEM_C) \
    $(CHIP_SOURCES) \
    $(MODUS_SRCS) \
    $(PERIF_LIB_SOURCES) \
    $(PERIPHERAL_SOURCES) \
    $(CLASS_SOURCES) \
    $(FOC_SOURCES)

ASM_SOURCES = $(STARTUP_S)

# ------------------------------------------------------------------------------
# Defines
# ------------------------------------------------------------------------------
C_DEFS += \
    -D__PERFC_USE_USER_CUSTOM_PORTING__=1 \
    -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1 \
    -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port_user.h\" \
    -D__COMPILER_HAS_GNU_EXTENSIONS__=1 \
    -DTRACE_USE_LIBC_PRINTF=0 \
    -DTRACE_MCU_WRITE_STRING="extern void user_trace_output(const char*); user_trace_output" \
    -DMODUS_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""

# ------------------------------------------------------------------------------
# Include paths
# ------------------------------------------------------------------------------
C_INCLUDES = \
    -I. \
    -Isrc \
    -I$(CMSIS_CORE) \
    -I$(CMSIS_DEV) \
    $(TARGET_INCLUDES) \
    $(MODUS_INCLUDES) \
    -I$(MODUS_ROOT)/src/utilities \
    -I$(MODUS_ROOT)/src/mdebug \
    -I$(MODUS_ROOT)/src/mdebug/segger_rtt \
    -I$(LIB_PLOOC_DIR) \
    -I$(LIB_PERF_DIR) \
    -Iperipheral \
    -Iperipheral/$(TARGET_CHIP) \
    -Iclass \
    -Ifoc \
    -Ifoc/math \
    -Ifoc/hal \
    -Ifoc/motor \
    -Ifoc/middleware \
    -Ifoc/app

# ------------------------------------------------------------------------------
# Build mode: BUILD=debug (default) | BUILD=release
# ------------------------------------------------------------------------------
BUILD ?= debug
ifeq ($(BUILD),release)
    OPT = -Oz
    C_DEFS += -D__NO_USE_LOG__
else
    OPT = -O0
endif

# ------------------------------------------------------------------------------
# Flags
# ------------------------------------------------------------------------------
$(info C_DEFS is $(C_DEFS))
CFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS) $(C_DEFS) $(C_INCLUDES) $(MODUS_CFLAGS) $(OPT) -g
CFLAGS += -Wall -Wextra -std=gnu11
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-exceptions
CFLAGS += -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare \
          -Wno-compare-distinct-pointer-types -Wno-unused-command-line-argument

ASFLAGS = $(TARGET_TRIPLE) $(CPU_FLAGS) -g

LDFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -lcrt0 -lc -lm

# ------------------------------------------------------------------------------
.PHONY: all clean size flash rtt
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin size

release:
	$(MAKE) BUILD=release

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -R .ARM.attributes $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex -R .ARM.attributes $< $@

$(BUILD_DIR):
	$(MKDIR)

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

clean:
	$(RMDIR)

# ------------------------------------------------------------------------------
# Flash / Debug
# ------------------------------------------------------------------------------

ifeq ($(OS),Windows_NT)
    OPENOCD_BIN     ?= $(SW_ROOT)/msys64/mingw64/bin/openocd.exe
    OPENOCD_SCRIPTS ?= $(SW_ROOT)/msys64/mingw64/share/openocd/scripts
    OPENOCD_CMD = $(OPENOCD_BIN) -s $(OPENOCD_SCRIPTS) -f target/$(TARGET_CHIP)/openocd.cfg -c "adapter speed 2000" -c "tcl_port 0"
else
    OPENOCD_BIN = openocd
    OPENOCD_CMD = $(OPENOCD_BIN) -f target/$(TARGET_CHIP)/openocd.cfg
endif

flash: $(BUILD_DIR)/$(TARGET).hex
	$(OPENOCD_CMD) -c "reset_config connect_assert_srst" -c "init" -c "reset halt" -c "sleep 200" -c "program $< verify" -c "reset run" -c "exit"

debug-server:

	$(OPENOCD_CMD)

openocd: debug-server

debug: $(BUILD_DIR)/$(TARGET).elf
	gdb-multiarch -x debug.gdb $<

# ------------------------------------------------------------------------------
# RTT
# ------------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
    RTT_ADDR = $(shell powershell -NoProfile -Command "$$nm = & '$(LLVM_PATH)llvm-nm.exe' $(BUILD_DIR)/$(TARGET).elf 2>$$null | Select-String '_SEGGER_RTT$$'; if ($$nm) { '0x' + ($$nm -split ' ')[0] }" 2>nul)
else
    RTT_ADDR = $(shell $(LLVM_PATH)llvm-nm $(BUILD_DIR)/$(TARGET).elf 2>/dev/null | awk '/_SEGGER_RTT$$/ {print "0x"$$1}')
endif

rtt-addr: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"

rtt: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"
	$(OPENOCD_CMD) -c "init" \
	    -c "rtt setup $(RTT_ADDR) 0xa8 \"SEGGER RTT\"" \
	    -c "rtt start" \
	    -c "rtt server start 9090 0" \
	    -c "rtt server start 9091 1"

# Flash via already-running OpenOCD telnet (port 4444)
flash-rtt: $(BUILD_DIR)/$(TARGET).hex
ifeq ($(OS),Windows_NT)
	@powershell -NoProfile -Command \
	    "$$hex = (Resolve-Path '$<').Path; \
	     $$c = New-Object System.Net.Sockets.TcpClient('localhost', 4444); \
	     $$s = $$c.GetStream(); \
	     $$w = New-Object System.IO.StreamWriter($$s); \
	     $$w.WriteLine(\"program $$hex verify reset\"); $$w.WriteLine('exit'); \
	     $$w.Flush(); Start-Sleep 2; $$c.Close()"
else
	@ABS=$$(realpath $<); \
	printf "program $$ABS verify reset\nexit\n" | nc -w 10 localhost 4444; \
	sleep 1; \
	printf "rtt stop\nrtt start\n" | nc -w 3 localhost 4444
endif

# ------------------------------------------------------------------------------
# Info
# ------------------------------------------------------------------------------
info:
	@echo "TARGET      = $(TARGET)"
	@echo "TARGET_CHIP = $(TARGET_CHIP)"
	@echo "BUILD_DIR   = $(BUILD_DIR)"
	@echo "LDSCRIPT    = $(LDSCRIPT)"
	@echo "LLVM_PATH   = $(LLVM_PATH)"
	@echo "C_SOURCES   = $(C_SOURCES)"
	@echo "ASM_SOURCES = $(ASM_SOURCES)"

.PHONY: all release clean flash flash-rtt debug-server debug size info rtt rtt-addr openocd
