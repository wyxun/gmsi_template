# ==============================================================================
# Project Makefile — AT32F421F8P7
# Toolchain : LLVM Embedded Toolchain for Arm (Windows)
# Usage     : make [BUILD=debug|release] [USE_DRV_UART=1] [SW_ROOT=D:/software] ...
# ==============================================================================
SW_ROOT ?= D:/0_software
MSYS2_BIN = $(SW_ROOT)/msys64/mingw64/bin
MAKE      = $(MSYS2_BIN)/mingw32-make.exe

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
TARGET = template
TARGET_CHIP ?= at32f421

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
# AT32F421 = Cortex-M4, NO FPU (confirmed by at32f421.h: __FPU_PRESENT=0)
# ------------------------------------------------------------------------------
TARGET_TRIPLE = --target=armv7em-none-eabi
CPU_FLAGS     = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft

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

# ---- Chip library paths (all inside chiplib/AT32F421_Firmware_Library) -------
ifeq ($(TARGET_CHIP),at32f421)
CHIPLIB_ROOT  = chiplib/AT32F421_Firmware_Library/libraries

CMSIS_CORE    = $(CHIPLIB_ROOT)/cmsis/cm4/core_support
CMSIS_DEV     = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
STARTUP_S     = $(CHIPLIB_ROOT)/cmsis/cm4/device_support/startup/gcc/startup_at32f421.s

DRV_INC       = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC       = $(CHIPLIB_ROOT)/drivers/src

# ---- GMSI framework -----------------------------------------------------------
GMSI_DIR      = gmsi/gmsi
GMSI_UTL_DIR  = gmsi/gmsi/utilities
LIB_PERF_DIR  = gmsi/lib/perf_counter
LIB_PLOOC_DIR = gmsi/lib/plooc
LIB_SEGGER    = $(GMSI_UTL_DIR)/segger_rtt

# ------------------------------------------------------------------------------
# AT32F421 Peripheral Driver selection  (set to 1 to include)
# 仅包含当前骨架工程需要的最小集合，后续按需开启
# ------------------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_SCFG   ?= 0
USE_DRV_TMR    ?= 0
USE_DRV_USART  ?= 1
USE_DRV_SPI    ?= 0
USE_DRV_I2C    ?= 0
USE_DRV_DMA    ?= 0
USE_DRV_ADC    ?= 0
USE_DRV_CMP    ?= 0
USE_DRV_DEBUG  ?= 1
USE_DRV_EXINT  ?= 0
USE_DRV_WDT    ?= 0
USE_DRV_WWDT   ?= 0
USE_DRV_CRC    ?= 0
USE_DRV_PWC    ?= 0

# ------------------------------------------------------------------------------
# Driver source accumulation
# ------------------------------------------------------------------------------
DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_flash.c
endif
ifeq ($(USE_DRV_SCFG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_scfg.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_tmr.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_usart.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_i2c.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_dma.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_adc.c
endif
ifeq ($(USE_DRV_CMP),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_cmp.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_debug.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_exint.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_wwdt.c
endif
ifeq ($(USE_DRV_CRC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_crc.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_pwc.c
endif

endif
    
# ------------------------------------------------------------------------------
# GMSI / Middleware sources
# ------------------------------------------------------------------------------
GMSI_SOURCES = \
    $(GMSI_DIR)/gmsi.c \
    $(GMSI_DIR)/gbase.c \
    $(GMSI_DIR)/gblinfo.c \
    $(GMSI_DIR)/gcoroutine.c \
    $(GMSI_DIR)/glog.c \
    $(GMSI_DIR)/gstorage.c \
    $(GMSI_UTL_DIR)/list.c \
    $(GMSI_UTL_DIR)/util_queue.c \
    $(GMSI_UTL_DIR)/trace.c \
    $(GMSI_UTL_DIR)/trace_fmt.c \
    $(LIB_SEGGER)/SEGGER_RTT.c \
    $(LIB_PERF_DIR)/perf_counter.c

PERIPHERAL_SOURCES = $(wildcard peripheral/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/$(TARGET_CHIP)/*.c)
CLASS_SOURCES      = $(wildcard class/*.c) $(wildcard class/*/*.c)

# FOC framework — per-layer wildcard
FOC_SOURCES = $(wildcard foc/math/*.c)       \
              $(wildcard foc/hal/*.c)         \
              $(wildcard foc/motor/*.c)       \
              $(wildcard foc/middleware/*.c)  \
              $(wildcard foc/app/*.c)

# ------------------------------------------------------------------------------
# All C sources
# ------------------------------------------------------------------------------
C_SOURCES = \
    src/main.c \
    src/perfc_port_user.c \
    src/at32f421_it.c \
    $(CMSIS_DEV)/system_at32f421.c \
    $(DRV_SOURCES) \
    $(GMSI_SOURCES) \
    $(PERIPHERAL_SOURCES) \
    $(CLASS_SOURCES) \
    $(FOC_SOURCES)

ASM_SOURCES = $(STARTUP_S)

# ------------------------------------------------------------------------------
# Defines
# ------------------------------------------------------------------------------
C_DEFS = \
    -D__PERFC_USE_USER_CUSTOM_PORTING__=1 \
    -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1 \
    -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port_user.h\" \
    -D__COMPILER_HAS_GNU_EXTENSIONS__=1 \
    -DTRACE_USE_LIBC_PRINTF=0 \
    -DTRACE_MCU_WRITE_STRING="extern void user_trace_output(const char*); user_trace_output"

# ------------------------------------------------------------------------------
# Include paths
# ------------------------------------------------------------------------------
C_INCLUDES = \
    -I. \
    -Isrc \
    -I$(CMSIS_CORE) \
    -I$(CMSIS_DEV) \
    -I$(DRV_INC) \
    -I$(GMSI_DIR) \
    -I$(GMSI_UTL_DIR) \
    -I$(LIB_SEGGER) \
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
ifeq ($(TARGET_CHIP),at32f421)
C_DEFS  += -DAT32F421F8P7 -DUSE_STDPERIPH_DRIVER
LDSCRIPT = AT32F421x8_FLASH.ld
endif

CFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS) $(C_DEFS) $(C_INCLUDES) $(OPT) -g
CFLAGS += -Wall -Wextra -std=gnu11
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-exceptions
CFLAGS += -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare \
          -Wno-compare-distinct-pointer-types -Wno-unused-command-line-argument

ASFLAGS = $(TARGET_TRIPLE) $(CPU_FLAGS) -g

# AT32F421F8P7: 64K Flash, 16K RAM (x8 density → use AT32F421x8_FLASH.ld)
LDFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -lcrt0 -lc -lm

# ------------------------------------------------------------------------------
# Build rules
# ------------------------------------------------------------------------------
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
    OPENOCD_BIN     = $(SW_ROOT)/msys64/mingw64/bin/openocd.exe
    OPENOCD_SCRIPTS = $(SW_ROOT)/msys64/mingw64/share/openocd/scripts
    #OPENOCD_CMD = $(OPENOCD_BIN) -s $(OPENOCD_SCRIPTS) -f $(OPENOCD_IF) -f $(OPENOCD_TARGET) -c "reset_config none" -c "adapter speed 2000" -c "tcl_port disabled"
    OPENOCD_CMD = $(OPENOCD_BIN) -s $(OPENOCD_SCRIPTS) -f openocd.cfg -c "reset_config none" -c "adapter speed 2000" -c "tcl_port 0"
else
    OPENOCD_BIN = openocd
    OPENOCD_CMD = $(OPENOCD_BIN) -f openocd.cfg
endif

flash: $(BUILD_DIR)/$(TARGET).hex
	$(OPENOCD_CMD) -c "init" -c "reset halt" -c "sleep 200" -c "program $< verify" -c "reset run" -c "exit"

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
	    -c "rtt server start 9090 0"

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
	@echo "BUILD_DIR   = $(BUILD_DIR)"
	@echo "LDSCRIPT    = $(LDSCRIPT)"
	@echo "LLVM_PATH   = $(LLVM_PATH)"
	@echo "DRV_SOURCES = $(DRV_SOURCES)"
	@echo "PERIPHERAL  = $(PERIPHERAL_SOURCES)"
	@echo "CLASS       = $(CLASS_SOURCES)"

.PHONY: all release clean flash flash-rtt debug-server debug size info rtt rtt-addr openocd
