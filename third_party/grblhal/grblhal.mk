# grblhal.mk — grblHAL CNC Controller Build Bridge
#
# Usage in modus_template Makefile:
#   include third_party/grblhal/grblhal.mk
#   C_SOURCES  += $(GRBLHAL_SRCS)
#   C_INCLUDES += $(GRBLHAL_INCLUDES)
#
# Only included when GRBLHAL_ENABLE=1 in target/*/target.mk.

# ---- Root auto-detection ----
GRBLHAL_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
GRBLHAL_CORE  = $(GRBLHAL_ROOT)/core

# ---- Feature switches (conservative defaults, all OFF) ----
GRBLHAL_SPINDLE_SYNC ?= 0
GRBLHAL_SD_CARD     ?= 0
GRBLHAL_CAN         ?= 0
GRBLHAL_MODBUS      ?= 0
GRBLHAL_ENCODERS    ?= 0

# ---- Core sources (always compiled) ----
GRBLHAL_SRCS = \
    $(GRBLHAL_CORE)/alarms.c \
    $(GRBLHAL_CORE)/coolant_control.c \
    $(GRBLHAL_CORE)/crc.c \
    $(GRBLHAL_CORE)/crossbar.c \
    $(GRBLHAL_CORE)/errors.c \
    $(GRBLHAL_CORE)/gcode.c \
    $(GRBLHAL_CORE)/grbllib.c \
    $(GRBLHAL_CORE)/ioports.c \
    $(GRBLHAL_CORE)/machine_limits.c \
    $(GRBLHAL_CORE)/messages.c \
    $(GRBLHAL_CORE)/motion_control.c \
    $(GRBLHAL_CORE)/ngc_expr.c \
    $(GRBLHAL_CORE)/ngc_flowctrl.c \
    $(GRBLHAL_CORE)/ngc_params.c \
    $(GRBLHAL_CORE)/nuts_bolts.c \
    $(GRBLHAL_CORE)/override.c \
    $(GRBLHAL_CORE)/planner.c \
    $(GRBLHAL_CORE)/probe.c \
    $(GRBLHAL_CORE)/protocol.c \
    $(GRBLHAL_CORE)/regex.c \
    $(GRBLHAL_CORE)/report.c \
    $(GRBLHAL_CORE)/settings.c \
    $(GRBLHAL_CORE)/sleep.c \
    $(GRBLHAL_CORE)/spindle_control.c \
    $(GRBLHAL_CORE)/state_machine.c \
    $(GRBLHAL_CORE)/stepper.c \
    $(GRBLHAL_CORE)/stream.c \
    $(GRBLHAL_CORE)/system.c \
    $(GRBLHAL_CORE)/tool_change.c \
    $(GRBLHAL_CORE)/nvs_buffer.c \
    $(GRBLHAL_CORE)/strutils.c \
    $(GRBLHAL_CORE)/vfs.c \
    $(GRBLHAL_CORE)/utf8.c


# ---- Optional: CAN bus ----
ifeq ($(GRBLHAL_CAN),1)
GRBLHAL_SRCS += $(GRBLHAL_CORE)/canbus.c
endif

# ---- Optional: Modbus RTU ----
ifeq ($(GRBLHAL_MODBUS),1)
GRBLHAL_SRCS += $(GRBLHAL_CORE)/modbus.c $(GRBLHAL_CORE)/modbus_rtu.c
endif

# ---- Optional: Encoders ----
ifeq ($(GRBLHAL_ENCODERS),1)
GRBLHAL_SRCS += $(GRBLHAL_CORE)/encoders.c
endif

# ---- Optional: SD card streaming ----
ifeq ($(GRBLHAL_SD_CARD),1)
GRBLHAL_SRCS += $(GRBLHAL_CORE)/stream_file.c $(GRBLHAL_CORE)/fs_device.c
endif

# ---- Optional: Spindle sync / PID ----
ifeq ($(GRBLHAL_SPINDLE_SYNC),1)
GRBLHAL_SRCS += $(GRBLHAL_CORE)/pid.c
endif

# ---- Include paths ----
GRBLHAL_INCLUDES = \
    -I$(GRBLHAL_CORE) \
    -I$(GRBLHAL_CORE)/kinematics

# ---- Compile flags ----
GRBLHAL_CFLAGS = -DGRBLHAL_ENABLE=1 -include grblhal_config.h

