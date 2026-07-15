# =============================================================================
# grblhal_adapt/grblhal_adapt.mk
# Build rules, sources and include paths for grblHAL integration
# =============================================================================

# 1. Include core grblHAL makefile
GRBLHAL_ROOT = third_party/grblhal
include $(GRBLHAL_ROOT)/grblhal.mk

# 2. Append grblHAL source files to CLASS_SOURCES
CLASS_SOURCES += class/grblhal.c
CLASS_SOURCES += $(GRBLHAL_SRCS)
CLASS_SOURCES += $(wildcard grblhal_adapt/*.c)

# 3. Compile settings.c with __SETTINGS_C__ defined
build/settings.o: CFLAGS += -D__SETTINGS_C__

# 4. Append grblHAL include paths and flags to TARGET_INCLUDES / C_DEFS
TARGET_INCLUDES += $(GRBLHAL_INCLUDES) -Igrblhal_adapt
C_DEFS          += $(GRBLHAL_CFLAGS)

# 5. RTT Stream Auto-completion logic (moved from target.mk and root makefile)
# If grblHAL uses RTT stream (not UART) and MSHELL is disabled, compile SEGGER_RTT.c
# We use deferred expansion since MSHELL_ENABLE is defined in the main makefile later.
CHIP_SOURCES += $(if $(filter 0,$(MSHELL_ENABLE)),$(if $(findstring -DGRBLHAL_STREAM_UART=1,$(C_DEFS)),,$(MODUS_ROOT)/src/mdebug/segger_rtt/SEGGER_RTT.c))
