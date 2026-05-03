# =============================================================================
# target/stm32g431/target.mk
# STM32G431xx — Cortex-M4F, single-precision FPU
# =============================================================================

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DSTM32G431xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DFOC_SUPPORT=1

# CMSIS paths
CMSIS_CORE = chiplib/cmsis_core
CMSIS_DEV  = chiplib/cmsis_device_g4/Include

# HAL paths
HAL_INC = chiplib/stm32g4xx_hal_driver/Inc
HAL_SRC = chiplib/stm32g4xx_hal_driver/Src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/stm32g431/STM32G431xB_FLASH.ld
STARTUP_S    = chiplib/cmsis_device_g4/Source/Templates/gcc/startup_stm32g431xx.s
SYSTEM_C     = chiplib/cmsis_device_g4/Source/Templates/system_stm32g4xx.c
IT_C         = target/stm32g431/stm32g4xx_it.c
PERFC_PORT_C = target/stm32g431/perfc_port_user.c

# Chip-specific include paths
TARGET_INCLUDES = -I$(HAL_INC) -Itarget/stm32g431

# HAL sources (all peripherals, excluding template files)
HAL_SOURCES = $(filter-out %_template.c, $(wildcard $(HAL_SRC)/*.c))

CHIP_SOURCES = $(HAL_SOURCES)

# FOC framework — STM32G431 supports FOC
FOC_SOURCES = $(wildcard foc/math/*.c)       \
              $(wildcard foc/hal/*.c)         \
              $(wildcard foc/motor/*.c)       \
              $(wildcard foc/middleware/*.c)  \
              $(wildcard foc/app/*.c)
