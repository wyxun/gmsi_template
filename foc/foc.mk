# =============================================================================
# Modus FOC — source and include configuration
#
# Include this from target/<chip>/target.mk for chips that support FOC.
# Chips that do not support FOC skip the include; FOC_SOURCES stays empty
# (guarded by "FOC_SOURCES ?=" in the top-level makefile) and the FOC
# include paths default to nothing.
# =============================================================================

FOC_INCLUDES = -Ifoc \
               -Ifoc/math -Ifoc/hal -Ifoc/motor \
               -Ifoc/middleware -Ifoc/control \
               -Ifoc/modulation -Ifoc/observer \
               -Ifoc/optimization -Ifoc/experimental \
               -Ifoc/app

FOC_SOURCES = $(filter-out foc/app/phase_test.c \
                             foc/experimental/foc_verify.c, \
               $(wildcard foc/math/*.c)       \
               $(wildcard foc/hal/*.c)         \
               $(wildcard foc/motor/*.c)       \
               $(wildcard foc/middleware/*.c)  \
               $(wildcard foc/control/*.c)     \
               $(wildcard foc/modulation/*.c)  \
               $(wildcard foc/observer/*.c)    \
               $(wildcard foc/optimization/*.c) \
               $(wildcard foc/experimental/*.c) \
               $(wildcard foc/app/*.c))

# Hardware bring-up diagnostics are opt-in and excluded from production builds.
ifeq ($(FOC_DIAGNOSTIC),1)
FOC_SOURCES += $(wildcard foc/diagnostic/*.c)
endif
