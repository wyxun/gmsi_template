CC ?= clang
CFLAGS = -std=c11 -Wall -Wextra -Werror -I../../foc -I../../foc/math \
         -I../../foc/middleware -I../../foc/modulation -I../../foc/control \
         -I../../foc/hal -I../../foc/motor -I../../foc/observer
CFLAGS += -I../../foc/optimization
FOC_EXPERIMENTAL_NSD ?= 1
FOC_EXPERIMENTAL_IDENTIFY ?= 1
CFLAGS += -I../../foc/experimental \
          -DFOC_ENABLE_EXPERIMENTAL_NSD=$(FOC_EXPERIMENTAL_NSD) \
          -DFOC_ENABLE_EXPERIMENTAL_IDENTIFY=$(FOC_EXPERIMENTAL_IDENTIFY)
TEST_SRCS = test_foc.c test_numeric.c test_transform.c \
            test_modulation.c test_control.c test_motor.c test_observer.c \
            test_advanced.c test_optimization.c test_experimental.c
FOC_SRCS = ../../foc/math/foc_numeric.c ../../foc/math/foc_angle.c \
           ../../foc/math/foc_math.c ../../foc/middleware/foc_core.c \
           ../../foc/modulation/foc_modulation.c \
           ../../foc/control/foc_pid.c ../../foc/control/foc_filter.c \
           ../../foc/control/foc_pll.c ../../foc/control/foc_ltd.c \
           ../../foc/control/foc_feedforward.c \
           ../../foc/control/foc_controller.c \
           ../../foc/control/foc_ladrc.c ../../foc/control/foc_smc.c \
           ../../foc/control/foc_sta.c ../../foc/control/foc_dob.c \
           ../../foc/hal/foc_hal.c \
           ../../foc/motor/motor.c ../../foc/motor/motor_control.c \
           ../../foc/observer/foc_observer.c \
           ../../foc/observer/foc_hall.c ../../foc/observer/foc_smo.c \
           ../../foc/observer/foc_nlfo.c \
           ../../foc/observer/foc_observer_selector.c
FOC_SRCS += ../../foc/observer/foc_hfi.c
FOC_SRCS += ../../foc/optimization/foc_optimization.c \
            ../../foc/optimization/foc_cogging.c
FOC_SRCS += ../../foc/experimental/foc_experiment.c \
            ../../foc/experimental/foc_nsd.c \
            ../../foc/experimental/foc_identify.c

.PHONY: all float fixed clean

all: float fixed

float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -lm -o foc_test_float
	./foc_test_float

fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -lm -o foc_test_fixed
	./foc_test_fixed

clean:
	$(RM) foc_test_float foc_test_fixed
