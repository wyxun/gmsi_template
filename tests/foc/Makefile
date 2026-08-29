CC ?= clang
CFLAGS = -std=gnu11 -Wall -Wextra -Werror -I. -I../../foc -I../../foc/math \
         -I../../foc/middleware -I../../foc/modulation -I../../foc/control \
         -I../../foc/hal -I../../foc/motor -I../../foc/observer \
         -I../../foc/app -I../../foc/experimental
CFLAGS += -I../../foc/optimization
CFLAGS += -I../../peripheral/driver -I../../peripheral/stm32g431 \
          -I../../modus/src -I../../modus/src/mdi -I../../modus/src/mdebug \
          -I../../modus/src/utilities
FOC_EXPERIMENTAL_NSD ?= 1
FOC_EXPERIMENTAL_IDENTIFY ?= 1
CFLAGS += -I../../foc/experimental \
          -I../../modus/lib/perf_counter \
          -I../../modus/lib/plooc \
          -DMOTOR_ENABLE_TEST_HOOKS=1 \
          -D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1 \
          -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port_stub.h\" \
          -D__NO_USE_LOG__ -D__NO_USE_ASSERT \
          -DFOC_ENABLE_EXPERIMENTAL_NSD=$(FOC_EXPERIMENTAL_NSD) \
          -DFOC_ENABLE_EXPERIMENTAL_IDENTIFY=$(FOC_EXPERIMENTAL_IDENTIFY)
STRICT_ALIAS_CFLAGS ?=
EXTRA_CFLAGS ?=
CFLAGS += $(STRICT_ALIAS_CFLAGS) $(EXTRA_CFLAGS)
TEST_SRCS = test_foc.c test_numeric.c test_transform.c \
            test_modulation.c test_control.c test_motor.c \
            test_motor_encapsulation.c test_motor_fsm.c test_observer.c \
            test_motor_position.c test_motor_control_runtime.c \
            test_motor_observation.c \
            test_advanced.c test_optimization.c test_experimental.c \
            test_trig.c
FOC_SRCS = ../../foc/math/foc_numeric.c ../../foc/math/foc_angle.c \
           ../../foc/math/foc_trig_lut.c \
           ../../foc/math/foc_math.c ../../foc/middleware/foc_core.c \
           ../../foc/modulation/foc_modulation.c \
           ../../foc/control/foc_pid.c ../../foc/control/foc_filter.c \
           ../../foc/control/foc_pll.c ../../foc/control/foc_ltd.c \
           ../../foc/control/foc_feedforward.c \
           ../../foc/control/foc_controller.c \
           ../../foc/control/foc_ladrc.c ../../foc/control/foc_smc.c \
           ../../foc/control/foc_sta.c ../../foc/control/foc_dob.c \
           ../../foc/hal/foc_hal.c \
           ../../foc/motor/motor.c ../../foc/motor/motor_fsm.c \
           ../../foc/motor/motor_control.c ../../foc/motor/motor_hf.c \
           ../../foc/motor/motor_position.c \
           ../../foc/observer/foc_hall.c ../../foc/observer/foc_smo.c \
           ../../foc/observer/foc_nlfo.c \
           ../../foc/observer/foc_observer_selector.c \
           ../../foc/observer/foc_open_loop_source.c
FOC_SRCS += ../../foc/observer/foc_hfi.c
FOC_SRCS += ../../foc/optimization/foc_optimization.c \
            ../../foc/optimization/foc_cogging.c
FOC_SRCS += ../../foc/experimental/foc_experiment.c \
            ../../foc/experimental/foc_nsd.c \
            ../../foc/experimental/foc_identify.c

.PHONY: all float fixed encapsulation minimal minimal-core \
        minimal-core-float minimal-core-fixed minimal-encoder \
        minimal-encoder-float minimal-encoder-fixed \
        minimal-lifecycle minimal-lifecycle-float minimal-lifecycle-fixed \
        clean

MINIMAL_CORE_SRCS = test_foc_minimal_core.c \
                    ../../foc/math/foc_numeric.c \
                    ../../foc/math/foc_angle.c \
                    ../../foc/math/foc_trig_lut.c \
                    ../../foc/math/foc_math.c \
                    ../../foc/middleware/foc_core.c \
                    ../../foc/control/foc_pid.c \
                    ../../foc/modulation/foc_modulation.c

minimal-core: minimal-core-float minimal-core-fixed

minimal-core-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_CORE_SRCS) \
	    -lm -o minimal_core_float
	./minimal_core_float

minimal-core-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_CORE_SRCS) \
	    -lm -o minimal_core_fixed
	./minimal_core_fixed

MINIMAL_ENCODER_SRCS = test_foc_minimal_encoder.c \
                       ../../foc/math/foc_numeric.c \
                       ../../foc/math/foc_angle.c \
                       ../../foc/math/foc_trig_lut.c \
                       ../../foc/math/foc_math.c \
                       ../../foc/observer/foc_encoder.c

minimal-encoder: minimal-encoder-float minimal-encoder-fixed

minimal-encoder-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_ENCODER_SRCS) \
	    -lm -o minimal_encoder_float
	./minimal_encoder_float

minimal-encoder-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_ENCODER_SRCS) \
	    -lm -o minimal_encoder_fixed
	./minimal_encoder_fixed

MINIMAL_LIFECYCLE_SRCS = test_foc_minimal_lifecycle.c \
                         ../../foc/math/foc_numeric.c \
                         ../../foc/math/foc_angle.c \
                         ../../foc/math/foc_trig_lut.c \
                         ../../foc/math/foc_math.c \
                         ../../foc/middleware/foc_core.c \
                         ../../foc/control/foc_pid.c \
                         ../../foc/modulation/foc_modulation.c \
                         ../../foc/observer/foc_encoder.c \
                         ../../foc/app/foc_app.c

minimal-lifecycle: minimal-lifecycle-float minimal-lifecycle-fixed

minimal-lifecycle-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_LIFECYCLE_SRCS) \
	    -lm -o minimal_lifecycle_float
	./minimal_lifecycle_float

minimal-lifecycle-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_LIFECYCLE_SRCS) \
	    -lm -o minimal_lifecycle_fixed
	./minimal_lifecycle_fixed

minimal: minimal-core minimal-encoder minimal-lifecycle

all: encapsulation float fixed

float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -lm -o foc_test_float
	./foc_test_float

fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(TEST_SRCS) $(FOC_SRCS) \
	    -lm -o foc_test_fixed
	./foc_test_fixed

encapsulation:
	@echo "=== checking motor_handle_t encapsulation ==="
	@$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 -c compile_pass_motor_type.c -o compile_pass_motor_type.o || { echo "ERROR: positive motor type probe failed"; exit 2; }
	@rm -f compile_pass_motor_type.o
	@$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 -c compile_fail_motor_member_access.c -o compile_fail_motor_member_access.o 2>compile_fail_motor_member_access.log && { echo "FAIL: motor_handle_t members remain publicly accessible"; rm -f compile_fail_motor_member_access.o compile_fail_motor_member_access.log; exit 1; } || true
	@if grep -qE "has no member named.*tRt|no member named.*tRt" compile_fail_motor_member_access.log; then echo "PASS: motor_handle_t rejects direct member access"; rm -f compile_fail_motor_member_access.o compile_fail_motor_member_access.log; else echo "ERROR: fixture failed for an unexpected reason"; cat compile_fail_motor_member_access.log; exit 2; fi

clean:
	$(RM) foc_test_float foc_test_fixed minimal_core_float minimal_core_fixed \
	      minimal_encoder_float minimal_encoder_fixed \
	      minimal_lifecycle_float minimal_lifecycle_fixed \
	      compile_pass_motor_type.o \
	      compile_fail_motor_member_access.o \
	      compile_fail_motor_member_access.log
