#include <stdio.h>

int test_numeric(void);
int test_transform(void);
int test_modulation(void);
int test_control(void);
int test_motor(void);
int test_motor_encapsulation(void);
int test_motor_fsm(void);
int test_observer(void);
int test_motor_position(void);
int test_motor_control_runtime(void);
int test_motor_observation(void);
int test_advanced(void);
int test_optimization(void);
int test_experimental(void);
int test_trig(void);

int main(void)
{
    int nFailures = 0;

    nFailures += test_numeric();
    nFailures += test_transform();
    nFailures += test_modulation();
    nFailures += test_control();
    nFailures += test_motor();
    nFailures += test_motor_encapsulation();
    nFailures += test_motor_fsm();
    nFailures += test_observer();
    nFailures += test_motor_position();
    nFailures += test_motor_control_runtime();
    nFailures += test_motor_observation();
    nFailures += test_advanced();
    nFailures += test_optimization();
    nFailures += test_experimental();
    nFailures += test_trig();

    printf("FOC tests: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);

    return nFailures == 0 ? 0 : 1;
}
