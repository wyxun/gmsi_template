#include <stdio.h>

int test_numeric(void);
int test_transform(void);
int test_modulation(void);
int test_control(void);
int test_motor(void);
int test_observer(void);
int test_advanced(void);
int test_optimization(void);
int test_experimental(void);

int main(void)
{
    int nFailures = 0;

    nFailures += test_numeric();
    nFailures += test_transform();
    nFailures += test_modulation();
    nFailures += test_control();
    nFailures += test_motor();
    nFailures += test_observer();
    nFailures += test_advanced();
    nFailures += test_optimization();
    nFailures += test_experimental();

    printf("FOC tests: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);

    return nFailures == 0 ? 0 : 1;
}
