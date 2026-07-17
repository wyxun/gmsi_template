#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <stdio.h>

#define TEST_CHECK(condition)                                             \
    do {                                                                  \
        if (!(condition)) {                                               \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            nFailures++;                                                  \
        }                                                                 \
    } while (0)

#define TEST_NEAR(actual, expected, tolerance)                             \
    do {                                                                   \
        float fActual = (actual);                                          \
        float fExpected = (expected);                                      \
        if (fabsf(fActual - fExpected) > (tolerance)) {                    \
            printf("FAIL %s:%d: actual=%g expected=%g tolerance=%g\n",    \
                   __FILE__, __LINE__, (double)fActual,                    \
                   (double)fExpected, (double)(tolerance));                \
            nFailures++;                                                   \
        }                                                                  \
    } while (0)

#endif
