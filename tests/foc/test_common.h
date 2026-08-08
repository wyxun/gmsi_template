#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <stdio.h>

#include "foc_hal.h"

/* 共用的高频 Fast Path 测试桩：motor_Init 要求有效的 tHfIo。 */
static inline foc_result_t test_hf_sample_ok(void *pContext,
                                             phase_current_handle_t *ptCurrent)
{
    (void)pContext;
    ptCurrent->qIu = FOC_ZERO;
    ptCurrent->qIv = FOC_ZERO;
    ptCurrent->qIw = FOC_ZERO;
    return FOC_RESULT_OK;
}

static inline foc_result_t test_hf_commit_ok(void *pContext,
                                             const foc_duty_abc_t *ptDuty)
{
    (void)pContext;
    (void)ptDuty;
    return FOC_RESULT_OK;
}

static inline void test_hf_stop_ok(void *pContext)
{
    (void)pContext;
}

static inline foc_hf_io_if_t test_hf_io(void *pContext)
{
    foc_hf_io_if_t tIo = {
        .wAbiVersion = FOC_HF_IO_ABI_VERSION,
        .pIoContext = pContext,
        .fnSampleCurrent = test_hf_sample_ok,
        .fnCommitDuty = test_hf_commit_ok,
        .fnEmergencyStop = test_hf_stop_ok,
    };
    return tIo;
}

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
