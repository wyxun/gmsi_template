#include "test_common.h"
#include "foc_angle.h"
#include "foc_trig.h"
#include <math.h>

#if defined(FOC_NUMERIC_FLOAT)
#define EPSILON 2e-6f

int test_trig(void)
{
    int nFailures = 0;

    /* 1. 精度和逐点扫描验证 */
    const int steps = 65536;
    float max_err = 0.0f;
    for (int i = 0; i < steps; i++) {
        float turns = (float)i / (float)steps;
        float ref_sin = sinf(turns * 2.0f * 3.141592653589793f);
        float lut_sin = lut_sin_turns(turns);
        float err = fabsf(ref_sin - lut_sin);
        if (err > max_err) {
            max_err = err;
        }
        TEST_NEAR(lut_sin, ref_sin, EPSILON);
    }
    printf("  [LUT Accuracy] Max error over %d steps: %e (threshold: %e)\n",
           steps, (double)max_err, (double)EPSILON);

    /* 2. 特殊角度精确命中测试 */
    TEST_NEAR(lut_sin_turns(0.0f), 0.0f, 1e-9f);
    TEST_NEAR(lut_sin_turns(0.25f), 1.0f, 1e-9f);
    TEST_NEAR(lut_sin_turns(0.5f), 0.0f, 1e-9f);
    TEST_NEAR(lut_sin_turns(0.75f), -1.0f, 1e-9f);

    /* 3. angle_wrap_scalar 的截断与归一等价性 */
    float test_cases[] = {
        0.0f, 0.123f, 0.999f,
        1.0f, 1.25f, 2.75f,
        -0.25f, -1.0f, -1.75f, -3.0f
    };
    for (size_t i = 0; i < sizeof(test_cases)/sizeof(test_cases[0]); i++) {
        float input = test_cases[i];
        float ref = input - floorf(input);
        if (ref < 0.0f) ref += 1.0f;
        if (ref >= 1.0f) ref = 0.0f; // wrap 归一契约

        foc_angle_t tIn = { input };
        foc_angle_t tOut = foc_angle_wrap(tIn);
        TEST_NEAR(tOut.qTurns, ref, 1e-7f);
    }

    return nFailures;
}
#else
int test_trig(void)
{
    return 0;
}
#endif
