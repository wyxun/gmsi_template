#include "test_common.h"
#include "foc_core.h"

int test_transform(void)
{
    static const float s_fAngles[] = {
        0.0f, 0.125f, 0.25f, 0.5f, 0.875f,
    };
    int nFailures = 0;
    unsigned int wIndex;
#if defined(FOC_NUMERIC_FLOAT)
    const float fTolerance = 0.0002f;
#else
    const float fTolerance = 0.008f;
#endif
    foc_ab_t tAB = {
        .qAlpha = FOC_SCALAR(0.35f),
        .qBeta = FOC_SCALAR(-0.20f),
    };
    foc_ab_t tClarke;

    TEST_CHECK(foc_clarke(FOC_SCALAR(1.0f), FOC_SCALAR(-0.5f),
                          FOC_SCALAR(-0.5f), &tClarke) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(tClarke.qAlpha), 1.0f, fTolerance);
    TEST_NEAR(foc_to_float(tClarke.qBeta), 0.0f, fTolerance);
    TEST_CHECK(foc_clarke(FOC_ZERO, FOC_ZERO, FOC_ZERO, NULL) ==
               FOC_RESULT_NULL);

    for (wIndex = 0; wIndex < sizeof(s_fAngles) / sizeof(s_fAngles[0]);
         wIndex++) {
        foc_dq_t tDQ;
        foc_ab_t tRoundTrip;
        foc_angle_t tAngle = foc_angle_from_turns(s_fAngles[wIndex]);

        TEST_CHECK(foc_park(&tAB, tAngle, &tDQ) == FOC_RESULT_OK);
        TEST_CHECK(foc_ipark(&tDQ, tAngle, &tRoundTrip) == FOC_RESULT_OK);
        TEST_NEAR(foc_to_float(tRoundTrip.qAlpha), 0.35f, fTolerance);
        TEST_NEAR(foc_to_float(tRoundTrip.qBeta), -0.20f, fTolerance);
    }

    return nFailures;
}
