#include "test_common.h"
#include "foc_angle.h"
#include "foc_math.h"
#include "foc_numeric.h"

int test_numeric(void)
{
    int nFailures = 0;
#if defined(FOC_NUMERIC_FLOAT)
    const float fScalarTolerance = 0.00001f;
    const float fTrigTolerance = 0.0001f;
#else
    const float fScalarTolerance = 0.00004f;
    const float fTrigTolerance = 0.005f;
#endif
    foc_scalar_t qResult = FOC_ZERO;
    foc_gain_t tGain;

    TEST_NEAR(foc_to_float(FOC_ZERO), 0.0f, fScalarTolerance);
    TEST_NEAR(foc_to_float(FOC_SCALAR(0.5f)), 0.5f,
              fScalarTolerance);
    TEST_NEAR(foc_to_float(foc_add_sat(FOC_SCALAR(0.25f),
                                        FOC_SCALAR(0.5f))),
              0.75f, fScalarTolerance);
    TEST_NEAR(foc_to_float(foc_mul_pu(FOC_SCALAR(0.5f),
                                       FOC_SCALAR(-0.25f))),
              -0.125f, fScalarTolerance);
    TEST_NEAR(foc_to_float(foc_sat(FOC_SCALAR(2.0f),
                                    FOC_NEG_ONE, FOC_ONE)),
              1.0f, fScalarTolerance);
    TEST_CHECK(foc_div_checked(FOC_SCALAR(0.5f), FOC_SCALAR(0.25f),
                               &qResult) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(qResult), 2.0f, fScalarTolerance);
    TEST_CHECK(foc_div_checked(FOC_ONE, FOC_ZERO, &qResult) ==
               FOC_RESULT_DIVIDE_BY_ZERO);
    TEST_CHECK(foc_gain_from_float(2.5f, &tGain) == FOC_RESULT_OK);
    TEST_NEAR(foc_to_float(foc_gain_apply(&tGain, FOC_SCALAR(0.2f))),
              0.5f, fTrigTolerance);

    TEST_NEAR(foc_angle_to_turns(foc_angle_from_turns(1.25f)),
              0.25f, fScalarTolerance);
    TEST_NEAR(foc_angle_to_turns(foc_angle_from_turns(-0.25f)),
              0.75f, fScalarTolerance);
    TEST_NEAR(foc_to_float(foc_angle_diff(foc_angle_from_turns(0.05f),
                                           foc_angle_from_turns(0.95f))),
              0.10f, fTrigTolerance);
    TEST_NEAR(foc_to_float(foc_angle_sin(foc_angle_from_turns(0.25f))),
              1.0f, fTrigTolerance);
    TEST_NEAR(foc_to_float(foc_angle_cos(foc_angle_from_turns(0.5f))),
              -1.0f, fTrigTolerance);
    TEST_NEAR(foc_angle_to_turns(foc_angle_atan2(FOC_ONE, FOC_ZERO)),
              0.25f, fTrigTolerance);
    TEST_NEAR(foc_to_float(foc_sin(FOC_SCALAR(0.25f))),
              1.0f, fTrigTolerance);
    TEST_NEAR(foc_to_float(foc_sqrt(FOC_SCALAR(0.25f))),
              0.5f, fTrigTolerance);

    return nFailures;
}
