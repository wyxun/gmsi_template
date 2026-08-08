/*******************************************************************************
 * @file    motor_diagnostic.c
 * @brief   Hardware bring-up diagnostics (build-gated, non-production)
 ******************************************************************************/

#include "motor_diagnostic.h"

#if defined(FOC_ENABLE_DIAGNOSTIC) && FOC_ENABLE_DIAGNOSTIC

#include "mdebug/mshell.h"
#include "perf_counter.h"

/* Fixed test vector: net-zero volt-second, every phase within the 0.1
 * per-unit limit enforced by motor_DiagnosticSetOutput(). */
#define MOTOR_DIAG_TEST_DUTY_U   FOC_SCALAR(0.08f)
#define MOTOR_DIAG_TEST_DUTY_V   FOC_SCALAR(-0.04f)
#define MOTOR_DIAG_TEST_DUTY_W   FOC_SCALAR(-0.04f)
#define MOTOR_DIAG_TEST_HOLD_MS  100U
/* Raw-count excursion above this fraction of the calibrated offset is
 * treated as an overcurrent wiring fault. */
#define MOTOR_DIAG_CURRENT_LIMIT_NUM 1U
#define MOTOR_DIAG_CURRENT_LIMIT_DEN 2U

static bool motor_diagnostic_OverCurrent(uint32_t wRaw,
                                         uint32_t wOffset)
{
    uint32_t wBase = wOffset != 0U ? wOffset : 2048U;
    uint32_t wDelta = wRaw > wBase ? wRaw - wBase : wBase - wRaw;

    return wDelta * MOTOR_DIAG_CURRENT_LIMIT_DEN >
           wBase * MOTOR_DIAG_CURRENT_LIMIT_NUM;
}

foc_result_t motor_diagnostic_FixedDutyTest(motor_handle_t *ptMotor)
{
    motor_state_e eRunState;
    uint32_t wFaults;
    foc_adc_calib_t tCalib;
    foc_result_t eResult;
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;
    int64_t lHoldStart;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = motor_GetStatus(ptMotor, &eRunState, &wFaults);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    if (eRunState != MOTOR_STATE_IDLE ||
        wFaults != MOTOR_FAULT_NONE) {
        MLOGF(E, "[Diag] Refused: state=%d faults=0x%lX\r\n",
              (int)eRunState, (unsigned long)wFaults);
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    MLOG(I, "\r\n=== Diag: Fixed-Duty PWM & ADC Test ===\r\n");
    eResult = motor_DiagnosticSetOutput(ptMotor,
                                        MOTOR_DIAG_TEST_DUTY_U,
                                        MOTOR_DIAG_TEST_DUTY_V,
                                        MOTOR_DIAG_TEST_DUTY_W);
    if (eResult != FOC_RESULT_OK) {
        MLOGF(E, "[Diag] SetOutput rejected: %d\r\n", (int)eResult);
        return eResult;
    }
    MLOGF(I, "[Diag] Duty U=%.1f%% V=%.1f%% W=%.1f%% for %lu ms\r\n",
          _D(MOTOR_DIAG_TEST_DUTY_U) * 100.0,
          _D(MOTOR_DIAG_TEST_DUTY_V) * 100.0,
          _D(MOTOR_DIAG_TEST_DUTY_W) * 100.0,
          (unsigned long)MOTOR_DIAG_TEST_HOLD_MS);

    lHoldStart = get_system_ms();
    while (get_system_ms() - lHoldStart <
           (int64_t)MOTOR_DIAG_TEST_HOLD_MS) {
        /* Bounded hold; motor side enforces the cumulative timeout. */
    }

    eResult = motor_GetRawCurrent(ptMotor, &wRawU, &wRawV, &wRawW);
    if (eResult == FOC_RESULT_OK &&
        motor_GetCurrentCalibration(ptMotor, &tCalib) == FOC_RESULT_OK) {
        MLOGF(I, "[Diag] Raw: U=%lu V=%lu W=%lu | Offset: U=%lu V=%lu"
              " W=%lu Calibrated=%d\r\n",
              (unsigned long)wRawU, (unsigned long)wRawV,
              (unsigned long)wRawW,
              (unsigned long)tCalib.wOffsetU,
              (unsigned long)tCalib.wOffsetV,
              (unsigned long)tCalib.wOffsetW,
              (int)tCalib.bIsCalibrated);
        if (motor_diagnostic_OverCurrent(
                wRawU, tCalib.wOffsetU) ||
            motor_diagnostic_OverCurrent(
                wRawV, tCalib.wOffsetV) ||
            motor_diagnostic_OverCurrent(
                wRawW, tCalib.wOffsetW)) {
            (void)motor_DiagnosticStopOutput(ptMotor);
            MLOG(E, "[Diag] Overcurrent limit hit, output stopped\r\n");
            return FOC_RESULT_SAFETY;
        }
    }

    eResult = motor_DiagnosticStopOutput(ptMotor);
    if (eResult == FOC_RESULT_OK) {
        MLOG(I, "[Diag] Output stopped, test passed\r\n");
    } else {
        MLOGF(E, "[Diag] StopOutput failed: %d\r\n", (int)eResult);
    }
    return eResult;
}

#endif /* FOC_ENABLE_DIAGNOSTIC */
