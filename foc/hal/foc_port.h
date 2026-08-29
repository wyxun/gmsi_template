/****************************************************************************
 * @file    foc_port.h
 * @brief   Compile-time hardware port for the single FOC power stage
 * @author  Codex
 * @date    2026-08-29
 ************************************************************************** */

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include "foc_types.h"

typedef enum {
    FOC_CALIBRATION_BUSY = 0,
    FOC_CALIBRATION_COMPLETE,
    FOC_CALIBRATION_FAILED,
} foc_calibration_state_e;

/** @brief Initialize the port-side calibration state. */
void foc_port_Init(void);
/** @brief Start a new three-phase current-offset calibration. */
void foc_port_CurrentCalibrationBegin(void);
/**
 * @brief Accumulate one current-offset sample.
 * @param ptCalibration Output offsets and calibration status.
 * @return Current calibration state.
 */
foc_calibration_state_e foc_port_CurrentCalibrationStep(
    foc_adc_calib_t *ptCalibration);
/**
 * @brief Read and normalize the three phase currents.
 * @param ptCalibration Validated current offsets.
 * @param ptInput Output core input; only phase currents are updated.
 * @return FOC_RESULT_OK or an input/safety error.
 */
foc_result_t foc_port_CurrentSample(const foc_adc_calib_t *ptCalibration,
                                    foc_core_input_t *ptInput);
/**
 * @brief Write all three PWM compare registers.
 * @param ptDuty Three phase duty values in the [0, 1] range.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_port_DutyCommit(const foc_duty_abc_t *ptDuty);
/**
 * @brief Enable or disable the power-stage PWM output.
 * @param bEnable True to enable the output.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_port_PwmEnable(bool bEnable);
/** @brief Disable the power stage immediately from any context. */
void foc_port_EmergencyStop(void);

#endif /* FOC_PORT_H */
