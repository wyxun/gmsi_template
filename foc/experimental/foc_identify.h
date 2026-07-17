/*******************************************************************************
 * @file    foc_identify.h
 * @brief   Experimental non-blocking PMSM parameter identification
 ******************************************************************************/

#ifndef FOC_IDENTIFY_H
#define FOC_IDENTIFY_H

#include "foc_config.h"
#include "foc_experiment.h"
#include "foc_core.h"

typedef enum {
    FOC_IDENTIFY_RS_LD_LQ = 0,
    FOC_IDENTIFY_FLUX,
} foc_identify_mode_t;

typedef enum {
    FOC_IDENTIFY_IDLE = 0,
    FOC_IDENTIFY_RS_HALF,
    FOC_IDENTIFY_RS_FULL,
    FOC_IDENTIFY_LD_ZERO,
    FOC_IDENTIFY_LD_RISE,
    FOC_IDENTIFY_LQ_ZERO,
    FOC_IDENTIFY_LQ_RISE,
    FOC_IDENTIFY_FLUX_SETTLE,
    FOC_IDENTIFY_COMPLETE,
    FOC_IDENTIFY_ABORTED,
} foc_identify_state_t;

typedef struct {
    foc_experiment_safety_t tSafety;
    foc_identify_mode_t eMode;
    foc_scalar_t qHalfVoltage;
    foc_scalar_t qFullVoltage;
    foc_scalar_t qFluxVoltage;
    foc_scalar_t qCurrentRiseRatio;
    foc_scalar_t qResetCurrentThreshold;
    foc_scalar_t qInductanceTimeStep;
    foc_scalar_t qMinimumFluxSpeed;
    foc_scalar_t qKnownResistance;
    uint32_t wSettleSamples;
    uint32_t wResetSamples;
    uint32_t wMaximumRiseSamples;
} foc_identify_params_t;

typedef struct {
    foc_scalar_t qCurrentD;
    foc_scalar_t qCurrentQ;
    foc_scalar_t qElectricalSpeed;
} foc_identify_input_t;

typedef struct {
    foc_dq_t tVoltage;
    foc_scalar_t qResistance;
    foc_scalar_t qInductanceD;
    foc_scalar_t qInductanceQ;
    foc_scalar_t qFlux;
    bool bComplete;
    foc_identify_state_t eState;
} foc_identify_output_t;

typedef struct {
    foc_identify_params_t tParams;
    foc_identify_state_t eState;
    uint32_t wStateSamples;
    uint32_t wTotalSamples;
    foc_scalar_t qHalfCurrent;
    foc_scalar_t qFullCurrent;
    foc_scalar_t qRiseTime;
    foc_scalar_t qResistance;
    foc_scalar_t qInductanceD;
    foc_scalar_t qInductanceQ;
    foc_scalar_t qFlux;
    bool bComplete;
} foc_identify_t;

foc_result_t foc_identify_Init(foc_identify_t *ptIdentify,
                               const foc_identify_params_t *ptParams);
foc_result_t foc_identify_Start(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard);
foc_result_t foc_identify_Step(
    foc_identify_t *ptIdentify,
    const foc_experiment_guard_t *ptGuard,
    const foc_identify_input_t *ptInput,
    foc_identify_output_t *ptOutput);

#endif /* FOC_IDENTIFY_H */
