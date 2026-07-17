/*******************************************************************************
 * @file    foc_nsd.h
 * @brief   Experimental non-blocking north/south polarity detection
 ******************************************************************************/

#ifndef FOC_NSD_H
#define FOC_NSD_H

#include "foc_config.h"
#include "foc_experiment.h"

typedef enum {
    FOC_NSD_IDLE = 0,
    FOC_NSD_SETTLE,
    FOC_NSD_POSITIVE,
    FOC_NSD_ZERO,
    FOC_NSD_NEGATIVE,
    FOC_NSD_COMPLETE,
    FOC_NSD_ABORTED,
} foc_nsd_state_t;

typedef struct {
    foc_experiment_safety_t tSafety;
    foc_scalar_t qTestVoltage;
    uint32_t wSettleSamples;
    uint32_t wPositiveSamples;
    uint32_t wZeroSamples;
    uint32_t wNegativeSamples;
} foc_nsd_params_t;

typedef struct {
    foc_scalar_t qVoltageD;
    bool bReversePolarity;
    bool bComplete;
    foc_nsd_state_t eState;
} foc_nsd_output_t;

typedef struct {
    foc_nsd_params_t tParams;
    foc_nsd_state_t eState;
    uint32_t wStateSamples;
    uint32_t wTotalSamples;
    foc_scalar_t qPositiveResponse;
    foc_scalar_t qNegativeResponse;
    bool bReversePolarity;
    bool bComplete;
} foc_nsd_t;

foc_result_t foc_nsd_Init(foc_nsd_t *ptNsd,
                          const foc_nsd_params_t *ptParams);
foc_result_t foc_nsd_Start(foc_nsd_t *ptNsd,
                           const foc_experiment_guard_t *ptGuard);
foc_result_t foc_nsd_Step(foc_nsd_t *ptNsd,
                          const foc_experiment_guard_t *ptGuard,
                          foc_scalar_t qDemodulatedResponse,
                          foc_nsd_output_t *ptOutput);

#endif /* FOC_NSD_H */
