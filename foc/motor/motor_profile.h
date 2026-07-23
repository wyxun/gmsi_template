/*******************************************************************************
 * @file    motor_profile.h
 * @brief   Public immutable high-frequency profiling snapshot and per-motor API
 ******************************************************************************/

#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H

#include "foc_numeric.h"
#include <stdint.h>

#define MOTOR_HF_PROFILE_VALID_TOTAL           (1U << 0)
#define MOTOR_HF_PROFILE_VALID_SAMPLE_CURRENT  (1U << 1)
#define MOTOR_HF_PROFILE_VALID_CLARKE          (1U << 2)
#define MOTOR_HF_PROFILE_VALID_PARK            (1U << 3)
#define MOTOR_HF_PROFILE_VALID_IPARK           (1U << 4)
#define MOTOR_HF_PROFILE_VALID_MODULATE        (1U << 5)
#define MOTOR_HF_PROFILE_VALID_COMMIT          (1U << 6)

typedef struct {
    uint32_t wSampleSequence;
    uint32_t wTotalCycles;
    uint32_t wSampleCurrentCycles;
    uint32_t wClarkeCycles;
    uint32_t wParkCycles;
    uint32_t wIparkCycles;
    uint32_t wModulateCycles;
    uint32_t wCommitCycles;
    uint32_t wValidFlags;
    foc_result_t eResult;
} motor_hf_profile_snapshot_t;

union motor_handle_u;
typedef union motor_handle_u motor_handle_t;

foc_result_t motor_GetHighFrequencyProfileSnapshot(
    const motor_handle_t *ptMotor,
    motor_hf_profile_snapshot_t *ptSnapshot);

#endif /* MOTOR_PROFILE_H */
