/*******************************************************************************
 * @file    foc_hall.h
 * @brief   Safe six-step Hall angle and speed observer
 ******************************************************************************/

#ifndef FOC_HALL_H
#define FOC_HALL_H

#include "motor_position.h"

typedef struct {
    uint8_t achSectorByCode[8];
    uint16_t hwInvalidTimeout;
    foc_scalar_t qSpeedFilterAlpha;
} foc_hall_params_t;

typedef struct {
    foc_hall_params_t tParams;
    foc_angle_t tAngle;
    foc_scalar_t qSpeed;
    foc_scalar_t qConfidence;
    uint16_t hwTicksSinceEdge;
    uint16_t hwInvalidSamples;
    uint8_t chPreviousSector;
    bool bInitialized;
    bool bValid;
} foc_hall_t;

typedef uint8_t (*foc_hall_read_code_fn_t)(void *pHardwareContext);

typedef struct {
    foc_hall_t *ptHall;
    void *pHardwareContext;
    foc_hall_read_code_fn_t fnReadCode;
} foc_hall_source_adapter_t;

void foc_hall_DefaultParams(foc_hall_params_t *ptParams);
foc_result_t foc_hall_Init(foc_hall_t *ptHall,
                           const foc_hall_params_t *ptParams);
void foc_hall_Reset(foc_hall_t *ptHall);
foc_result_t foc_hall_Step(foc_hall_t *ptHall,
                           uint8_t chHallCode,
                           foc_position_output_t *ptOutput);
foc_result_t foc_hall_source_Init(foc_hall_source_adapter_t *ptAdapter,
                                  foc_hall_t *ptHall,
                                  void *pHardwareContext,
                                  foc_hall_read_code_fn_t fnReadCode);
foc_position_source_if_t foc_hall_PositionSourceInterface(
    foc_hall_source_adapter_t *ptAdapter);

#endif /* FOC_HALL_H */
