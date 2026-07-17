#ifndef FOC_HAL_MDI_ADAPTER_H
#define FOC_HAL_MDI_ADAPTER_H

#include "foc_hal.h"
#include "mdi_hw.h"

typedef struct {
    const mdi_hardware_t *ptHardware;
    uint32_t wPwmPeriod;
} foc_mdi_motor_context_t;

foc_result_t foc_hal_mdi_Bind(foc_hal_t *ptHal,
                              foc_mdi_motor_context_t *ptContext);
foc_result_t foc_hal_mdi_BindDefault(foc_hal_t *ptHal);

#endif /* FOC_HAL_MDI_ADAPTER_H */
