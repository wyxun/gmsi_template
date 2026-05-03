/*******************************************************************************
 * @file    foc_hal.c
 * @brief   FOC HAL 层 — 平台无关部分
 ******************************************************************************/

#include <stddef.h>
#include "foc_hal.h"

static foc_pwm_ops_t *s_ptPwmOps = NULL;
static foc_adc_ops_t *s_ptAdcOps = NULL;

void foc_hal_pwm_register(foc_pwm_ops_t *ptOps)
{
    s_ptPwmOps = ptOps;
}

foc_pwm_ops_t *foc_hal_pwm_get(void)
{
    return s_ptPwmOps;
}

void foc_hal_adc_register(foc_adc_ops_t *ptOps)
{
    s_ptAdcOps = ptOps;
}

foc_adc_ops_t *foc_hal_adc_get(void)
{
    return s_ptAdcOps;
}

int foc_hal_Init(void)
{
    if (s_ptPwmOps == NULL || s_ptAdcOps == NULL) {
        return -1;
    }
    return 0;
}

void foc_hal_current_reconstruct(phase_current_handle_t *ptHandle)
{
    if (s_ptAdcOps == NULL || s_ptAdcOps->fnReconstruct == NULL) {
        return;
    }
    s_ptAdcOps->fnReconstruct(ptHandle);
}
