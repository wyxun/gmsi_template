/**
 * @file  haldac.c
 * @brief DAC3 — internal-only, CH1→COMP1, CH2→COMP2/4 reference
 */

#include "haldac.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_bus.h"

void haldac_Init(void)
{
    LL_DAC_InitTypeDef init = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC3);

    /* CH1 — internal output, software trigger */
    LL_DAC_SetSignedFormat(DAC3, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
    init.TriggerSource       = LL_DAC_TRIG_SOFTWARE;
    init.TriggerSource2      = LL_DAC_TRIG_SOFTWARE;
    init.WaveAutoGeneration  = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    init.OutputBuffer        = LL_DAC_OUTPUT_BUFFER_DISABLE;
    init.OutputConnection    = LL_DAC_OUTPUT_CONNECT_INTERNAL;
    init.OutputMode          = LL_DAC_OUTPUT_MODE_NORMAL;
    LL_DAC_Init(DAC3, LL_DAC_CHANNEL_1, &init);
    LL_DAC_DisableTrigger(DAC3, LL_DAC_CHANNEL_1);
    LL_DAC_DisableDMADoubleDataMode(DAC3, LL_DAC_CHANNEL_1);

    /* CH2 — same config */
    LL_DAC_Init(DAC3, LL_DAC_CHANNEL_2, &init);
    LL_DAC_DisableTrigger(DAC3, LL_DAC_CHANNEL_2);
    LL_DAC_DisableDMADoubleDataMode(DAC3, LL_DAC_CHANNEL_2);

    LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);
    LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_2);
}

void haldac_SetCH1(uint16_t wVal)
{
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_1, wVal);
}

void haldac_SetCH2(uint16_t wVal)
{
    LL_DAC_ConvertData12RightAligned(DAC3, LL_DAC_CHANNEL_2, wVal);
}
