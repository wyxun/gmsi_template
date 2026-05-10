#ifndef __HALADC_H__
#define __HALADC_H__

#include <stdint.h>
#include <stdbool.h>

/* ADC ordinary channel indices (DMA order) */
#define HALADC_ORD_BUS_VOLT   0
#define HALADC_ORD_MOS_TEMP   1
#define HALADC_ORD_POTENTIO   2
#define HALADC_ORD_IBUS_AVG   3
#define HALADC_ORD_CHANNELS   4

void haladc_Init(void);
uint16_t haladc_GetOrdinary(uint8_t chIndex);
void haladc_GetPreemptRaw(uint16_t *phwU, uint16_t *phwV, uint16_t *phwW);
void haladc_StartRegular(void);

#endif /* __HALADC_H__ */
