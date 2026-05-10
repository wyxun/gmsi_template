#ifndef __HALPWM_H__
#define __HALPWM_H__

#include <stdint.h>
#include <stdbool.h>

void halpwm_Init(void);
void halpwm_Start(void);
void halpwm_Stop(void);
void halpwm_SetDuty(uint32_t wDutyU, uint32_t wDutyV, uint32_t wDutyW);

#endif /* __HALPWM_H__ */
