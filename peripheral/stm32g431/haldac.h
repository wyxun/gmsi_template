/**
 * @file  haldac.h
 * @brief DAC3 driver — internal outputs for COMP reference voltages
 */

#ifndef __HALDAC_H__
#define __HALDAC_H__

#include <stdint.h>

void haldac_Init(void);
void haldac_SetCH1(uint16_t wVal);
void haldac_SetCH2(uint16_t wVal);

#endif /* __HALDAC_H__ */
