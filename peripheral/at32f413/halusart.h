#ifndef __HALUSART_H__
#define __HALUSART_H__

#include <stdint.h>

void halusart_Init(void);
int32_t halusart_SendData(uint8_t *pchData, uint16_t hwLen);
uint16_t halusart_ReceiveData(uint8_t *pchBuf);

#endif /* __HALUSART_H__ */
