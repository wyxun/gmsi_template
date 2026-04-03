#ifndef __PORT_GDI_H__
#define __PORT_GDI_H__

#include "at32f421.h"
#include "util_queue.h"
#include "at32f421_usart.h"

typedef struct {
    usart_type *ptUsart;
    util_queue_t tTxQueue;
    util_queue_t tRxQueue;
    volatile uint8_t chTXFlag;
    volatile uint8_t chRXFlag;
    volatile uint8_t chRXFinishTime;
} at32_usart_priv_t;

/* Generic USART Handlers */
void at32_usart_init(at32_usart_priv_t *ptPriv, 
                     usart_type *ptUsart,
                     uint8_t *pchTxBuf, uint32_t wTxBufSize,
                     uint8_t *pchRxBuf, uint32_t wRxBufSize);

void at32_usart_timer_1ms(at32_usart_priv_t *ptPriv);
void at32_usart_irq_handler(at32_usart_priv_t *ptPriv);

/* Instance Wrappers */
void at32_usart1_init(void);

#endif /* __PORT_GDI_H__ */
