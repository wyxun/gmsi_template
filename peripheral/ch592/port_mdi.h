#ifndef __PORT_MDI_H__
#define __PORT_MDI_H__

#include "mringbuf.h"

#define USART_RXFLAG_IDLE       0
#define USART_RXFLAG_BUSY       1
#define USART_RXFLAG_FINISH     2

#define USART_TXFLAG_IDLE       0
#define USART_TXFLAG_BUSY       1

#define USART_DELAYTIME         4

typedef struct {
    mringbuf_t tTxQueue;
    mringbuf_t tRxQueue;
    volatile uint8_t chTXFlag;
    volatile uint8_t chRXFlag;
    volatile uint8_t chRXFinishTime;
} ch592_usart_priv_t;

/* 串口驱动的核心处理接口 */
void ch592_usart_init(uint32_t baudrate);
void ch592_usart_timer_1ms(ch592_usart_priv_t *ptPriv);
void ch592_usart_irq_handler(ch592_usart_priv_t *ptPriv);

#endif /* __PORT_MDI_H__ */
