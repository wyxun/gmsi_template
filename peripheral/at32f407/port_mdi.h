#ifndef __PORT_MDI_H__
#define __PORT_MDI_H__

#include "at32f403a_407.h"
#include "mringbuf.h"

typedef struct {
    usart_type *ptUsart;
    mringbuf_t tTxQueue;
    mringbuf_t tRxQueue;
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
void at32_usart_rx_dma_poll(void);
uint32_t at32_usart_rx_overflow_count(void);
void at32_usart_tx_dma_isr(void);

/* Instance Wrappers */
void at32_usart2_init(void);

typedef bool (*port_mdi_enqueue_rt_ptr)(uint8_t c);
void port_mdi_set_enqueue_rt_handler(port_mdi_enqueue_rt_ptr fn);

#endif /* __PORT_MDI_H__ */
