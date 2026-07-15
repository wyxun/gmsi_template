/**
 * @file   at32f407_conf.h
 * @brief  AT32F407 peripheral library configuration (project-local copy).
 *         Enable only the modules used in this project.
 */

#ifndef __AT32F407_CONF_H
#define __AT32F407_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Clock values -------------------------------------------------------- */
#if !defined(HEXT_VALUE)
#define HEXT_VALUE               ((uint32_t)8000000)   /* 8 MHz external crystal */
#endif
#define HEXT_STARTUP_TIMEOUT     ((uint16_t)0x3000)
#define HICK_VALUE               ((uint32_t)8000000)   /* 8 MHz internal RC */

/* ---- Module enable ------------------------------------------------------- */
#define CRM_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define DEBUG_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define USB_MODULE_ENABLED
#define ACC_MODULE_ENABLED
#define TMR_MODULE_ENABLED

/* ---- Includes ------------------------------------------------------------ */
#ifdef CRM_MODULE_ENABLED
    #include "at32f403a_407_crm.h"
#endif
#ifdef GPIO_MODULE_ENABLED
    #include "at32f403a_407_gpio.h"
#endif
#ifdef USART_MODULE_ENABLED
    #include "at32f403a_407_usart.h"
#endif
#ifdef DEBUG_MODULE_ENABLED
    #include "at32f403a_407_debug.h"
#endif
#ifdef FLASH_MODULE_ENABLED
    #include "at32f403a_407_flash.h"
#endif
#ifdef MISC_MODULE_ENABLED
    #include "at32f403a_407_misc.h"
#endif
#ifdef DMA_MODULE_ENABLED
    #include "at32f403a_407_dma.h"
#endif
#ifdef USB_MODULE_ENABLED
    #include "at32f403a_407_usb.h"
#endif
#ifdef ACC_MODULE_ENABLED
    #include "at32f403a_407_acc.h"
#endif
#ifdef TMR_MODULE_ENABLED
    #include "at32f403a_407_tmr.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __AT32F407_CONF_H */
