/**
 * @file   at32f421_conf.h
 * @brief  AT32F421 peripheral library configuration (project-local copy).
 *         Enable only the modules used in this project.
 */

#ifndef __AT32F421_CONF_H
#define __AT32F421_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Clock values -------------------------------------------------------- */
#if !defined(HEXT_VALUE)
#define HEXT_VALUE               ((uint32_t)8000000)   /* 8 MHz external crystal */
#endif
#define HEXT_STARTUP_TIMEOUT     ((uint16_t)0x3000)
#define HICK_VALUE               ((uint32_t)8000000)   /* 8 MHz internal RC (base) */
#define LEXT_VALUE               ((uint32_t)32768)

/* ---- Module enable ------------------------------------------------------- */
#define CRM_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define DEBUG_MODULE_ENABLED

/* Enable below as needed:
#define TMR_MODULE_ENABLED
*/
#define USART_MODULE_ENABLED
/*
#define SPI_MODULE_ENABLED
#define I2C_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define ADC_MODULE_ENABLED
#define CMP_MODULE_ENABLED
#define SCFG_MODULE_ENABLED
#define EXINT_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define CRC_MODULE_ENABLED
#define WDT_MODULE_ENABLED
#define WWDT_MODULE_ENABLED
#define ERTC_MODULE_ENABLED
*/

/* ---- Includes ------------------------------------------------------------ */
#ifdef CRM_MODULE_ENABLED
    #include "at32f421_crm.h"
#endif
#ifdef GPIO_MODULE_ENABLED
    #include "at32f421_gpio.h"
#endif
#ifdef MISC_MODULE_ENABLED
    #include "at32f421_misc.h"
#endif
#ifdef FLASH_MODULE_ENABLED
    #include "at32f421_flash.h"
#endif
#ifdef DEBUG_MODULE_ENABLED
    #include "at32f421_debug.h"
#endif
#ifdef TMR_MODULE_ENABLED
    #include "at32f421_tmr.h"
#endif
#ifdef USART_MODULE_ENABLED
    #include "at32f421_usart.h"
#endif
#ifdef SPI_MODULE_ENABLED
    #include "at32f421_spi.h"
#endif
#ifdef I2C_MODULE_ENABLED
    #include "at32f421_i2c.h"
#endif
#ifdef DMA_MODULE_ENABLED
    #include "at32f421_dma.h"
#endif
#ifdef ADC_MODULE_ENABLED
    #include "at32f421_adc.h"
#endif
#ifdef CMP_MODULE_ENABLED
    #include "at32f421_cmp.h"
#endif
#ifdef SCFG_MODULE_ENABLED
    #include "at32f421_scfg.h"
#endif
#ifdef EXINT_MODULE_ENABLED
    #include "at32f421_exint.h"
#endif
#ifdef PWC_MODULE_ENABLED
    #include "at32f421_pwc.h"
#endif
#ifdef CRC_MODULE_ENABLED
    #include "at32f421_crc.h"
#endif
#ifdef WDT_MODULE_ENABLED
    #include "at32f421_wdt.h"
#endif
#ifdef WWDT_MODULE_ENABLED
    #include "at32f421_wwdt.h"
#endif
#ifdef ERTC_MODULE_ENABLED
    #include "at32f421_ertc.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __AT32F421_CONF_H */
