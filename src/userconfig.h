#ifndef __USERCONFIG_H__
#define __USERCONFIG_H__

/*============================ INCLUDES ======================================*/
#include "at32f421.h"
#include "gmsi.h"
/*============================ MACROS ========================================*/

/* System clock frequency (Hz) — HICK 48 MHz */
#define SYSTEM_CLOCK_HZ         48000000UL

/* Debug output: 1 = enable GLOG/RTT output, 0 = disable */
#define USERCONFIG_DEBUG_ENABLE 1

/*============================ TYPES =========================================*/

/* GMSI object IDs — extend as new class modules are added */
#define TEMPLATE_CLASS   ((GMSI_ID_MOCK<<8)+1)
#define FOC_APP          ((GMSI_ID_MOCK<<8)+2)


/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ GLOBAL VARIABLES ==============================*/
/*============================ PROTOTYPES ====================================*/

#endif /* __USERCONFIG_H__ */
