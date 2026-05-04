#ifndef __USERCONFIG_H__
#define __USERCONFIG_H__

/*============================ INCLUDES ======================================*/
#include "gmsi.h"
/*============================ MACROS ========================================*/

/* Debug output: 1 = enable GLOG/RTT output, 0 = disable */
#define USERCONFIG_DEBUG_ENABLE 1

/* GMSI waveform: 1 = enable gwaveform for real-time plotting */
#if defined(AT32F421F8P7)
#   define GWAVEFORM_ENABLE 0
#else
#   define GWAVEFORM_ENABLE 1
#endif

/*============================ TYPES =========================================*/

/* GMSI object IDs — extend as new class modules are added */
#define TEMPLATE_CLASS   ((GMSI_ID_MOCK<<8)+1)
#define FOC_APP          ((GMSI_ID_MOCK<<8)+2)


/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ GLOBAL VARIABLES ==============================*/
/*============================ PROTOTYPES ====================================*/

#endif /* __USERCONFIG_H__ */
