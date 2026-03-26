#ifndef __TEMPLATE_CLASS_H__
#define __TEMPLATE_CLASS_H__

/*============================ INCLUDES ======================================*/
#include "gmsi.h"
#include "userconfig.h"

/*============================ MACROS ========================================*/
/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ TYPES =========================================*/

/* Configuration structure — passed to template_class_Init() */
typedef struct {
    uint8_t  *pchRingBuffer;
    uint16_t  hwRingSize;
} template_class_cfg_t;

/* Object structure */
typedef struct {
    gmsi_base_t *ptBase;
    /* add hardware-related fields here */
} template_class_t;

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ LOCAL VARIABLES ===============================*/
/*============================ PROTOTYPES ====================================*/

int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/*============================ IMPLEMENTATION ================================*/

#endif /* __TEMPLATE_CLASS_H__ */
