/*============================ INCLUDES ======================================*/
#include "template_class.h"

/*============================ MACROS ========================================*/
/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ TYPES =========================================*/
/*============================ PROTOTYPES ====================================*/

int template_class_Clock(uintptr_t wObjectAddr);
int template_class_Run  (uintptr_t wObjectAddr);

/*============================ GLOBAL VARIABLES ==============================*/

gcoroutine_handle_t tGcoroutineTemplateClassHandle = {
    .bIsRunning = false,
    .pfcn       = NULL,
};

/*============================ LOCAL VARIABLES ===============================*/

static gmsi_base_t     s_tTemplateClassBase;

static gmsi_base_cfg_t s_tTemplateClassBaseCfg = {
    .wId     = TEMPLATE_CLASS,
    .wParent = 0,
    .FcnInterface = {
        .Clock = template_class_Clock,
        .Run   = template_class_Run,
    },
};

/*============================ IMPLEMENTATION ================================*/

/* Optional: coroutine — triggered by an event */
fsm_rt_t template_class_coroutine(void *pvParam)
{
    template_class_t    *ptObject = (template_class_t *)pvParam;
    gcoroutine_handle_t *ptThis   = &tGcoroutineTemplateClassHandle;

PERFC_PT_BEGIN(this.chState)
    do {
    PERFC_PT_WAIT_FOR_RES_UNTIL(
        (ptObject != NULL),
        ptObject = (template_class_t *)pvParam;
    )
    /* TODO: coroutine body */
    } while (0);
PERFC_PT_END()

    return fsm_rt_cpl;
}

/* Event handler — called from template_class_Run() */
static void template_class_EventHandle(template_class_t *ptThis, uint32_t wEvent)
{
    if (ptThis == NULL) {
        return;
    }

    /* TODO: handle events, e.g.:
     * if (wEvent & Event_SomeEvent) { ... }
     */
    (void)wEvent;
}

/* Called in the GMSI while(1) main loop */
int template_class_Run(uintptr_t wObjectAddr)
{
    int        wRet  = GMSI_SUCCESS;
    uint32_t   wEvent;
    template_class_t *ptThis = (template_class_t *)wObjectAddr;

    if (ptThis == NULL) {
        return GMSI_EFAIL;
    }

    wEvent = gbase_EventPend(ptThis->ptBase);
    if (wEvent) {
        template_class_EventHandle(ptThis, wEvent);
    }

    /* TODO: add periodic logic or state machine here */

    return wRet;
}

/* Called in the 1ms SysTick interrupt (via gmsi_Clock) */
int template_class_Clock(uintptr_t wObjectAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    int wRet = GMSI_SUCCESS;

    if (ptThis == NULL) {
        return GMSI_EFAIL;
    }

    /* TODO: add 1ms periodic operations */

    return wRet;
}

/* Initialize the object and register it in the GMSI list */
int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    template_class_t     *ptThis = (template_class_t *)wObjectAddr;
    template_class_cfg_t *ptCfg  = (template_class_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        GLOG_PRINTF("template_class_Init: NULL pointer.");
        return GMSI_EFAIL;
    }

    ptThis->ptBase = &s_tTemplateClassBase;
    if (ptThis->ptBase == NULL) {
        return GMSI_EAGAIN;
    }

    s_tTemplateClassBaseCfg.wParent = wObjectAddr;

    if (ptCfg->pchRingBuffer != NULL && ptCfg->hwRingSize != 0) {
        perfc_with(ptThis->ptBase) {
            _->tRingBuffer.buffer      = ptCfg->pchRingBuffer;
            _->tRingBuffer.hwBufferSize = ptCfg->hwRingSize;
            _->tRingBuffer.hwWriteIndex = 0;
            _->tRingBuffer.hwReadIndex  = 0;
        };
    }

    return gbase_Init(ptThis->ptBase, &s_tTemplateClassBaseCfg);
}
