/*============================ INCLUDES ======================================*/
#include "template_class.h"
#include "peripheral.h"
#include "mdi_hw.h"
#include <string.h>

/*============================ MACROS ========================================*/
/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ TYPES =========================================*/
/*============================ PROTOTYPES ====================================*/

int template_class_Clock(uintptr_t wObjectAddr);
int template_class_Run  (uintptr_t wObjectAddr);

/*============================ GLOBAL VARIABLES ==============================*/

mcoroutine_handle_t tMcoroutineTemplateClassHandle = {
    .bIsRunning = false,
    .pfcn       = NULL,
};

/*============================ LOCAL VARIABLES ===============================*/

static modus_base_t     s_tTemplateClassBase;

static modus_base_cfg_t s_tTemplateClassBaseCfg = {
    .wId     = TEMPLATE_CLASS,
    .wParent = 0,
    .FcnInterface = {
        .Clock = template_class_Clock,
        .Run   = template_class_Run,
    },
};

/*============================ IMPLEMENTATION ================================*/

/* Optional: coroutine 鈥?triggered by an event */
fsm_rt_t template_class_coroutine(void *pvParam)
{
    template_class_t    *ptObject = (template_class_t *)pvParam;
    mcoroutine_handle_t *ptThis   = &tMcoroutineTemplateClassHandle;

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

/* Event handler 鈥?called from template_class_Run() */
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

const char *msg = "Hello from template_class via USART1 MDI!\r\n";
/* Called in the MODUS while(1) main loop */
int template_class_Run(uintptr_t wObjectAddr)
{
    int        wRet  = MODUS_SUCCESS;
    uint32_t   wEvent;
    template_class_t *ptThis = (template_class_t *)wObjectAddr;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }

    wEvent = mbase_EventPend(ptThis->ptBase);
    if (wEvent) {
        template_class_EventHandle(ptThis, wEvent);
    }

    /* Demonstration: Print to UART every 500ms using the MDI stream interface */
#if 0
    if (perfc_is_time_out_ms(1000))
    {
        MDI_Write(HW.ptSerial, (const uint8_t*)msg, strlen(msg));
    }

    // echo
    uint8_t chData[128];
    int     hwReadLen = MDI_Read(HW.ptSerial, chData, sizeof(chData));
    if (hwReadLen > 0) {
        MDI_Write(HW.ptSerial, chData, hwReadLen);
    }
#endif


    return wRet;
}

/* Called in the 1ms SysTick interrupt (via modus_Clock) */
int template_class_Clock(uintptr_t wObjectAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    int wRet = MODUS_SUCCESS;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }

    /* TODO: add 1ms periodic operations */

    return wRet;
}

/* Initialize the object and register it in the MODUS list */
int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    template_class_t     *ptThis = (template_class_t *)wObjectAddr;
    template_class_cfg_t *ptCfg  = (template_class_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        MLOGF(E, "template_class_Init: NULL pointer.\n");
        return MODUS_EFAIL;
    }

    ptThis->ptBase = &s_tTemplateClassBase;
    if (ptThis->ptBase == NULL) {
        return MODUS_EAGAIN;
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

    return mbase_Init(ptThis->ptBase, &s_tTemplateClassBaseCfg);
}

// 鍔犺浇妯″潡
MODUS_DECLARE_OBJECT(template_class, TemplateClass, 
    .pchRingBuffer = NULL, 
    .hwRingSize = 0
)
