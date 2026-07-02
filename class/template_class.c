#include "template_class.h"
#include "mdi_hw.h"
#include <string.h>

#undef  this
#define this (*ptThis)

static int template_class_Clock(uintptr_t wObjectAddr);
static int template_class_Run  (uintptr_t wObjectAddr);

static modus_base_t     s_tTemplateClassBase;
static modus_base_cfg_t s_tTemplateClassBaseCfg = {
    .wId     = TEMPLATE_CLASS,
    .wParent = 0,
    .FcnInterface = {
        .Clock = template_class_Clock,
        .Run   = template_class_Run,
    },
};

int template_class_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    template_class_t     *ptThis = (template_class_t *)wObjectAddr;
    template_class_cfg_t *ptCfg  = (template_class_cfg_t *)wObjectCfgAddr;

    if (ptThis == NULL || ptCfg == NULL) {
        MLOGF(E, "template_class_Init: NULL pointer\n");
        return MODUS_EFAIL;
    }

    ptThis->ptBase = &s_tTemplateClassBase;
    s_tTemplateClassBaseCfg.wParent = wObjectAddr;

    /* 显性数据绑定 */
    ptThis->pwSharedSystemTick = ptCfg->pwSharedSystemTick;

    /* 框架底层自动绑定（已由 mbase_Init 内部静默实现，直接赋值传递即可） */
    s_tTemplateClassBaseCfg.pchRingBuffer = ptCfg->pchRingBuffer;
    s_tTemplateClassBaseCfg.hwRingSize    = ptCfg->hwRingSize;

    this.chState = 0;
    mbase_TimerInit(&this.tLedTimer);

    return mbase_Init(ptThis->ptBase, &s_tTemplateClassBaseCfg);
}

int template_class_Run(uintptr_t wObjectAddr)
{
    template_class_t *ptThis = (template_class_t *)wObjectAddr;
    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }

    /* [跨模块通信接收示例] 
     * 从其他 Class (如 FocApp) 通过以下方式发送数据到本模块的 RingBuffer：
     * mbase_MessagePostToRing(TEMPLATE_CLASS, data, len); 
     * 
     * 注：写在 PERFC_PT_BEGIN 之前以确保每一帧都能无条件获得调度，避免死代码。 */
    // uint8_t chRxBuf[64];
    // int nLen = mbase_MessagePendFromRing(ptThis->ptBase, chRxBuf, sizeof(chRxBuf));
    // if (nLen > 0) {
    //     MLOGF(I, "[TemplateClass] Recv IPC message: %.*s\r\n", nLen, (char *)chRxBuf);
    //     /* 物理串口回显输出（可选） */
    //     if (HW.ptSerial != NULL) {
    //         MDI_Write(HW.ptSerial, chRxBuf, nLen);
    //     }
    // }

    /* 基于 perf_counter FSM 的轮询状态机 */
    PERFC_PT_BEGIN(this.chState)

    /* 软定时器驱动非阻塞闪灯示范 */
    mbase_TimerStart(&this.tLedTimer, 200);
    while (1) {
        PERFC_PT_WAIT_UNTIL(mbase_TimerPoll(&this.tLedTimer))
        if (HW.ptLedStatus != NULL) {
            MDI_Toggle(HW.ptLedStatus);
        }
        mbase_TimerStart(&this.tLedTimer, 200);
    }

    PERFC_PT_END()

    return MODUS_SUCCESS;
}

int template_class_Clock(uintptr_t wObjectAddr)
{
    (void)wObjectAddr;
    return MODUS_SUCCESS;
}

static uint8_t s_chTemplateRxBuffer[128];
static const uint32_t s_wMockSystemTick = 0; /* 模拟共享数据 */

MODUS_DECLARE_OBJECT(template_class, TemplateClass, 
    .pchRingBuffer = s_chTemplateRxBuffer, 
    .hwRingSize    = sizeof(s_chTemplateRxBuffer),
    .pwSharedSystemTick = &s_wMockSystemTick
)
