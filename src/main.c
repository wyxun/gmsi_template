/*============================ INCLUDES ======================================*/
#include <stdio.h>
#include "peripheral.h"
#include "gdi_hw.h"
#include "gmsi.h"
#include "SEGGER_RTT.h"
#include "perf_counter.h"
#include "util_debug.h"     /* LOG_OUT */

/*============================ MACROS ========================================*/
/*
 * DEBUG_MINIMAL = 1 : 最小模式 — 仅时钟 + perf_counter + RTT 心跳
 *                     用于验证最小启动链路
 * DEBUG_MINIMAL = 0 : 完整模式（接入 gmsi 框架 + 外设）
 */
#define DEBUG_MINIMAL   1

/*============================ TYPES =========================================*/
/*============================ PROTOTYPES ====================================*/

/*============================ GLOBAL VARIABLES ==============================*/

/* 初始化完成标志：SysTick_Handler 中用于保护 gmsi_Clock 等调用 */
volatile uint8_t s_bInitDone = 0;

/*============================ LOCAL VARIABLES ===============================*/

#if !DEBUG_MINIMAL
/* ---------- GMSI 存储桩（无 flash 时只做 RAM 缓存） --------- */
static void s_StorageWrite(uint16_t *phwAddr, uint16_t hwLen)
    { (void)phwAddr; (void)hwLen; }
static void s_StorageRead(uint16_t *phwAddr, uint16_t hwLen)
    { (void)phwAddr; (void)hwLen; }

static uint16_t       s_hwSysDataArray[16];
static gstorage_data_t s_tSysData = {
    .phwStorageStartAddr = s_hwSysDataArray,
    .hwStorageLength     = 16,
    .hwCrcFlag           = 0,
    .fcnWrite            = s_StorageWrite,
    .fcnRead             = s_StorageRead,
};
static gmsi_t s_tGmsi = { &s_tSysData };
#endif

/*============================ IMPLEMENTATION ================================*/

/**
 * @brief RTT 输出桥接函数（由 TRACE_MCU_WRITE_STRING 宏调用）
 */
void user_trace_output(const char *str)
{
    SEGGER_RTT_WriteString(0, str);
}

/*============================ MAIN ==========================================*/

int main(void)
{
    /* ------------------------------------------------------------------ */
    /* 1. 底层硬件初始化 (时钟/GPIO/外部中断等)                           */
    /* ------------------------------------------------------------------ */
    peripheral_Init();

    /* ------------------------------------------------------------------ */
    /* 2. perf_counter 初始化（自动接管/配置 SysTick）                   */
    /*    传入 false → perf_counter 自行配置 SysTick 为 1ms 节拍          */
    /* ------------------------------------------------------------------ */
    perfc_init(false);

    /* ------------------------------------------------------------------ */
    /* 3. RTT 初始化                                                       */
    /* ------------------------------------------------------------------ */
    SEGGER_RTT_Init();
    LOG_OUT("\r\n=== AT32F421F8P7 BOOT OK @ HICK 48 MHz ===\r\n");

#if !DEBUG_MINIMAL
    /* ------------------------------------------------------------------ */
    /* 4. [完整模式] 外设初始化 + GMSI 框架                               */
    /* ------------------------------------------------------------------ */
    /* peripheral_Init(); */          /* 取消注释以启用外设层 */
    gmsi_Init(&s_tGmsi);
#endif

    /* 允许 SysTick_Handler 中调用 gmsi_Clock */
    s_bInitDone = 1;

    /* ------------------------------------------------------------------ */
    /* 5. 主循环                                                           */
    /* ------------------------------------------------------------------ */
    uint32_t wCounter = 0;

    while (1) {
#if !DEBUG_MINIMAL
        gmsi_Run();
        /* peripheral 业务放这里 */
#endif
        /* 每 1000ms 打印一次心跳，验证时钟和 perf_counter 工作正常 */
        if (perfc_is_time_out_ms(1000)) {
            wCounter++;
            char buf[64];
            snprintf(buf, sizeof(buf),
                "[TICK] %lu s  SYSCLK=%lu Hz\r\n",
                (unsigned long)wCounter,
                (unsigned long)get_system_core_clock_hz());
            SEGGER_RTT_WriteString(0, buf);
        }
    }
    return 0;
}
