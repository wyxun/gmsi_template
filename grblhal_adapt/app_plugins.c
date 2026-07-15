/**
 * @file   app_plugins.c
 * @brief  Project Custom Plugins Dispatcher for grblHAL
 *
 * Plugin: MCU CPU Usage (via perfc_counter)
 *
 * 使用 perfc_counter 的 __cpu_usage__ 宏对 grblHAL 实时回调循环进行精确采样。
 * 每 1000 次迭代（约 10ms × 1000 ÷ 迭代次数）计算一次 CPU 使用率，
 * 结果以百分比存储在 s_fCpuUsage 中，通过 mshell 的 "cpu" 命令随时打印。
 *
 * Hook chain:
 *   my_plugin_init() → grbl.on_execute_realtime (count loop + measure)
 *                    → grbl.on_report_options    (declare plugin)
 */

#include "grblhal_driver.h"
#include "report.h"
#include "mshell.h"
#include "mlog.h"
#include "perf_counter.h"
#include <string.h>

// 1. Declare custom user plugins init functions here:
// extern void custom_safety_init(void);

/* =========================================================================
 *  CPU Usage Plugin
 * ========================================================================= */

/* 采样结果：最近一次 1000 迭代窗口内的 CPU 使用率（%）*/
static volatile float s_fCpuUsage = 0.0f;

static on_execute_realtime_ptr s_fnPrevExecuteRealtime;
static on_report_options_ptr   s_fnPrevReportOptions;
static on_realtime_report_ptr  s_fnPrevRealtimeReport;

/* Called at ~kHz rate from grblHAL's idle / protocol loop.
 * __cpu_usage__ measures cycles consumed by this scope vs. total elapsed
 * cycles over 1000 iterations — giving true CPU load percentage. */
static void mcuload_OnExecuteRealtime(sys_state_t state)
{
    /* 每 1000 次迭代采样一次。
     * __usage__ 是 float，代表本段代码（及其调用的所有函数）
     * 在过去 1000 次迭代窗口内占用 CPU 的百分比。 */
    __cpu_usage__(1000, {
        s_fCpuUsage = __usage__;
    }) {
        /* 被测量的代码段：调用下级 hook（包含 modus_Run 等） */
        if (s_fnPrevExecuteRealtime) {
            s_fnPrevExecuteRealtime(state);
        }
    }
}

/* Called twice during startup */
static void mcuload_OnReportOptions(bool bNewopt)
{
    if (s_fnPrevReportOptions) {
        s_fnPrevReportOptions(bNewopt);
    }
    if (!bNewopt) {
        report_plugin("CPU Monitor (perfc)", "0.02");
    }
}

/* Called when grblHAL assembles a real-time status report.
 * Injects "|CPU:<value>" into the status line. */
static void mcuload_OnRealtimeReport(stream_write_ptr fnStreamWrite,
                                     report_tracking_flags_t wReport)
{
    static char szBuf[16];
    strcpy(szBuf, "|CPU:");
    strcat(szBuf, uitoa((uint32_t)s_fCpuUsage));
    fnStreamWrite(szBuf);

    if (s_fnPrevRealtimeReport) {
        s_fnPrevRealtimeReport(fnStreamWrite, wReport);
    }
}

/* =========================================================================
 *  mshell command: "cpu"
 *  用法：在 RTT 终端输入 "cpu"，打印当前 CPU 占用率
 * ========================================================================= */
static void cmd_cpu(const char *args)
{
    (void)args;
    MLOGF(I, "CPU Usage: %.2f%%\r\n", (double)s_fCpuUsage);
}

MODUS_SHELL_CMD(cpu, cmd_cpu, "Show grblHAL loop CPU usage %%");

/* ---- my_plugin_init (weak override from grblHAL) ---- */

void my_plugin_init(void)
{
    // 2. Call custom plugin initializers:
    // custom_safety_init();

    /* Chain on_report_options */
    s_fnPrevReportOptions  = grbl.on_report_options;
    grbl.on_report_options = mcuload_OnReportOptions;

    /* Chain on_execute_realtime — wrap with __cpu_usage__ measurement */
    s_fnPrevExecuteRealtime  = grbl.on_execute_realtime;
    grbl.on_execute_realtime = mcuload_OnExecuteRealtime;

    /* Chain on_realtime_report — inject |CPU:N into status reports */
    s_fnPrevRealtimeReport  = grbl.on_realtime_report;
    grbl.on_realtime_report = mcuload_OnRealtimeReport;
}
