#include "mshell.h"
#include "mdi_hw.h"

/*
 * MShell 自定义 I/O 后端 — 将串口指令 shell 的输入输出
 * 绑定到 MDI UART0 流（PB7 TX / PB4 RX, 115200 8N1）
 */

/* 非阻塞读  — 通过 MDI 层从硬件 FIFO 读取 */
static unsigned mshell_uart_Read(char *pchBuf, unsigned hwSize)
{
    return (unsigned)MDI_Read(HW.ptSerial, (uint8_t *)pchBuf, hwSize);
}

/* 阻塞写  — 通过 MDI 层逐字节等待 TX 空闲后发送 */
static void mshell_uart_Write(const char *pchBuf, unsigned hwSize)
{
    MDI_Write(HW.ptSerial, (const uint8_t *)pchBuf, hwSize);
}

static const mshell_io_t s_tMshellUartIO = {
    .pfcnRead  = mshell_uart_Read,
    .pfcnWrite = mshell_uart_Write,
};

/*
 * 需要在 main() 中调用一次（在 modus_Init / modus_Run 之前），
 * 将 mshell 的 I/O 后端从默认的 SEGGER RTT 切换到 UART0 串口
 */
void mshell_uart_init(void)
{
    mshell_SetIO(&s_tMshellUartIO);
}
