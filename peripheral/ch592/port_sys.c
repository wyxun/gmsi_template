#include "peripheral.h"
#include <stdint.h>
#include <stdbool.h>
#include "CH592SFR.h"
#include "core_riscv.h"

/* 系统内核时钟强符号回传，完美覆盖 perf_counter 中的弱符号 */
uint32_t SystemCoreClock = 60000000UL;

uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}

/* 官方中断向量表默认调用的时钟/SysTick初始化入口 */
void SystemInit(void)
{
    /* 1. 重置并配置硬件 SysTick 全局自减计数器 */
    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    SysTick->SR = 0;

    /* 2. 配置 60MHz 时钟下 1ms 的比较中断值 (60,000 counts) */
    SysTick->CMP = 60000UL - 1U;

    /* 3. 开启 System Timer，使用 HCLK 作为基准并开启中断 */
    SysTick->CTLR = SysTick_CTLR_INIT |
                    SysTick_CTLR_STRE |
                    SysTick_CTLR_STCLK |
                    SysTick_CTLR_STIE |
                    SysTick_CTLR_STE;
}

/* 串口0波特率配置算法 */
static void Uart0_Init(uint32_t baudrate)
{
    /* 1. 引脚配置: PB7(TX0) 设为输出高电平，PB4(RX0) 设为输入 */
    R32_PB_OUT |= (1 << 7);
    R32_PB_DIR |= (1 << 7);
    R32_PB_DIR &= ~(1 << 4);

    /* 2. 算波特率分频因子 (16C550 整除公式) */
    uint32_t x = 10 * get_system_core_clock_hz() / 8 / baudrate;
    uint32_t div = (x + 5) / 10;

    /* 3. 写入 UART 寄存器组并使能 FIFO */
    R8_UART0_LCR = RB_LCR_DLAB; /* 开启 Latch 访问 */
    R16_UART0_DL = (uint16_t)div;
    R8_UART0_DIV = 1;
    R8_UART0_LCR = RB_LCR_WORD_SZ; /* 8N1，退出 DLAB */

    R8_UART0_FCR = 0x07; /* 清空并使能 FIFO */
    R8_UART0_IER = 0;    /* 轮询模式 */
}

/* 外设初始化总入口 */
void peripheral_Init(void)
{
    /* 1. 开启 UART0 外设时钟 (清零表示使能时钟) */
    R8_SLP_CLK_OFF0 &= ~RB_SLP_CLK_UART0;

    /* 2. 板载 GPIO LED (PA8) 初始化 */
    R32_PA_DIR |= (1 << 8);
    R32_PA_OUT |= (1 << 8); /* 默认灭 (低电平亮) */

    /* 3. 串口 0 初始化为 115200 */
    Uart0_Init(115200);
}

void peripheral_Clock(void) {}

void peripheral_EnableIRQ(void)
{
    __asm__ __volatile__("csrs mstatus, 8");
}

void peripheral_DisableIRQ(void)
{
    __asm__ __volatile__("csrc mstatus, 8");
}

/* =============================================================================
 * Libc / Compiler-RT Builtin Shims for Bare-Metal RISC-V Target (No Libc)
 * =============================================================================
 */

#include <stddef.h>

void *memset(void *s, int c, size_t n) {
    unsigned char *p = s;
    while (n--) *p++ = (unsigned char)c;
    return s;
}

void *memcpy(void *dest, const void *src, size_t n) {
    unsigned char *d = dest;
    const unsigned char *s = src;
    while (n--) *d++ = *s++;
    return dest;
}

size_t strlen(const char *s) {
    const char *p = s;
    while (*p) p++;
    return p - s;
}

char *strncpy(char *dest, const char *src, size_t n) {
    char *d = dest;
    while (n && (*d++ = *src++)) n--;
    while (n--) *d++ = '\0';
    return dest;
}

/* 经典 64位有符号整数除法 (基于移位-减法，不依赖 "/" / "%" 运算符) */
int64_t __divdi3(int64_t a, int64_t b) {
    if (b == 0) return 0;
    int sign = 1;
    if (a < 0) { a = -a; sign = -sign; }
    if (b < 0) { b = -b; sign = -sign; }
    uint64_t ua = a;
    uint64_t ub = b;
    uint64_t res = 0;
    for (int i = 63; i >= 0; i--) {
        if ((ua >> i) >= ub) {
            res |= (1ULL << i);
            ua -= (ub << i);
        }
    }
    return sign < 0 ? -((int64_t)res) : (int64_t)res;
}

/* 浮点转换内建辅助函数的纯位操作实现，避开 C 编译器类型转换递归 */
double __extendsfdf2(float a) {
    union { float f; uint32_t u; } src;
    union { double d; uint64_t u; } dst;
    src.f = a;
    uint32_t sign = src.u & 0x80000000UL;
    uint32_t exp  = src.u & 0x7F800000UL;
    uint32_t frac = src.u & 0x007FFFFFUL;

    if (exp == 0 && frac == 0) {
        dst.u = ((uint64_t)sign) << 32;
        return dst.d;
    }

    uint64_t d_sign = ((uint64_t)sign) << 32;
    uint64_t d_exp  = ((((uint64_t)(exp >> 23)) - 127 + 1023) & 0x7FF) << 52;
    uint64_t d_frac = ((uint64_t)frac) << 29;

    dst.u = d_sign | d_exp | d_frac;
    return dst.d;
}

float __truncdfsf2(double a) {
    union { double d; uint64_t u; } src;
    union { float f; uint32_t u; } dst;
    src.d = a;
    uint64_t sign = src.u & 0x8000000000000000ULL;
    uint64_t exp  = src.u & 0x7FF0000000000000ULL;
    uint64_t frac = src.u & 0x000FFFFFFFFFFFFFULL;

    if (exp == 0 && frac == 0) {
        dst.u = (uint32_t)(sign >> 32);
        return dst.f;
    }

    uint32_t f_sign = (uint32_t)(sign >> 32);
    int32_t  real_exp = (int32_t)(exp >> 52) - 1023;
    uint32_t f_exp;
    if (real_exp < -126) {
        f_exp = 0;
    } else if (real_exp > 127) {
        f_exp = 0xFF << 23;
    } else {
        f_exp = (uint32_t)((real_exp + 127) & 0xFF) << 23;
    }
    uint32_t f_frac = (uint32_t)(frac >> 29);

    dst.u = f_sign | f_exp | f_frac;
    return dst.f;
}

float __mulsf3(float a, float b) {
    (void)a; (void)b;
    return 0.0f;
}

int32_t __fixsfsi(float a) {
    (void)a;
    return 0;
}

int __ltdf2(double a, double b) {
    (void)a; (void)b;
    return 0;
}

double __floatunsidf(uint32_t a) {
    (void)a;
    return 0.0;
}

double __divdf3(double a, double b) {
    (void)a; (void)b;
    return 0.0;
}

double __adddf3(double a, double b) {
    (void)a; (void)b;
    return 0.0;
}

uint32_t __fixunsdfsi(double a) {
    (void)a;
    return 0;
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *p1 = s1, *p2 = s2;
    while (n--) {
        if (*p1 != *p2) return *p1 - *p2;
        p1++; p2++;
    }
    return 0;
}

int strncmp(const char *s1, const char *s2, size_t n) {
    if (n == 0) return 0;
    while (n-- && *s1 && *s1 == *s2) {
        if (n == 0 || *s1 == '\0') break;
        s1++; s2++;
    }
    return *(const unsigned char *)s1 - *(const unsigned char *)s2;
}

double __subdf3(double a, double b) {
    (void)a; (void)b;
    return 0.0;
}

double __muldf3(double a, double b) {
    (void)a; (void)b;
    return 0.0;
}

float __floatunsisf(uint32_t a) {
    (void)a;
    return 0.0f;
}

float __divsf3(float a, float b) {
    (void)a; (void)b;
    return 0.0f;
}
