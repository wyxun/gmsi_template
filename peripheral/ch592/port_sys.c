#include "peripheral.h"
#include <stdint.h>
#include <stdbool.h>
#include "CH59x_common.h"

/* 系统内核时钟强符号回传，完美覆盖 perf_counter 中的弱符号 */
uint32_t SystemCoreClock = 60000000UL;

uint32_t get_system_core_clock_hz(void)
{
    return SystemCoreClock;
}

/* WCH 官方 UART 库要求的 GetSysClock 实现 */
uint32_t GetSysClock(void)
{
    return SystemCoreClock;
}

/* 系统时钟配置 — 使用 WCH 官方 sys_safe_access 宏操作 RWA 寄存器
 *
 * 之前自定义 PORT_SAFE_ACCESS_ENABLE 可能未正确解锁 SAM，
 * 导致 R32_CLK_SYS_CFG 写入被硬件忽略，系统仍跑 6.4MHz。
 * 现在直接用 CH59x_sys.h 提供的 sys_safe_access_enable() 宏。 */
static void SystemClock_Config(void)
{
    uint8_t chip_type = 0;
    if (((*(uint32_t *)0x7F010) & 0xFF) == 0xA4) {
        chip_type = 1;
    }

    /* 1. PLL 配置前置 */
    sys_safe_access_enable();
    R8_PLL_CONFIG &= ~(1 << 5);
    sys_safe_access_disable();

    /* 2. 选择 PLL 60MHz */
    sys_safe_access_enable();
    R32_CLK_SYS_CFG = (1 << 6)           /* PLL 时钟源 */
                    | (0x48 & 0x1f)       /* 分频系数 8 → 480/8=60MHz */
                    | RB_TX_32M_PWR_EN    /* 32MHz HSE 上电 */
                    | RB_PLL_PWR_EN;      /* PLL 上电 */
    __nop(); __nop(); __nop(); __nop();
    sys_safe_access_disable();

    /* 3. Flash 等待周期 */
    sys_safe_access_enable();
    R8_FLASH_CFG = chip_type ? 0x53 : 0x52;
    sys_safe_access_disable();

    /* 4. 使能 FLASH 时钟加速 */
    sys_safe_access_enable();
    R8_PLL_CONFIG |= (1 << 7);
    sys_safe_access_disable();
}

/* 官方中断向量表默认调用的时钟/SysTick初始化入口 */
void SystemInit(void)
{
    /* 1. 配置系统时钟至 60MHz (必须在 SysTick 之前，SysTick 使用 HCLK) */
    SystemClock_Config();

    /* 2. 重置并配置硬件 SysTick 全局自减计数器 */
    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    SysTick->SR = 0;

    /* 3. 配置 60MHz 时钟下 1ms 的比较中断值 (60,000 counts) */
    SysTick->CMP = 60000UL - 1U;

    /* 4. 在 PFIC 中断控制器中使能 SysTick 中断线 (SysTick_IRQn = 12)
     *    ⚠️ 这是关键步骤！CH592 QingKe V4C 内核要求所有中断
     *    必须在 PFIC 中显式使能，否则外设中断无法送达 CPU。 */
    PFIC_EnableIRQ(SysTick_IRQn);

    /* 5. 开启 System Timer，使用 HCLK 作为基准并开启中断 */
    SysTick->CTLR = SysTick_CTLR_INIT |
                    SysTick_CTLR_STRE |
                    SysTick_CTLR_STCLK |
                    SysTick_CTLR_STIE |
                    SysTick_CTLR_STE;
}

/* 串口0初始化 — 直接用 WCH 官方 API 排除自定义配置差异 */
static void Uart0_Init(uint32_t baudrate)
{
    /* 1. GPIO: TX 先拉高再设推挽输出, RX 设输入上拉 (WCH EVT 标准做法) */
    GPIOB_SetBits(GPIO_Pin_7);
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);

    /* 2. 与 WCH 官方例程完全一致，DL 对应 60MHz 系统时钟 */
    UART0_BaudRateCfg(baudrate);

    R8_UART0_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART0_LCR = RB_LCR_WORD_SZ;
    R8_UART0_IER = RB_IER_TXD_EN;
    R8_UART0_DIV = 1;
}

/* 外设初始化总入口 */
void peripheral_Init(void)
{
    /* 初始化系统时钟与 SysTick 定时器 */
    SystemInit();

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
    uint32_t mask = 8;
    __asm__ __volatile__("csrrs zero, mstatus, %0" :: "r"(mask));
}

void peripheral_DisableIRQ(void)
{
    uint32_t mask = 8;
    __asm__ __volatile__("csrrc zero, mstatus, %0" :: "r"(mask));
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

/* 64位无符号右移逻辑实现，避免产生递归的编译器内置函数调用 */
uint64_t __lshrdi3(uint64_t a, int b) {
    if (b <= 0) return a;
    if (b >= 64) return 0;
    
    union {
        uint64_t val;
        struct {
            uint32_t low;
            uint32_t high;
        } words;
    } src, dst;
    
    src.val = a;
    if (b >= 32) {
        dst.words.low = src.words.high >> (b - 32);
        dst.words.high = 0;
    } else {
        dst.words.low = (src.words.low >> b) | (src.words.high << (32 - b));
        dst.words.high = src.words.high >> b;
    }
    return dst.val;
}

/* 64位无符号左移逻辑实现，避免产生递归的编译器内置函数调用 */
uint64_t __ashldi3(uint64_t a, int b) {
    if (b <= 0) return a;
    if (b >= 64) return 0;
    
    union {
        uint64_t val;
        struct {
            uint32_t low;
            uint32_t high;
        } words;
    } src, dst;
    
    src.val = a;
    if (b >= 32) {
        dst.words.high = src.words.low << (b - 32);
        dst.words.low = 0;
    } else {
        dst.words.high = (src.words.high << b) | (src.words.low >> (32 - b));
        dst.words.low = src.words.low << b;
    }
    return dst.val;
}

/* 单精度浮点不相等比较内置辅助函数实现，避开 C 编译器 float 比较递归 */
__attribute__((optnone)) int __nesf2(float a, float b) {
    union { float f; uint32_t u; } ua, ub;
    ua.f = a;
    ub.f = b;
    
    uint32_t exp_a = ua.u & 0x7F800000UL;
    uint32_t frac_a = ua.u & 0x007FFFFFUL;
    uint32_t exp_b = ub.u & 0x7F800000UL;
    uint32_t frac_b = ub.u & 0x007FFFFFUL;
    
    if ((exp_a == 0x7F800000UL && frac_a != 0) ||
        (exp_b == 0x7F800000UL && frac_b != 0)) {
        return 1;
    }
    
    if (((ua.u & 0x7FFFFFFFUL) == 0) && ((ub.u & 0x7FFFFFFFUL) == 0)) {
        return 0;
    }
    
    return (ua.u != ub.u) ? 1 : 0;
}

/* 双精度浮点相等比较内置辅助函数实现，避开 C 编译器 double 比较递归 */
__attribute__((optnone)) int __eqdf2(double a, double b) {
    union { double d; uint64_t u; } ua, ub;
    ua.d = a;
    ub.d = b;
    
    uint64_t exp_a = ua.u & 0x7FF0000000000000ULL;
    uint64_t frac_a = ua.u & 0x000FFFFFFFFFFFFFULL;
    uint64_t exp_b = ub.u & 0x7FF0000000000000ULL;
    uint64_t frac_b = ub.u & 0x000FFFFFFFFFFFFFULL;
    
    if ((exp_a == 0x7FF0000000000000ULL && frac_a != 0) ||
        (exp_b == 0x7FF0000000000000ULL && frac_b != 0)) {
        return 1;
    }
    
    if (((ua.u & 0x7FFFFFFFFFFFFFFFULL) == 0) && ((ub.u & 0x7FFFFFFFFFFFFFFFULL) == 0)) {
        return 0;
    }
    
    return (ua.u == ub.u) ? 0 : 1;
}
