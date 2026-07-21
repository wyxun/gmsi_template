#include "halcordic.h"
#include "stm32g4xx.h"
#include <math.h>

void hal_cordic_Init(void)
{
    /* 开启 CORDIC 外设时钟 */
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    __DSB();
}

void hal_cordic_SinCos(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos)
{
    /* 1. Turns [0, 1) 映射到 q1.31 定点二进制角度 BAM32 */
    uint32_t u32Turns = (uint32_t)(qTurns * 4294967296.0f);
    int32_t q31Angle = (int32_t)u32Turns;

    /* 2. 配置 CSR：Cosine功能，24次迭代(PRECISION=6), 双结果读取(NRES=1) */
    CORDIC->CSR = (6U << CORDIC_CSR_PRECISION_Pos) | 
                  (0U << CORDIC_CSR_FUNC_Pos) | 
                  CORDIC_CSR_NRES;

    /* 3. 写入 WDATA 触发硬件计算 (零等待总线阻塞式) */
    CORDIC->WDATA = q31Angle;

    /* 4. 依次读取 X_RESULT (cos) 和 Y_RESULT (sin) */
    int32_t qCos = CORDIC->RDATA;
    int32_t qSin = CORDIC->RDATA;

    /* 5. 格式转换回 float */
    *pqCos = (float)qCos * 4.6566128730773926e-10f;
    *pqSin = (float)qSin * 4.6566128730773926e-10f;
}

foc_scalar_t hal_cordic_Atan2(foc_scalar_t qY, foc_scalar_t qX)
{
    float fMax = fmaxf(fabsf(qX), fabsf(qY));
    if (fMax <= 0.0f) {
        return 0.0f;
    }
    
    /* 归一化输入，防止 CORDIC Q1.31 溢出 */
    int32_t q31X = (int32_t)((qX / fMax) * 2147483647.0f);
    int32_t q31Y = (int32_t)((qY / fMax) * 2147483647.0f);

    /* 配置 CSR：Phase (atan2), NARGS=1 (双参数，X+Y), NRES=0 (单结果) */
    CORDIC->CSR = (6U << CORDIC_CSR_PRECISION_Pos) | 
                  (2U << CORDIC_CSR_FUNC_Pos) | 
                  CORDIC_CSR_NARGS;

    /* 写入 X 紧接着写入 Y 触发计算 */
    CORDIC->WDATA = q31X;
    CORDIC->WDATA = q31Y;

    /* 读取 RDATA 获得 BAM32 角度 */
    int32_t qAngle = CORDIC->RDATA;

    /* 转换为 turns 范围 [0, 1) */
    float fTurns = (float)qAngle * 2.3283064365386963e-10f;
    if (fTurns < 0.0f) {
        fTurns += 1.0f;
    }
    return fTurns;
}
