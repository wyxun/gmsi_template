/*******************************************************************************
 * @file    foc_math.c
 * @brief   FOC 数学函数实现
 ******************************************************************************/

#include "foc_math.h"

#if FOC_USE_FPU_HARDWARE
#include <math.h>
#endif

#if !FOC_USE_FPU_HARDWARE
static const int16_t s_hwSinTable[257] = {
    0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012,
    3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195,
    6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319,
    9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
    12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
    15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
    18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
    20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
    23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
    25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
    27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
    28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
    30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
    31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
    32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
    32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766, 32767
};

q_type foc_sin(q_type angle)
{
    uint16_t hwAngle = (uint16_t)angle;
    uint8_t  chQuadrant = hwAngle >> 14;
    uint16_t hwIndex = (hwAngle >> 6) & 0xFF;
    int16_t hwSin;
    switch (chQuadrant) {
        case 0: hwSin =  s_hwSinTable[hwIndex]; break;
        case 1: hwSin =  s_hwSinTable[256 - hwIndex]; break;
        case 2: hwSin = -s_hwSinTable[hwIndex]; break;
        case 3: hwSin = -s_hwSinTable[256 - hwIndex]; break;
    }
    return (q_type)hwSin;
}

q_type foc_cos(q_type angle)
{
    return foc_sin(angle + 16384);
}
#else
q_type foc_sin(q_type angle)
{
    return sinf(angle);
}

q_type foc_cos(q_type angle)
{
    return cosf(angle);
}
#endif

q_type foc_atan2(q_type y, q_type x)
{
#if FOC_USE_FPU_HARDWARE
    return atan2f(y, x);
#else
    if (x == 0 && y == 0) return 0;
    static const int16_t atan_table[] = {
        8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1
    };
    int32_t xi = (int32_t)x;
    int32_t yi = (int32_t)y;
    int32_t angle = 0;
    if (xi < 0) {
        if (yi >= 0) {
            int32_t temp = xi;
            xi = yi;
            yi = -temp;
            angle = 16384;
        } else {
            int32_t temp = xi;
            xi = -yi;
            yi = temp;
            angle = -16384;
        }
    }
    for (int i = 0; i < 14; i++) {
        int32_t x_next, y_next;
        if (yi >= 0) {
            x_next = xi + (yi >> i);
            y_next = yi - (xi >> i);
            angle += atan_table[i];
        } else {
            x_next = xi - (yi >> i);
            y_next = yi + (xi >> i);
            angle -= atan_table[i];
        }
        xi = x_next;
        yi = y_next;
    }
    return (q_type)angle;
#endif
}

q_type foc_sqrt(q_type x)
{
#if FOC_USE_FPU_HARDWARE
    if (x <= 0.0f) return 0.0f;
    return sqrtf(x);
#else
    if (x <= 0) return 0;
    uint32_t op  = (uint32_t)x << 15;
    uint32_t res = 0;
    uint32_t one = 1uL << 30;
    while (one > op) { one >>= 2; }
    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res += one << 1;
        }
        res >>= 1;
        one >>= 2;
    }
    return (q_type)res;
#endif
}
