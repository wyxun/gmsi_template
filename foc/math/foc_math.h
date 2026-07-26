/*******************************************************************************
 * @file    foc_math.h
 * @brief   Compatibility math functions using normalized-turn angles
 ******************************************************************************/

#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_angle.h"
#include "foc_math_types.h"

/**
 * @brief  计算以圈数为单位的正弦值
 * @param  qTurns  角度，单位：圈
 * @return         正弦值
 */
q_type foc_sin(q_type qTurns);
/**
 * @brief  计算以圈数为单位的余弦值
 * @param  qTurns  角度，单位：圈
 * @return         余弦值
 */
q_type foc_cos(q_type qTurns);
/**
 * @brief  根据直角坐标计算以圈数为单位的反正切
 * @param  qY  Y 分量
 * @param  qX  X 分量
 * @return     角度值，单位：圈
 */
q_type foc_atan2(q_type qY, q_type qX);
/**
 * @brief  定点开平方
 * @param  qValue  输入值
 * @return         平方根
 */
q_type foc_sqrt(q_type qValue);

#endif /* FOC_MATH_H */
