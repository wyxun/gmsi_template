/*******************************************************************************
 * @file    foc_math_types.h
 * @brief   Compatibility aliases for the unified FOC numeric backend
 ******************************************************************************/

#ifndef FOC_MATH_TYPES_H
#define FOC_MATH_TYPES_H

#include "foc_numeric.h"

typedef foc_scalar_t q_type;            /**< 定点标量类型别名（兼容） */

#define Q_SHIFT FOC_Q_FRACTION_BITS     /**< 定点小数位数 */
#define _Q(value) FOC_SCALAR(value)     /**< 浮点常量转定点 */
#define Q_ZERO FOC_ZERO                 /**< 定点 0 */
#define Q_HALF FOC_HALF                 /**< 定点 0.5 */
#define Q_ONE FOC_ONE                   /**< 定点 1.0 */

#define _D(value) ((double)foc_to_float(value))          /**< 定点转浮点（调试用） */
#define _D_ANG(value) ((double)foc_to_float(value) * 360.0) /**< 定点角度转度数（调试用） */

#endif /* FOC_MATH_TYPES_H */
