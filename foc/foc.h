/*******************************************************************************
 * @file    foc.h
 * @brief   Universal-FOC 框架顶层统一 include
 *          用户只需 #include "foc/foc.h" 即可使用完整框架
 ******************************************************************************/

#ifndef __FOC_H__
#define __FOC_H__

#include "foc_config.h"

#include "math/foc_math_types.h"
#include "math/foc_math_port.h"
#include "math/foc_math.h"

#include "hal/foc_hal_types.h"
#include "hal/foc_hal_pwm.h"
#include "hal/foc_hal_adc.h"
#include "hal/foc_hal.h"

#include "motor/motor_types.h"
#include "motor/motor.h"

#include "middleware/observer_lib.h"
#include "middleware/foc_core.h"

#include "app/foc_app.h"

#endif /* __FOC_H__ */
