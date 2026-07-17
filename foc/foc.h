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

#include "control/foc_pid.h"
#include "control/foc_filter.h"
#include "control/foc_pll.h"
#include "control/foc_ltd.h"
#include "control/foc_feedforward.h"
#include "control/foc_controller.h"
#include "control/foc_ladrc.h"
#include "control/foc_smc.h"
#include "control/foc_sta.h"
#include "control/foc_dob.h"

#include "modulation/foc_modulation.h"

#include "hal/foc_hal_types.h"
#include "hal/foc_hal_pwm.h"
#include "hal/foc_hal_adc.h"
#include "hal/foc_hal.h"

#include "motor/motor_types.h"
#include "motor/motor.h"
#include "motor/motor_control.h"
#include "observer/foc_observer.h"
#include "observer/foc_hall.h"
#include "observer/foc_smo.h"
#include "observer/foc_nlfo.h"
#include "observer/foc_observer_selector.h"
#include "observer/foc_hfi.h"

#include "optimization/foc_optimization.h"
#include "optimization/foc_cogging.h"

#include "experimental/foc_experiment.h"
#include "experimental/foc_nsd.h"
#include "experimental/foc_identify.h"

#include "middleware/observer_lib.h"
#include "middleware/foc_core.h"

#include "app/foc_app.h"

#endif /* __FOC_H__ */
