/*******************************************************************************
 * @file    motor_private.h
 * @brief   Motor implementation layout, private to motor implementation files
 ******************************************************************************/

#ifndef MOTOR_PRIVATE_H
#define MOTOR_PRIVATE_H

#include "motor_types.h"

#define MOTOR_IMPL_MAGIC 0x4D4F544FU

#if defined(__GNUC__) || defined(__clang__)
/*
 * The public handle has declared byte-array storage. This local alias type
 * permits implementation lvalue access without disabling strict aliasing for
 * the rest of the firmware. GCC, Clang, and ARM GCC support may_alias.
 */
#define MOTOR_PRIVATE_MAY_ALIAS __attribute__((__may_alias__))
#else
#error "Opaque motor storage requires compiler may_alias support"
#endif

typedef struct MOTOR_PRIVATE_MAY_ALIAS {
    motor_params_t          tParams;
    motor_state_t           tRt;
    foc_hal_t               tHal;
    motor_control_t         tControl;
    phase_current_handle_t  tCurrent;
    /* One copy backs both bindings because different sources are rejected. */
    foc_position_source_if_t tPositionSource;
    foc_angle_t             tMechanicalAngle;
    foc_scalar_t            qMechanicalSpeed;
    foc_position_valid_flag_e eMechanicalValidFlags;
    motor_time_if_t         tTime;
    motor_sync_if_t         tSync;
    foc_scalar_t            qInitialAngle;
    foc_scalar_t            qOpenLoopSpeed;
    foc_scalar_t            qAcceleration;
    foc_scalar_t            qOpenLoopCommandSpeed;
    foc_scalar_t            qHighFrequencyPeriod;
    foc_scalar_t            qLowFrequencyPeriod;
    foc_position_config_t   tPositionConfig;
    uint32_t                wStartupDelayMs;
    uint32_t                wStartupStartMs;
    motor_startup_phase_e   eStartupPhase;
    motor_command_e         ePendingCommand;
    uint32_t                wMagic;
    bool                    bCommandPending;
    bool                    bPwmEnabled;
    bool                    bInitialPositionSourceBound;
    bool                    bTargetPositionSourceBound;
    bool                    bOuterLoopActive;
    bool                    bHighFrequencyStepInProgress;
    bool                    bLowFrequencyStepInProgress;
} motor_impl_t;

_Static_assert(sizeof(motor_impl_t) <= MOTOR_HANDLE_STORAGE_SIZE - 32U,
               "motor implementation must reserve 32 bytes for expansion");
_Static_assert(_Alignof(motor_handle_t) >= _Alignof(motor_impl_t),
               "motor_handle_t private storage is insufficiently aligned");

static inline motor_impl_t *motor_private(motor_handle_t *ptMotor)
{
    return (motor_impl_t *)(void *)ptMotor;
}

static inline const motor_impl_t *motor_private_const(
    const motor_handle_t *ptMotor)
{
    return (const motor_impl_t *)(const void *)ptMotor;
}

static inline bool motor_private_is_initialized(
    const motor_handle_t *ptMotor)
{
    return ptMotor != NULL &&
           motor_private_const(ptMotor)->wMagic == MOTOR_IMPL_MAGIC;
}

static inline uintptr_t motor_private_enter(const motor_impl_t *ptImpl)
{
    return ptImpl->tSync.fnEnter != NULL ?
        ptImpl->tSync.fnEnter(ptImpl->tSync.pContext) : 0U;
}

static inline void motor_private_exit(const motor_impl_t *ptImpl,
                                      uintptr_t wState)
{
    if (ptImpl->tSync.fnExit != NULL) {
        ptImpl->tSync.fnExit(ptImpl->tSync.pContext, wState);
    }
}

foc_result_t motor_private_SetDuty(motor_handle_t *, q_type, q_type, q_type);
foc_result_t motor_private_Enable(motor_handle_t *, bool);
foc_result_t motor_private_CalibrateCurrent(motor_handle_t *);
foc_result_t motor_private_SampleCurrent(motor_handle_t *);

#endif /* MOTOR_PRIVATE_H */
