/*******************************************************************************
 * @file    motor_private.h
 * @brief   Motor implementation layout, private to motor implementation files
 ******************************************************************************/

#ifndef MOTOR_PRIVATE_H
#define MOTOR_PRIVATE_H

#include "motor_types.h"

#define MOTOR_IMPL_MAGIC 0x4D4F544FU
#define MOTOR_EVENT_CAPACITY 4U

/*
 * Compact 8-byte event record. wSequence is an independent monotonic
 * event counter (never milliseconds or high-frequency sample indices).
 * chMeta packs from-state (3 bits), to-state (3 bits), and detail (2
 * bits, e.g. motor_position_role_e). hwPayload is one event-specific
 * 16-bit numeric value: fault bits for FAULT, command | result << 8
 * for command events, old | new << 8 flags or phases for validity and
 * transition events. Realtime code never formats strings for events.
 */
typedef struct {
    uint32_t wSequence;
    uint16_t hwPayload;
    uint8_t chType;
    uint8_t chMeta;
} motor_event_record_t;

#define MOTOR_EVENT_META(eFrom, eTo, chDetail) \
    ((uint8_t)((uint8_t)(eFrom) | ((uint8_t)(eTo) << 3) | \
               ((uint8_t)(chDetail) << 6)))
#define MOTOR_EVENT_META_FROM(chMeta) ((uint8_t)((chMeta) & 0x7U))
#define MOTOR_EVENT_META_TO(chMeta)   ((uint8_t)(((chMeta) >> 3) & 0x7U))
#define MOTOR_EVENT_META_DETAIL(chMeta) ((uint8_t)((chMeta) >> 6))

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

/*
 * Layout is ordered by alignment so the private implementation stays
 * inside the fixed public handle storage on both 32-bit targets and
 * 64-bit hosts: pointer-bearing blocks first, then 4-byte, 2-byte, and
 * 1-byte members. Enum state is stored narrowed to uint8_t; flags that
 * are never address-taken are bit-packed. Do not grow fields without
 * checking the static assert below.
 */
typedef struct MOTOR_PRIVATE_MAY_ALIAS {
    /* 8-byte aligned interface and control blocks. */
    foc_hal_t               tHal;
    motor_control_t         tControl;
    /* One copy backs both bindings because different sources are rejected. */
    foc_position_source_if_t tPositionSource;
    motor_time_if_t         tTime;
    motor_sync_if_t         tSync;
    /* 4-byte block. */
    motor_state_t           tRt;
    phase_current_handle_t  tCurrent;
    foc_position_config_t   tPositionConfig;
    foc_angle_t             tMechanicalAngle;
    foc_scalar_t            qMechanicalSpeed;
    foc_angle_t             tOpenLoopAngle;
    foc_angle_t             tCandidateAngle;
    foc_scalar_t            qCandidateSpeed;
    foc_scalar_t            qAngleError;
    foc_scalar_t            qBlendFactor;
    foc_scalar_t            qOpenLoopSpeed;
    foc_scalar_t            qAcceleration;
    foc_scalar_t            qOpenLoopCommandSpeed;
    foc_scalar_t            qHighFrequencyPeriod;
    foc_scalar_t            qTransitionMinimumConfidence;
    foc_scalar_t            qTransitionMinimumSpeed;
    foc_scalar_t            qTransitionMaximumAngleError;
    foc_angle_t             tTransitionStartAngle;
    foc_scalar_t            qTransitionStartSpeed;
    uint32_t                wStartupDelayMs;
    uint32_t                wStartupStartMs;
    /* Diagnostic bring-up timestamp; only used under FOC_ENABLE_DIAGNOSTIC. */
    uint32_t                wDiagnosticStartMs;
    uint32_t                wTransitionTimeoutMs;
    uint32_t                wPositionSampleTimestamp;
    uint32_t                wNextEventSequence;
    uint32_t                wEventOverwriteCount;
    uint32_t                wMagic;
    motor_event_record_t    atEvents[MOTOR_EVENT_CAPACITY];
    /* 2-byte block. */
    uint16_t                hwTransitionQualificationSamples;
    uint16_t                hwTransitionBlendSamples;
    uint16_t                hwTransitionSampleCount;
    /* 1-byte block: flag bytes and narrowed enum state. */
    uint8_t                 chActiveValidFlags;
    uint8_t                 chCandidateValidFlags;
    uint8_t                 chMechanicalValidFlags;
    uint8_t                 chStartupPhase;
    uint8_t                 chPendingCommand;
    uint8_t                 chEventHead;
    uint8_t                 chEventCount;
    /* Address-taken re-entrancy flags stay plain bool. */
    bool                    bHighFrequencyStepInProgress;
    bool                    bLowFrequencyStepInProgress;
    /* Bit-packed flags (never address-taken). */
    uint8_t                 bCommandPending : 1;
    uint8_t                 bPwmEnabled : 1;
    uint8_t                 bInitialPositionSourceBound : 1;
    uint8_t                 bTargetPositionSourceBound : 1;
    uint8_t                 bOuterLoopActive : 1;
    /* Fixed-duty diagnostic output active (FOC_ENABLE_DIAGNOSTIC only). */
    uint8_t                 bDiagnosticActive : 1;
} motor_impl_t;

_Static_assert(sizeof(motor_impl_t) <= MOTOR_HANDLE_STORAGE_SIZE,
               "motor implementation exceeds public handle storage");
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
void motor_private_AppendEvent(motor_impl_t *, motor_event_type_e,
                               motor_state_e, motor_state_e,
                               uint8_t, uint16_t);
#endif /* MOTOR_PRIVATE_H */
