/*******************************************************************************
 * @file    motor_types.h
 * @brief   电机对象核心类型定义
 ******************************************************************************/

#ifndef __MOTOR_TYPES_H__
#define __MOTOR_TYPES_H__

#include <stddef.h>
#include <stdint.h>

#include "foc_math_types.h"
#include "foc_angle.h"
#include "foc_hal.h"
#include "motor_control_types.h"
#include "motor_position.h"
#include "perf_counter.h"

typedef enum {
    MOTOR_STATE_IDLE        = 0,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} motor_state_e;

typedef enum { MOTOR_STARTUP_IDLE = 0, MOTOR_STARTUP_CALIBRATE,
    MOTOR_STARTUP_WAIT_DELAY, MOTOR_STARTUP_ENABLE,
} motor_startup_phase_e;
typedef enum { MOTOR_COMMAND_NONE = 0, MOTOR_COMMAND_START,
    MOTOR_COMMAND_STOP,
} motor_command_e;

typedef enum {
    MOTOR_FAULT_NONE            = 0U,
    MOTOR_FAULT_HARDWARE        = 1U << 0,
    MOTOR_FAULT_CURRENT_SAMPLE  = 1U << 1,
    MOTOR_FAULT_INVALID_COMMAND = 1U << 2,
} motor_fault_e;

typedef struct {
    uint32_t wResistanceMilliOhm;
    uint32_t wLdMicroHenry;
    uint32_t wLqMicroHenry;
    uint32_t wBackEmfMicroVoltPerRadSec;
    uint32_t wInertiaNanoKgM2;
    uint32_t wRatedVoltageMilliVolt;
    uint32_t wRatedCurrentMilliAmp;
    uint8_t chPolePairs;
} motor_params_t;

typedef struct {
    foc_angle_t         tThetaE;
    q_type              qOmegaE;
    q_type              qVbus;
    uint32_t            wFaults;
    motor_state_e       eRunState;
} motor_state_t;

typedef struct {
    void *pContext;
    uint32_t (*fnGetMilliseconds)(void *pContext);
} motor_time_if_t;

typedef struct {
    void *pContext;
    uintptr_t (*fnEnter)(void *pContext);
    void (*fnExit)(void *pContext, uintptr_t wState);
} motor_sync_if_t;

typedef struct {
    motor_control_mode_e eControlMode;
    /* Descriptors are copied by motor_Start; their pContext must stay valid. */
    const foc_position_source_if_t *ptInitialPositionSource;
    const foc_position_source_if_t *ptTargetPositionSource;
    /* Electrical turns, turns/s, and turns/s^2 for deterministic open loop. */
    foc_scalar_t qInitialAngle;
    foc_scalar_t qOpenLoopSpeed;
    foc_scalar_t qAcceleration;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    /* Outer-loop references: mechanical turns/s and wrapped mechanical turn. */
    foc_scalar_t qSpeedReference;
    foc_scalar_t qPositionReference;
} motor_run_config_t;

typedef struct {
    motor_params_t          tParams;
    foc_hal_t               tHal;
    motor_control_config_t  tControl;
    current_sensing_type_t  eTopology;
    motor_time_if_t         tTime;
    motor_sync_if_t         tSync;
    /* Seconds per deterministic loop step, normalized in q_type. */
    foc_scalar_t            qHighFrequencyPeriod;
    foc_scalar_t            qLowFrequencyPeriod;
    /* Mechanical-to-electrical angle/speed conversion. */
    foc_position_config_t   tPosition;
    uint32_t                wStartupDelayMs;
} motor_config_t;

/* Per-instance RAM and public ABI capacity. motor_private.h enforces the limit. */
#define MOTOR_HANDLE_STORAGE_SIZE 512U

typedef union {
    max_align_t tAlignment;
    uint8_t achPrivate[MOTOR_HANDLE_STORAGE_SIZE];
} motor_handle_t;

_Static_assert(sizeof(motor_handle_t) == MOTOR_HANDLE_STORAGE_SIZE,
               "motor_handle_t storage size includes unexpected padding");

typedef struct {
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
} motor_phase_current_t;

typedef struct {
    motor_state_e eRunState;
    uint32_t wFaults;
    motor_phase_current_t tPhaseCurrent;
    foc_dq_t tCurrent;
    foc_dq_t tVoltage;
    foc_duty_abc_t tDuty;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    foc_scalar_t qVbus;
    foc_adc_calib_t tCurrentCalibration;
    bool bPwmEnabled;
    motor_startup_phase_e eStartupPhase;
    motor_command_e ePendingCommand;
} motor_snapshot_t;

#endif /* __MOTOR_TYPES_H__ */
