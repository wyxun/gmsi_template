/*******************************************************************************
 * @file    motor_control.c
 * @brief   Per-instance FOC loop orchestration
 ******************************************************************************/

#include "motor_control.h"
#include "motor_private.h"
#include "foc_hf_profile.h"
#include "perf_counter.h"

#include <stddef.h>

static foc_result_t motor_control_begin_step(motor_impl_t *, bool);
static void motor_control_end_step(motor_impl_t *, bool);

void motor_SetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tHfCommand.tVoltageReference = (foc_dq_t){qD, qQ};
        motor_private_exit(p, s);
    }
}

void motor_SetCurrentReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tHfCommand.tCurrentReference = (foc_dq_t){qD, qQ};
        motor_private_exit(p, s);
    }
}

void motor_SetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tHfCommand.qSpeedReference = qSpeed;
        motor_private_exit(p, s);
    }
}

void motor_SetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tHfCommand.qPositionReference = qPosition;
        motor_private_exit(p, s);
    }
}

foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    motor_hf_command_t *ptCommand;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    ptCommand = &ptImpl->tHfCommand;
    foc_result_t begin = motor_control_begin_step(ptImpl, false);
    if (begin != FOC_RESULT_OK) return begin;
    uintptr_t s = motor_private_enter(ptImpl);
    motor_private_DrainPendingEvents(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING &&
        ptImpl->tRt.eRunState != MOTOR_STATE_STARTING) {
        motor_private_exit(ptImpl, s);
        motor_control_end_step(ptImpl, false);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    motor_control_mode_e mode = ptCommand->eMode;
    foc_scalar_t position_ref = ptCommand->qPositionReference;
    foc_scalar_t speed_ref = ptCommand->qSpeedReference;
    foc_scalar_t id_ref = ptCommand->tCurrentReference.qD;
    foc_angle_t mechanical_angle = ptImpl->tMechanicalAngle;
    foc_scalar_t mechanical_speed = ptImpl->qMechanicalSpeed;
    foc_position_valid_flag_e mechanical_valid =
        ptImpl->chMechanicalValidFlags;
    bool direct_source = ptImpl->bOuterLoopActive;
    motor_private_exit(ptImpl, s);
    if (mode >= MOTOR_CONTROL_SPEED && !direct_source) {
        motor_control_end_step(ptImpl, false);
        return FOC_RESULT_OK;
    }
    if (mode == MOTOR_CONTROL_POSITION) {
        if ((mechanical_valid & FOC_POSITION_VALID_MECHANICAL_ANGLE) == 0U) {
            goto fail;
        }
        foc_scalar_t error = foc_angle_diff(
            foc_angle_from_scalar(position_ref), mechanical_angle);
        speed_ref =
            ptImpl->tControlConfig.tPositionController.fnStep(
                ptImpl->tControlConfig.tPositionController.pContext,
                error, FOC_ZERO);
    }
    if (mode >= MOTOR_CONTROL_SPEED) {
        if ((mechanical_valid & FOC_POSITION_VALID_MECHANICAL_SPEED) == 0U) {
            goto fail;
        }
        foc_scalar_t iq_ref =
            ptImpl->tControlConfig.tSpeedController.fnStep(
                ptImpl->tControlConfig.tSpeedController.pContext,
                speed_ref, mechanical_speed);
        s = motor_private_enter(ptImpl);
        bool stopping = ptImpl->bCommandPending &&
                        ptImpl->chPendingCommand == MOTOR_COMMAND_STOP;
        if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING || stopping) {
            ptImpl->bLowFrequencyStepInProgress = false;
            motor_private_exit(ptImpl, s);
            return stopping ? FOC_RESULT_BUSY : FOC_RESULT_INVALID_ARGUMENT;
        }
        ptCommand->qSpeedReference = speed_ref;
        ptCommand->tCurrentReference = (foc_dq_t){id_ref, iq_ref};
        ptImpl->bLowFrequencyStepInProgress = false;
        motor_private_exit(ptImpl, s);
    } else {
        motor_control_end_step(ptImpl, false);
    }
    return FOC_RESULT_OK;
fail:
    motor_control_end_step(ptImpl, false);
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_INVALID_COMMAND);
    return FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t motor_control_begin_step(motor_impl_t *impl, bool hf)
{
    uintptr_t state = motor_private_enter(impl);
    bool *active = hf ? &impl->bHighFrequencyStepInProgress :
                        &impl->bLowFrequencyStepInProgress;
    if (*active) {
        motor_private_exit(impl, state);
        return FOC_RESULT_BUSY;
    }
    *active = true;
    motor_private_exit(impl, state);
    return FOC_RESULT_OK;
}

static void motor_control_end_step(motor_impl_t *impl, bool hf)
{
    uintptr_t state = motor_private_enter(impl);
    if (hf) impl->bHighFrequencyStepInProgress = false;
    else impl->bLowFrequencyStepInProgress = false;
    motor_private_exit(impl, state);
}
