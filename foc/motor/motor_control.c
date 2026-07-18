/*******************************************************************************
 * @file    motor_control.c
 * @brief   Per-instance FOC loop orchestration
 ******************************************************************************/

#include "motor_control.h"
#include "motor_private.h"

#include <stddef.h>

static foc_result_t motor_control_begin_step(motor_impl_t *, bool);
static void motor_control_end_step(motor_impl_t *, bool);

static foc_result_t motor_control_modulate(motor_control_t *ptControl)
{
    switch (ptControl->tConfig.eModulation) {
        case MOTOR_MODULATION_SPWM:
            return foc_spwm(&ptControl->tVoltageAlphaBeta,
                            &ptControl->tDuty);
        case MOTOR_MODULATION_THIRD_HARMONIC:
            return foc_third_harmonic_spwm(&ptControl->tVoltageAlphaBeta,
                                           &ptControl->tDuty);
        case MOTOR_MODULATION_SVPWM:
        default:
            return foc_svpwm(&ptControl->tVoltageAlphaBeta,
                             &ptControl->tDuty);
    }
}

void motor_SetVoltageReference(motor_handle_t *ptMotor,
                                      foc_scalar_t qD,
                                      foc_scalar_t qQ)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.tVoltageReference = (foc_dq_t){qD, qQ};
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
        p->tControl.tCurrentReference = (foc_dq_t){qD, qQ};
        motor_private_exit(p, s);
    }
}

void motor_SetSpeedReference(motor_handle_t *ptMotor,
                                    foc_scalar_t qSpeed)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.qSpeedReference = qSpeed;
        motor_private_exit(p, s);
    }
}

void motor_SetPositionReference(motor_handle_t *ptMotor,
                                       foc_scalar_t qPosition)
{
    if (motor_private_is_initialized(ptMotor)) {
        motor_impl_t *p = motor_private(ptMotor);
        uintptr_t s = motor_private_enter(p);
        p->tControl.qPositionReference = qPosition;
        motor_private_exit(p, s);
    }
}

foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    motor_control_t *ptControl;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    ptControl = &ptImpl->tControl;
    foc_result_t begin = motor_control_begin_step(ptImpl, false);
    if (begin != FOC_RESULT_OK) return begin;
    uintptr_t s = motor_private_enter(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING) {
        motor_private_exit(ptImpl, s);
        motor_control_end_step(ptImpl, false);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    motor_control_mode_e mode = ptControl->eMode;
    foc_scalar_t position_ref = ptControl->qPositionReference;
    foc_scalar_t speed_ref = ptControl->qSpeedReference;
    foc_scalar_t id_ref = ptControl->tCurrentReference.qD;
    foc_angle_t mechanical_angle = ptImpl->tMechanicalAngle;
    foc_scalar_t mechanical_speed = ptImpl->qMechanicalSpeed;
    foc_position_valid_flag_e mechanical_valid =
        ptImpl->eMechanicalValidFlags;
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
        speed_ref = foc_controller_Step(
            &ptControl->tConfig.tPositionController,
            error, FOC_ZERO);
    }
    if (mode >= MOTOR_CONTROL_SPEED) {
        if ((mechanical_valid & FOC_POSITION_VALID_MECHANICAL_SPEED) == 0U) {
            goto fail;
        }
        foc_scalar_t iq_ref = foc_controller_Step(
            &ptControl->tConfig.tSpeedController,
            speed_ref, mechanical_speed);
        s = motor_private_enter(ptImpl);
        bool stopping = ptImpl->bCommandPending &&
                        ptImpl->ePendingCommand == MOTOR_COMMAND_STOP;
        if (ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING || stopping) {
            ptImpl->bLowFrequencyStepInProgress = false;
            motor_private_exit(ptImpl, s);
            return stopping ? FOC_RESULT_BUSY : FOC_RESULT_INVALID_ARGUMENT;
        }
        ptControl->qSpeedReference = speed_ref;
        ptControl->tCurrentReference = (foc_dq_t){id_ref, iq_ref};
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

static foc_result_t motor_control_commit_hf(
    motor_impl_t *impl, const motor_control_t *work,
    foc_angle_t angle, foc_scalar_t speed,
    const foc_position_output_t *output, bool *hardware_failed)
{
    uintptr_t state = motor_private_enter(impl);
    bool stopping = impl->bCommandPending &&
                    impl->ePendingCommand == MOTOR_COMMAND_STOP;
    if (impl->tRt.eRunState != MOTOR_STATE_RUNNING ||
        impl->tRt.wFaults != MOTOR_FAULT_NONE ||
        !impl->bPwmEnabled || stopping) {
        impl->bHighFrequencyStepInProgress = false;
        motor_private_exit(impl, state);
        return stopping ? FOC_RESULT_BUSY : FOC_RESULT_INVALID_ARGUMENT;
    }
    foc_result_t result = foc_hal_SetDuty(&impl->tHal.tPwm,
        work->tDuty.qU, work->tDuty.qV, work->tDuty.qW);
    *hardware_failed = result != FOC_RESULT_OK;
    if (result == FOC_RESULT_OK) {
        impl->tRt.tThetaE = angle;
        impl->tRt.qOmegaE = speed;
        impl->qOpenLoopCommandSpeed = speed;
        impl->tMechanicalAngle = output->tMechanicalAngle;
        impl->qMechanicalSpeed = output->qMechanicalSpeed;
        impl->eMechanicalValidFlags =
            (foc_position_valid_flag_e)(output->eValidFlags &
             (FOC_POSITION_VALID_MECHANICAL_ANGLE |
              FOC_POSITION_VALID_MECHANICAL_SPEED));
        impl->tControl.tCurrentAlphaBeta = work->tCurrentAlphaBeta;
        impl->tControl.tCurrent = work->tCurrent;
        impl->tControl.tVoltage = work->tVoltage;
        impl->tControl.tVoltageAlphaBeta = work->tVoltageAlphaBeta;
        impl->tControl.tDuty = work->tDuty;
    }
    impl->bHighFrequencyStepInProgress = false;
    motor_private_exit(impl, state);
    return result;
}

foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor)
{
    motor_impl_t *ptImpl;
    motor_control_t *ptControl;
    foc_result_t eResult;
    foc_dq_t voltage_ref, current_ref;
    motor_control_t work;
    motor_control_mode_e mode;
    uintptr_t s;

    if (!motor_private_is_initialized(ptMotor)) {
        return ptMotor == NULL ? FOC_RESULT_NULL :
                                 FOC_RESULT_INVALID_ARGUMENT;
    }
    ptImpl = motor_private(ptMotor);
    eResult = motor_control_begin_step(ptImpl, true);
    if (eResult != FOC_RESULT_OK) return eResult;
    s = motor_private_enter(ptImpl);
    if (ptImpl->tRt.eRunState != MOTOR_STATE_STARTING &&
        ptImpl->tRt.eRunState != MOTOR_STATE_RUNNING) {
        motor_private_exit(ptImpl, s);
        motor_control_end_step(ptImpl, true);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptControl = &ptImpl->tControl;
    work = *ptControl;
    mode = ptControl->eMode;
    voltage_ref = ptControl->tVoltageReference;
    current_ref = ptControl->tCurrentReference;
    bool direct_source = ptImpl->bInitialPositionSourceBound;
    bool candidate_source = ptImpl->bTargetPositionSourceBound;
    foc_position_source_if_t source = ptImpl->tPositionSource;
    foc_angle_t angle = ptImpl->tRt.tThetaE;
    foc_scalar_t speed = ptImpl->qOpenLoopCommandSpeed;
    motor_private_exit(ptImpl, s);
    eResult = motor_private_SampleCurrent(ptMotor);
    if (eResult != FOC_RESULT_OK) {
        motor_control_end_step(ptImpl, true);
        return eResult;
    }
    foc_position_output_t output = {0};
    if (direct_source || candidate_source) {
        /* Timestamp zero means that this scheduler supplied no clock.
         * Freshness qualification and handover policy belong to Task6. */
        foc_position_input_t input = {.qSamplePeriod =
                                      ptImpl->qHighFrequencyPeriod};
        eResult = foc_position_source_Step(&source, &input, &output);
        if (eResult != FOC_RESULT_OK) goto fail;
        if (output.wFaults != 0U) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto fail;
        }
        eResult = foc_position_ApplyMechanicalConfig(
            &ptImpl->tPositionConfig, &output);
        if (eResult != FOC_RESULT_OK) goto fail;
    }
    if (direct_source) {
        if ((output.eValidFlags & FOC_POSITION_VALID_ELECTRICAL_ANGLE) == 0U) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
            goto fail;
        }
        angle = output.tElectricalAngle;
        if ((output.eValidFlags & FOC_POSITION_VALID_ELECTRICAL_SPEED) != 0U)
            speed = output.qElectricalSpeed;
    } else {
        foc_scalar_t delta = foc_mul_wide(
            ptImpl->qAcceleration < FOC_ZERO ?
            foc_sub_sat(FOC_ZERO, ptImpl->qAcceleration) :
            ptImpl->qAcceleration, ptImpl->qHighFrequencyPeriod);
        foc_scalar_t target = ptImpl->qOpenLoopSpeed;
        if (speed < target) {
            speed = foc_add_sat(speed, delta);
            if (speed > target) speed = target;
        } else if (speed > target) {
            speed = foc_sub_sat(speed, delta);
            if (speed < target) speed = target;
        }
        angle = foc_angle_wrap(foc_angle_from_scalar(foc_add_sat(
            angle.qTurns, foc_mul_wide(speed,
                                      ptImpl->qHighFrequencyPeriod))));
    }
    eResult = foc_clarke(ptImpl->tCurrent.qIu,
                         ptImpl->tCurrent.qIv,
                         ptImpl->tCurrent.qIw,
                            &work.tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    eResult = foc_park(&work.tCurrentAlphaBeta,
                       angle,
                       &work.tCurrent);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    if (mode == MOTOR_CONTROL_VOLTAGE_OPEN_LOOP) {
        work.tVoltage = voltage_ref;
    } else {
        work.tVoltage.qD = foc_controller_Step(
            &work.tConfig.tIdController,
            current_ref.qD,
            work.tCurrent.qD);
        work.tVoltage.qQ = foc_controller_Step(
            &work.tConfig.tIqController,
            current_ref.qQ,
            work.tCurrent.qQ);
    }
    eResult = foc_ipark(&work.tVoltage,
                        angle,
                        &work.tVoltageAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    eResult = motor_control_modulate(&work);
    if (eResult != FOC_RESULT_OK) {
        goto fail;
    }
    bool hardware_failed = false;
    eResult = motor_control_commit_hf(ptImpl, &work, angle, speed, &output,
                                      &hardware_failed);
    if (hardware_failed) goto hardware_fail;
    if (eResult == FOC_RESULT_BUSY ||
        eResult == FOC_RESULT_INVALID_ARGUMENT) return eResult;
    return FOC_RESULT_OK;

fail:
    motor_control_end_step(ptImpl, true);
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_INVALID_COMMAND);
    return eResult;
hardware_fail:
    motor_EmergencyStop(ptMotor, MOTOR_FAULT_HARDWARE);
    return eResult;
}
