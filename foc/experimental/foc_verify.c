#include "foc_verify.h"
#include "foc_app.h"
#include "mlog.h"
#include "mshell.h"
#include <string.h>
#include <stdio.h>

#if FOC_ENABLE_MOTOR_VERIFY

foc_result_t foc_verify_StaticLock(motor_handle_t *ptMotor, float fTurns, float fVq)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }

    /* Strict voltage safety limit: clamp to 0.05 pu max */
    if (fVq > 0.05f) fVq = 0.05f;
    if (fVq < -0.05f) fVq = -0.05f;

    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
        .ptInitialPositionSource = NULL,
        .ptTargetPositionSource = NULL,
        .qInitialAngle = FOC_SCALAR(fTurns),
        .qOpenLoopSpeed = FOC_ZERO,
        .qAcceleration = FOC_ZERO,
        .tVoltageReference = {
            .qD = FOC_ZERO,
            .qQ = FOC_SCALAR(fVq),
        },
    };

    foc_result_t res = motor_Start(ptMotor, &run);
    if (res == FOC_RESULT_BUSY) {
        motor_Stop(ptMotor);
        res = motor_Start(ptMotor, &run);
    }
    return res;
}

foc_result_t foc_verify_OpenLoopRun(motor_handle_t *ptMotor, float fVoltageQ, float fSpeedHz)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }

    /* Strict voltage safety limit: clamp to 0.05 pu max */
    if (fVoltageQ > 0.05f) fVoltageQ = 0.05f;
    if (fVoltageQ < -0.05f) fVoltageQ = -0.05f;

    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_VOLTAGE_OPEN_LOOP,
        .ptInitialPositionSource = NULL,
        .ptTargetPositionSource = NULL,
        .qInitialAngle = FOC_ZERO,
        .qOpenLoopSpeed = FOC_SCALAR(fSpeedHz),
        .qAcceleration = FOC_SCALAR(10.0f),
        .tVoltageReference = {
            .qD = FOC_ZERO,
            .qQ = FOC_SCALAR(fVoltageQ),
        },
    };

    foc_result_t res = motor_Start(ptMotor, &run);
    if (res == FOC_RESULT_BUSY) {
        motor_Stop(ptMotor);
        res = motor_Start(ptMotor, &run);
    }
    return res;
}

foc_result_t foc_verify_CurrentLoopRun(motor_handle_t *ptMotor, float fIqRef, float fSpeedHz)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }

    /* Safety clamp for target Iq reference (max 0.10 pu) */
    if (fIqRef > 0.10f) fIqRef = 0.10f;
    if (fIqRef < -0.10f) fIqRef = -0.10f;

    motor_run_config_t run = {
        .eControlMode = MOTOR_CONTROL_CURRENT,
        .ptInitialPositionSource = NULL,
        .ptTargetPositionSource = NULL,
        .qInitialAngle = FOC_ZERO,
        .qOpenLoopSpeed = FOC_SCALAR(fSpeedHz),
        .qAcceleration = FOC_SCALAR(10.0f),
        .tCurrentReference = {
            .qD = FOC_ZERO,
            .qQ = FOC_SCALAR(fIqRef),
        },
    };

    foc_result_t res = motor_Start(ptMotor, &run);
    if (res == FOC_RESULT_BUSY) {
        motor_Stop(ptMotor);
        res = motor_Start(ptMotor, &run);
    }
    return res;
}

foc_result_t foc_verify_Stop(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return motor_Stop(ptMotor);
}

static void cmd_motor_verify(const char *args)
{
    extern foc_app_t tFocApp;
    if (tFocApp.ptMotor == NULL) {
        MLOG(E, "Motor instance not bound in FOC App\r\n");
        return;
    }

    if (strncmp(args, "static", 6) == 0) {
        float turns = 0.0f, vq = 0.05f;
        if (sscanf(args + 6, "%f %f", &turns, &vq) >= 1) {
            foc_result_t res = foc_verify_StaticLock(tFocApp.ptMotor, turns, vq);
            if (res == FOC_RESULT_OK) {
                MLOGF(I, "[Verify] Locked electrical angle: %.4f turns, Vq = %.4f pu\r\n", 
                      (double)turns, (double)vq);
            } else {
                MLOGF(E, "[Verify] Static lock rejected by FSM (code %d)\r\n", (int)res);
            }
        } else {
            MLOG(I, "Usage: motor_verify static <turns> [vq=0.05]\r\n");
        }
    }
    else if (strncmp(args, "run", 3) == 0) {
        float speed = 2.0f, vq = 0.05f;
        if (sscanf(args + 3, "%f %f", &speed, &vq) >= 1) {
            foc_result_t res = foc_verify_OpenLoopRun(tFocApp.ptMotor, vq, speed);
            if (res == FOC_RESULT_OK) {
                MLOGF(I, "[Verify] Open-loop running at %.2f Hz, Vq = %.4f pu\r\n", 
                      (double)speed, (double)vq);
            } else {
                MLOGF(E, "[Verify] Open-loop run rejected by FSM (code %d)\r\n", (int)res);
            }
        } else {
            MLOG(I, "Usage: motor_verify run <speed-hz> [vq=0.05]\r\n");
        }
    }
    else if (strncmp(args, "current", 7) == 0) {
        float iq = 0.015f, speed = 2.0f;
        if (sscanf(args + 7, "%f %f", &iq, &speed) >= 1) {
            foc_result_t res = foc_verify_CurrentLoopRun(tFocApp.ptMotor, iq, speed);
            if (res == FOC_RESULT_OK) {
                MLOGF(I, "[Verify] Current-loop closed: Iq = %.4f pu, Speed = %.2f Hz\r\n", 
                      (double)iq, (double)speed);
            } else {
                MLOGF(E, "[Verify] Current loop rejected by FSM (code %d)\r\n", (int)res);
            }
        } else {
            MLOG(I, "Usage: motor_verify current <iq-pu=0.015> [speed-hz=2.0]\r\n");
        }
    }
    else if (strncmp(args, "stop", 4) == 0) {
        foc_verify_Stop(tFocApp.ptMotor);
        MLOG(I, "[Verify] Motor stopped.\r\n");
    }
    else {
        MLOG(I, "Usage: motor_verify <static|run|stop>\r\n");
    }
}

MODUS_SHELL_CMD(motor_verify, cmd_motor_verify, "Motor verification and bring-up utilities");

#endif /* FOC_ENABLE_MOTOR_VERIFY */
