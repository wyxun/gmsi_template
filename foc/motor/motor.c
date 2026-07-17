/*******************************************************************************
 * @file    motor.c
 * @brief   Multi-instance motor aggregation object
 ******************************************************************************/

#include "motor.h"

#include <stddef.h>
#include <string.h>

foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig)
{
    foc_result_t eResult;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptConfig->tParams.chPolePairs == 0U ||
        ptConfig->eTopology < SENSING_TOPOLOGY_1P ||
        ptConfig->eTopology > SENSING_TOPOLOGY_3P) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    eResult = foc_hal_Validate(&ptConfig->tHal);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }

    memset(ptMotor, 0, sizeof(*ptMotor));
    ptMotor->tParams = ptConfig->tParams;
    ptMotor->tHal = ptConfig->tHal;
    ptMotor->tControl.tConfig = ptConfig->tControl;
    ptMotor->tCurrent.eTopology = ptConfig->eTopology;
    ptMotor->tCurrent.tCalib.wOffsetU = 2048U;
    ptMotor->tCurrent.tCalib.wOffsetV = 2048U;
    ptMotor->tCurrent.tCalib.wOffsetW = 2048U;
    ptMotor->tRt.eRunState = MOTOR_STATE_IDLE;
    return FOC_RESULT_OK;
}

void motor_Reset(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    memset(&ptMotor->tRt, 0, sizeof(ptMotor->tRt));
    ptMotor->tRt.eRunState = MOTOR_STATE_IDLE;
}

foc_result_t motor_SetDuty(motor_handle_t *ptMotor,
                           q_type qDutyU,
                           q_type qDutyV,
                           q_type qDutyW)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tRt.wFaults != MOTOR_FAULT_NONE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return foc_hal_SetDuty(&ptMotor->tHal.tPwm, qDutyU, qDutyV, qDutyW);
}

foc_result_t motor_Enable(motor_handle_t *ptMotor, bool bEnable)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (bEnable && ptMotor->tRt.wFaults != MOTOR_FAULT_NONE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return foc_hal_Enable(&ptMotor->tHal.tPwm, bEnable);
}

foc_result_t motor_CalibrateCurrent(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return foc_hal_CurrentCalibrate(&ptMotor->tHal.tAdc,
                                    &ptMotor->tCurrent.tCalib);
}

foc_result_t motor_GetRawCurrent(motor_handle_t *ptMotor,
                                 uint32_t *pwRawU,
                                 uint32_t *pwRawV,
                                 uint32_t *pwRawW)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return foc_hal_CurrentGetRaw(&ptMotor->tHal.tAdc,
                                 pwRawU, pwRawV, pwRawW);
}

foc_result_t motor_SampleCurrent(motor_handle_t *ptMotor)
{
    foc_result_t eResult;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = foc_hal_CurrentReconstruct(&ptMotor->tHal.tAdc,
                                         &ptMotor->tCurrent);
    if (eResult != FOC_RESULT_OK) {
        motor_EmergencyStop(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
    }
    return eResult;
}

void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_t eFault)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->tRt.wFaults |= (uint32_t)eFault;
    ptMotor->tRt.eRunState = MOTOR_STATE_FAULT;
    foc_hal_EmergencyStop(&ptMotor->tHal.tPwm);
}

void motor_AttachSensor(motor_handle_t *ptMotor,
                        sensor_interface_t *ptSensor)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptSensor = ptSensor;
    ptMotor->ptObserver = NULL;
}

void motor_AttachObserver(motor_handle_t *ptMotor,
                          observer_interface_t *ptObserver)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptObserver = ptObserver;
    ptMotor->ptSensor = NULL;
}
