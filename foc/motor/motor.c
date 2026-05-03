/*******************************************************************************
 * @file    motor.c
 * @brief   电机对象操作实现
 ******************************************************************************/

#include "motor.h"
#include <string.h>

int motor_Init(motor_handle_t *ptMotor, motor_config_t *ptConfig)
{
    if (ptMotor == NULL) { return -1; }

    memset(ptMotor, 0, sizeof(motor_handle_t));

    if (ptConfig != NULL) {
        ptMotor->tParams = ptConfig->tParams;
        ptMotor->tPwm = ptConfig->tPwm;
        ptMotor->tCurrent.tOps      = ptConfig->tAdc;
        ptMotor->tCurrent.eTopology = ptConfig->eTopology;
        ptMotor->tCurrent.tCalib.wOffsetU      = 2048U;
        ptMotor->tCurrent.tCalib.wOffsetV      = 2048U;
        ptMotor->tCurrent.tCalib.wOffsetW      = 2048U;
        ptMotor->tCurrent.tCalib.bIsCalibrated = false;
    }

    ptMotor->tRt.eRunState = MOTOR_STATE_IDLE;

    return 0;
}

void motor_Reset(motor_handle_t *ptMotor)
{
    if (ptMotor == NULL) { return; }
    memset(&ptMotor->tRt, 0, sizeof(ptMotor->tRt));
    ptMotor->tRt.eRunState = MOTOR_STATE_IDLE;
}

void motor_AttachSensor(motor_handle_t *ptMotor, sensor_interface_t *ptSensor)
{
    if (ptMotor == NULL) { return; }
    ptMotor->ptSensor   = ptSensor;
    ptMotor->ptObserver = NULL;
}

void motor_AttachObserver(motor_handle_t *ptMotor, observer_interface_t *ptObserver)
{
    if (ptMotor == NULL) { return; }
    ptMotor->ptObserver = ptObserver;
    ptMotor->ptSensor   = NULL;
}
