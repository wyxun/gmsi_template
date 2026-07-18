#include "motor.h"

uint32_t compile_fail_motor_member_access(motor_handle_t *ptMotor)
{
    return ptMotor->tRt.wFaults;
}
