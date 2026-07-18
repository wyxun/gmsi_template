/*******************************************************************************
 * @file    motor.h
 * @brief   电机对象操作接口
 ******************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_types.h"

foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig);
void motor_Reset(motor_handle_t *ptMotor);
foc_result_t motor_GetRawCurrent(motor_handle_t *ptMotor,
                                 uint32_t *pwRawU,
                                 uint32_t *pwRawV,
                                 uint32_t *pwRawW);
void motor_EmergencyStop(motor_handle_t *ptMotor, motor_fault_e eFault);
void motor_SetVoltageReference(motor_handle_t *, foc_scalar_t, foc_scalar_t);
void motor_SetCurrentReference(motor_handle_t *, foc_scalar_t, foc_scalar_t);
void motor_SetSpeedReference(motor_handle_t *, foc_scalar_t);
void motor_SetPositionReference(motor_handle_t *, foc_scalar_t);
foc_result_t motor_HighFrequencyStep(motor_handle_t *);
foc_result_t motor_LowFrequencyStep(motor_handle_t *);
foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot);
foc_result_t motor_Start(motor_handle_t *ptMotor,
                         const motor_run_config_t *ptRunConfig);
foc_result_t motor_Stop(motor_handle_t *ptMotor);
fsm_rt_t motor_RunFSM(motor_handle_t *ptMotor);
foc_result_t motor_ClearFault(motor_handle_t *ptMotor);
#if defined(MOTOR_ENABLE_TEST_HOOKS)
size_t motor_TestGetImplementationSize(void);
void motor_TestSetOpenLoopCommandSpeed(motor_handle_t *ptMotor,
                                      foc_scalar_t qSpeed);
void motor_test_CorruptFSM(motor_handle_t *ptMotor,
                           motor_state_e eState,
                           motor_startup_phase_e ePhase);
bool motor_test_PositionBindingsValid(const motor_handle_t *ptMotor);
#endif

#endif /* __MOTOR_H__ */
