/*******************************************************************************
 * @file    foc_app.h
 * @brief   FOC 应用层接口 — 挂载在 GMSI 调度框架下
 ******************************************************************************/

#ifndef __FOC_APP_H__
#define __FOC_APP_H__

#include "gmsi.h"
#include "userconfig.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"
#include "motor_types.h"

typedef struct {
    motor_handle_t *ptMotor;
} foc_app_cfg_t;

typedef struct foc_app_s {
    gmsi_base_t    *ptBase;
    uint8_t         chState;
    int64_t         lLastHeartbeat;
    q_type          qSpeedRef;
    motor_handle_t *ptMotor;
} foc_app_t;

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);
fsm_rt_t foc_app_RunFSM(foc_app_t *ptThis);
void foc_app_SetSpeedRef(foc_app_t *ptThis, q_type qRef);
void foc_app_Start(foc_app_t *ptThis);
void foc_app_Stop(foc_app_t *ptThis);

extern void phase_testA(void);
extern void phase_testB(motor_handle_t *ptMotor);
extern void phase_testC(struct foc_app_s *ptApp);
extern void phase_test_waveform_init(void);
extern void phase_test_waveform_step(void);

#endif /* __FOC_APP_H__ */
