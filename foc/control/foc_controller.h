/*******************************************************************************
 * @file    foc_controller.h
 * @brief   Runtime-selectable scalar controller interface
 ******************************************************************************/

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <stdbool.h>

#include "foc_pid.h"

typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);
    void (*fnTrack)(void *pContext,
                    foc_scalar_t qOutput,
                    foc_scalar_t qReference,
                    foc_scalar_t qFeedback);
} foc_controller_if_t;

foc_controller_if_t foc_controller_FromPid(foc_pid_t *ptPid);
bool foc_controller_IsValid(const foc_controller_if_t *ptController);
bool foc_controller_CanTrack(const foc_controller_if_t *ptController);
void foc_controller_Reset(const foc_controller_if_t *ptController);
void foc_controller_Track(const foc_controller_if_t *ptController,
                          foc_scalar_t qOutput,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);
foc_scalar_t foc_controller_Step(const foc_controller_if_t *ptController,
                                 foc_scalar_t qReference,
                                 foc_scalar_t qFeedback);

#endif /* FOC_CONTROLLER_H */
