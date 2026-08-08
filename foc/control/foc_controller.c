/*******************************************************************************
 * @file    foc_controller.c
 * @brief   Controller adapters used only where product modes require selection
 ******************************************************************************/

#include "foc_controller.h"

#include <stddef.h>

static void controller_pid_reset(void *pController)
{
    foc_pid_Reset((foc_pid_t *)pController);
}

static foc_scalar_t controller_pid_step(void *pController,
                                        foc_scalar_t qReference,
                                        foc_scalar_t qFeedback)
{
    return foc_pid_Step((foc_pid_t *)pController, qReference, qFeedback);
}

static void controller_pid_track(void *pController,
                                 foc_scalar_t qOutput,
                                 foc_scalar_t qReference,
                                 foc_scalar_t qFeedback)
{
    foc_pid_Track((foc_pid_t *)pController, qOutput, qReference, qFeedback);
}

foc_controller_if_t foc_controller_FromPid(foc_pid_t *ptPid)
{
    foc_controller_if_t tController = {
        .pController = ptPid,
        .fnReset = controller_pid_reset,
        .fnStep = controller_pid_step,
        .fnTrack = controller_pid_track,
    };

    if (ptPid == NULL) {
        tController.fnReset = NULL;
        tController.fnStep = NULL;
        tController.fnTrack = NULL;
    }
    return tController;
}

bool foc_controller_IsValid(const foc_controller_if_t *ptController)
{
    return ptController != NULL && ptController->pController != NULL &&
           ptController->fnStep != NULL;
}

bool foc_controller_CanTrack(const foc_controller_if_t *ptController)
{
    return foc_controller_IsValid(ptController) &&
           ptController->fnTrack != NULL;
}

void foc_controller_Reset(const foc_controller_if_t *ptController)
{
    if (ptController != NULL && ptController->pController != NULL &&
        ptController->fnReset != NULL) {
        ptController->fnReset(ptController->pController);
    }
}

void foc_controller_Track(const foc_controller_if_t *ptController,
                          foc_scalar_t qOutput,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback)
{
    if (foc_controller_CanTrack(ptController)) {
        ptController->fnTrack(ptController->pController, qOutput,
                              qReference, qFeedback);
    }
}

foc_scalar_t foc_controller_Step(const foc_controller_if_t *ptController,
                                 foc_scalar_t qReference,
                                 foc_scalar_t qFeedback)
{
    if (!foc_controller_IsValid(ptController)) {
        return FOC_ZERO;
    }
    return ptController->fnStep(ptController->pController,
                                qReference, qFeedback);
}
