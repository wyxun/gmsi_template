/*******************************************************************************
 * @file    foc_observer.c
 * @brief   Checked observer interface dispatch
 ******************************************************************************/

#include "foc_observer.h"

#include <stddef.h>

bool foc_observer_IsValid(const foc_observer_if_t *ptObserver)
{
    return ptObserver != NULL && ptObserver->pContext != NULL &&
           ptObserver->fnStep != NULL;
}

void foc_observer_Reset(const foc_observer_if_t *ptObserver)
{
    if (foc_observer_IsValid(ptObserver) && ptObserver->fnReset != NULL) {
        ptObserver->fnReset(ptObserver->pContext);
    }
}

foc_result_t foc_observer_Step(const foc_observer_if_t *ptObserver,
                               const foc_observer_input_t *ptInput,
                               foc_observer_output_t *ptOutput)
{
    if (!foc_observer_IsValid(ptObserver) || ptInput == NULL ||
        ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }
    return ptObserver->fnStep(ptObserver->pContext, ptInput, ptOutput);
}
