#ifndef HALCORDIC_H
#define HALCORDIC_H

#include "foc_numeric.h"

void hal_cordic_Init(void);
void hal_cordic_SinCos(foc_scalar_t qTurns, foc_scalar_t *pqSin, foc_scalar_t *pqCos);
foc_scalar_t hal_cordic_Atan2(foc_scalar_t qY, foc_scalar_t qX);

#endif /* HALCORDIC_H */
