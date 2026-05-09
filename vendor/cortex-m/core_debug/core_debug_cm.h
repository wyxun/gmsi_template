#ifndef __CORE_DEBUG_CM_H__
#define __CORE_DEBUG_CM_H__

#include <stdint.h>

/* Sentinel: when defined, IT files skip their stub fault handlers
 * because core_debug_cm_fault.c provides real ones. */
#define CORE_DEBUG_FAULT_HANDLERS_ACTIVE

/*============================================================================
 * Cortex-M core debug — shell-command registration entry
 *===========================================================================*/

/**
 * @brief Register all core_debug shell commands (regs, peek, poke, stack).
 *        Called automatically via MODUS_SHELL_CMD macro; no manual init needed.
 */

/*============================================================================
 * HardFault auto-dump entry — called from HardFault_Handler ISR
 *===========================================================================*/

/**
 * @brief Print stacked exception frame and fault status registers via RTT.
 *        This is the default HardFault handler installed by core_debug_cm_fault.c.
 *        Link your own handler by defining CORE_DEBUG_OVERRIDE_FAULT_HANDLER.
 */
void core_debug_DefaultFaultHandler(void);

#endif /* __CORE_DEBUG_CM_H__ */
