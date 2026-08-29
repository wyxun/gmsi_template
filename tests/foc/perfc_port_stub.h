#ifndef PERFC_PORT_STUB_H
#define PERFC_PORT_STUB_H

/* Host 测试用 perf_counter 移植桩：中断守卫为无操作。
 * 通过 -D__PERFC_CFG_PORTING_INCLUDE__="perfc_port_stub.h" 引用。 */

#include <stdint.h>

typedef uint32_t perfc_global_interrupt_status_t;

static inline perfc_global_interrupt_status_t
perfc_port_disable_global_interrupt(void)
{
    return 0U;
}

static inline void perfc_port_resume_global_interrupt(
    perfc_global_interrupt_status_t tStatus)
{
    (void)tStatus;
}

#endif /* PERFC_PORT_STUB_H */
