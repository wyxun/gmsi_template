#ifndef __CMSIS_COMPILER_H__
#define __CMSIS_COMPILER_H__

/* 
 * Blank cmsis_compiler.h for RISC-V bare-metal compilations.
 * This is a shadow header used to bypass the CMSIS compiler dependency inside perf_counter.c.
 * 
 * Crucial Design: This file resides ONLY in target/ch592/ and is included ONLY when 
 * compiling for the RISC-V target, fully protecting the Cortex-M system compiler headers.
 */

#ifndef __WEAK
#define __WEAK    __attribute__((weak))
#endif

#endif /* __CMSIS_COMPILER_H__ */
