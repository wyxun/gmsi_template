.section .text, "ax"
.global HardFault_Handler
.align 2
HardFault_Handler:
    /* 1. 自动在栈上分配 124 字节以存储通用寄存器快照 x1-x31 */
    addi sp, sp, -124
    sw x1, 0(sp)
    sw x3, 4(sp)
    sw x4, 8(sp)
    sw x5, 12(sp)
    sw x6, 16(sp)
    sw x7, 20(sp)
    sw x8, 24(sp)
    sw x9, 28(sp)
    sw x10, 32(sp)
    sw x11, 36(sp)
    sw x12, 40(sp)
    sw x13, 44(sp)
    sw x14, 48(sp)
    sw x15, 52(sp)
    sw x16, 56(sp)
    sw x17, 60(sp)
    sw x18, 64(sp)
    sw x19, 68(sp)
    sw x20, 72(sp)
    sw x21, 76(sp)
    sw x22, 80(sp)
    sw x23, 84(sp)
    sw x24, 88(sp)
    sw x25, 92(sp)
    sw x26, 96(sp)
    sw x27, 100(sp)
    sw x28, 104(sp)
    sw x29, 108(sp)
    sw x30, 112(sp)
    sw x31, 116(sp)

    /* 2. 原子提取 RISC-V 异常状态寄存器 (CSR) */
    csrr t0, mcause
    csrr t1, mepc
    csrr t2, mtval

    /* 3. 传递核心现场参数 (a0=sp, a1=mepc, a2=mcause, a3=mtval) 并调用诊断 */
    mv a0, sp
    mv a1, t1
    mv a2, t0
    mv a3, t2
    call mdebug_riscv_DumpException

_fault_hang:
    j _fault_hang
