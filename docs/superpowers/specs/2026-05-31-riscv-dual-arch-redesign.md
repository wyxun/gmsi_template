# Modus v0.5.0.x 双架构升级与高内聚重构设计方案 (Spec)

## 📌 大局宏观定位与设计宗旨

本升级的核心目标是：**在 `modus` 核心库中增加对通用 RISC-V 内核的系统级架构支持，使其演进为支持 ARM (Cortex-M) 与 RISC-V 双架构的嵌入式高硬核框架**。

在这个大局目标下，我们始终坚守以下设计哲学：
1.  **核心归核心（高度通用，零芯片依赖）**：`modus` 核心库对 Cortex-M 与 RISC-V 内核的兼容必须是纯架构级、指令集级的（例如仅依赖标准的 RISC-V CSR 寄存器如 `mcause`, `mstatus` 等），绝不掺杂任何特定芯片的外设代码。
2.  **外设归芯片（完全对称，无污染继承）**：具体的芯片（如首发 Demo 沁恒 `CH592`）仅作为 **功能验证与测试的典型板载载体**。它必须以最纯净的、无侵入包含官方 SDK 的方式融入，且多芯片目录结构 100% 保持完美的物理对称，为未来扩展其他芯片树立业界标杆。

---

## 📂 架构重构蓝图

```mermaid
graph TD
    subgraph Modus 主框架 (v0.5.0.x)
        A[global_define.h 双架构开关中断]
        B[perfc_port.c 双架构统一移植源文件]
        C[mdebug_riscv.c 通用RISC-V故障诊断]
    end

    subgraph Modus Template 模板仓库 (CH592 验证载体)
        D[peripheral/ch592/port_sys.c 底层初始化/时钟]
        E[peripheral/ch592/port_mdi.c 外设池静态注册]
        F[target/ch592/ch592_exception.S 强符号Fault覆盖拦截]
        G[vendor/riscv/ch592 纯官方SDK Submodule]
    end

    B -- 区分 __riscv 与 __ARM_ARCH 宏 --> B
    D -- 弱符号 get_system_core_clock_hz --> B
    G -- target.mk 直接引用官方汇编 --> Startup[官方启动汇编 startup_CH592.S]
    F -- 覆盖官方 HardFault_Handler 弱符号 --> Startup
    F -- 传递sp/mepc/mcause/mtval --> C
```

---

## 📝 详细重构设计方案

### 1. `perf_counter` 底层移植极简整合（只用一套文件，内部宏确认）
为了追求极致的高内聚，我们在 `modus` 核心库中**仅使用一个通用的 `perfc_port.c`** 文件。
*   **`perfc_port.c` (主库统一源文件)**：
    *   内部使用 `#if defined(__riscv)` 宏判定，启用标准 64位 CSR 硬件周期数读取（通过内联汇编获取 `cycle` 与 `cycleh`）及协程 SP 栈指针读写；
    *   使用 `#elif defined(__ARM_ARCH) || defined(__CORTEX_M)` 宏判定，启用 Cortex-M 通用的 SysTick 计数和 DWT 寄存器适配；
    *   两套分架构底层逻辑并存，使得 `modus.mk` 编译链只需维护唯一的 `perfc_port.c`。
*   **全局时钟解耦**：
    `perfc_port.c` 在获取系统频率时，仅依赖全局统一的弱符号 (Weak symbol) 函数：
    ```c
    __attribute__((weak)) uint32_t get_system_core_clock_hz(void) {
        return 60000000UL; // 弱符号兜底值，由芯片侧 port_sys.c 强符号定义覆盖
    }
    ```
    这彻底解耦了底层移植与特定芯片，在具体的芯片 target 层实现 **零冗余、零胶水**。

### 2. 芯片启动文件（startup）的无污染继承与强符号覆盖
*   **官方启动文件直接继承**：
    在 `target/ch592/target.mk` 中，将汇编编译变量 `STARTUP_S` 直接指向沁恒官方 SDK 的原始启动文件：
    ```make
    STARTUP_S = vendor/riscv/ch592/EVT/EXAM/SRC/Startup/startup_CH592.S
    ```
    我们**不在 target 路径下重新写一个完整的汇编文件**，从而实现 100% 对官方源文件的无侵入继承与重用。
*   **高性能异常强符号覆盖拦截**：
    为了在不污染官方文件的同时，让 `modus` 的异常诊断捕获（Fault Dump）起作用，我们在 `target/ch592/` 下只编写一个极其精炼的汇编文件 **`ch592_exception.S`**。
    在该文件中，我们定义一个**强符号**的 `HardFault_Handler`：
    ```assembly
    .section .text, "ax"
    .global HardFault_Handler
    .align 2
HardFault_Handler:
    /* 1. 保存所有通用寄存器 x1-x31 的快照到栈上 (124 字节) */
    addi sp, sp, -124
    sw x1, 0(sp)
    sw x3, 4(sp)
    ...
    sw x31, 116(sp)

    /* 2. 读取 exception 控制寄存器 */
    csrr t0, mcause
    csrr t1, mepc
    csrr t2, mtval

    /* 3. 传递 sp, mepc, mcause, mtval，调用 modus 库的 C 语言诊断函数 */
    mv a0, sp
    mv a1, t1
    mv a2, t0
    mv a3, t2
    call mdebug_riscv_DumpException

    /* 4. 挂起等待调试器挂载 */
_fault_hang:
    j _fault_hang
    ```
    **原理解析**：由于官方启动汇编里的 `HardFault_Handler` 被声明为了 `.weak` 弱符号，链接器在链接阶段会自动将控制流重定向到我们编写的强符号 `HardFault_Handler`，从而优雅、零侵入地实现了最强大的芯片硬件故障解码！

### 3. `modus_template` 验证芯片目录 100% 对称规整
为了与 STM32 架构完美对称，彻底移除 target 下的 system/peripheral 零散代码，规整如下：
*   **`peripheral/ch592/port_sys.c` (芯片底层适配)**：
    *   承载 CH592 官方的最低层系统时钟初始化（在 `SystemInit()` 中配置硬件 SysTick 与 CMP 比较寄存器）；
    *   定义全局强符号 `uint32_t get_system_core_clock_hz(void)` 回传芯片实际运行频率（如 60MHz），覆盖 `perf_counter` 的弱符号；
    *   实现外设引脚配置与底层串口波特率配置的 `peripheral_Init()` 物理入口。
*   **`peripheral/ch592/port_mdi.c` (外设池适配层)**：
    *   静态对接框架全局外设池 `HW`，注册 GPIO（LED）和 Stream（串口0）。
*   **`target/ch592/` (极致净化)**：
    *   **彻底删除**：`perfc_port_user.c`/`.h`、`system_ch592.c`、`peripheral_ch592.c`/`.h`，以及冗余的手写 `startup_CH592.S`。
    *   **仅保留**：`CH592_FLASH.ld`（链接脚本）、`ch592_exception.S`（强符号拦截）与 `ch592_it.c`（中断向量通知接口）。

### 4. Mstudio 与 aitrace 智能调试界面的双架构协议兼容与自适应渲染

为了让工程师使用的上位机 **Mstudio**（以及其集成的 **`aitrace` 智能调试引擎**）完美、无感地兼容 RISC-V 架构，我们在 `modus` 协议层和控制台上做出了如下自适应兼容设计：

*   **调试命令完全对齐 (API Alignment)**：
    *   `modus` 核心库提供的所有 RTT 调试指令如 `regs`、`peek`、`poke`、`stack`，在 Cortex-M 与 RISC-V 两套底层均实现了**平齐的接口协议**。
    *   Cortex-M 底层通过读取 DWT 寄存器及内核状态回传；RISC-V 底层通过读取标准 CSR 及保存栈帧回传。
*   **调试面板与 CPU 架构自适应匹配 (Signature Auto-detection)**：
    *   为了使 Mstudio 的调试界面能够智能切换寄存器渲染视图（Cortex-M 展现 r0-r15，RISC-V 展现 x0-x31 通用寄存器），我们在调试指令的输出头部添加了**核心架构签名**：
        *   **Cortex-M 签名**：输出以 `=== Cortex-M Core Registers ===` 开头；
        *   **RISC-V 签名**：输出以 `=== RISC-V Core Registers ===` 开头。
    *   Mstudio 调试控制台解析该签名后，即可 100% 自动切换渲染模板，无需任何工程师手动配制。
*   **`aitrace` 异常智能分发 (aitrace Fault Dispatch)**：
    *   在发生硬件故障（Fault）时，RTT 自动向 aitrace 调试信道抛出带有架构特征的致命异常签名：
        *   Cortex-M：输出 `!! HARDWARE FAULT !!`；
        *   RISC-V：输出 `!! RISC-V HARDWARE FAULT !!`。
    *   `aitrace` 解析到对应签名后，自动进入其特有的分发解码器：
        *   Cortex-M 解析器 -> 智能分析 `CFSR`、`HFSR` 寄存器状态并绘制异常栈；
        *   RISC-V 解析器 -> 智能分析 `MCAUSE` 异常码、`MEPC` 程序飞出点与 `MTVAL` 非法内存地址，精细转储 x0-x31。
    *   这打通了上位机与下位机的双架构调试闭环，对工程师体验完全平滑一致！

---

## 📂 变更清单 (Proposed Changes)

### [Component 1: Modus Framework (e:\Project\modus)]

*   **`[NEW]` [perfc_port.c](file:///e:/Project/modus/src/mdebug/perfc_port.c)**：统一的双架构移植源文件，内部通过宏处理 Cortex-M 与 RISC-V 逻辑。
*   **`[MODIFY]` [modus.mk](file:///e:/Project/modus/modus.mk)**：将 `perfc_port.c` 注册入框架源编译清单，并在编译参数中废除各芯片自定义 `perfc_port_user.h` 头文件的依赖。

### [Component 2: Modus Template (e:\Project\modus_template)]

*   **`[DELETE]` [所有已过时的 perfc_port_user.c 与 .h 胶水文件]**：
    *   `target/stm32g431/perfc_port_user.c` / `perfc_port_user.h`
    *   `target/at32f421/perfc_port_user.c` / `perfc_port_user.h`
    *   `target/at32f413/perfc_port_user.c` / `perfc_port_user.h`
    *   以及 `target/ch592/` 下的原有手写驱动文件。
*   **`[NEW]` [peripheral/ch592/port_sys.c](file:///e:/Project/modus_template/peripheral/ch592/port_sys.c)**：沁恒官方外设初始化、SysTick配置与系统时钟强符号回传。
*   **`[NEW]` [target/ch592/ch592_exception.S](file:///e:/Project/modus_template/target/ch592/ch592_exception.S)**：以强符号对官方弱符号异常处理实现零侵入拦截与 dump。
*   **`[MODIFY]` [target/ch592/target.mk](file:///e:/Project/modus_template/target/ch592/target.mk)**：
    *   `CHIPLIB_ROOT` 归口指向官方 EVT 核心路径 `vendor/riscv/ch592/EVT/EXAM/SRC`；
    *   `STARTUP_S` 直接重定向至官方 SDK 原始启动汇编 `$(CHIPLIB_ROOT)/Startup/startup_CH592.S`；
    *   链接源文件追加 `target/ch592/ch592_exception.S`，包含路径加入 `-I$(CHIPLIB_ROOT)/RVMSIS` 和 `-I$(CHIPLIB_ROOT)/StdPeriphDriver/inc`。
*   **`[MODIFY]` [makefile](file:///e:/Project/modus_template/makefile)**：
    *   支持小写 `target` 参数映射为 `TARGET_CHIP`。
