# GMSI Bare-Metal Template
 
 本工程是一个面向 ARM Cortex-M（以 AT32F421 为例）的纯裸机开发模板。
 
 此外，本模板采用 **Git Submodule (子模块)** 机制来统一管理第三方依赖（包括 CMSIS 核心库、厂家芯片库代码、以及 GMSI 本身）。这也是目前业界的主流演进趋势：**后续所有原厂固件库和第三方组件都直接通过 Git 仓库的形式被项目引用和同步更新**，告别手工复制粘贴代码。
 
 ---
 
 ## 1. 目录组织架构 (Directory Structure)
 
 整个工程的目录被严格划分为以下几个层次，以保证架构的清晰：
 
 - **`gmsi/`**：GMSI 核心框架模块。包含了 GDI 接口定义、底层协程、队列、以及内嵌的 `plooc`（面向对象 C 宏）、`perf_counter`（性能测量库）和 `Segger RTT`（日志输出库）。
 - **`chiplib/`**：提供芯片启动文件及厂商底层驱动库。
   - 为了便于同步原厂的漏洞热修复和更新，这里通过 Git Submodule 形式引入了官方的 **CMSIS** 核心架构依赖和 **AT32F421_Firmware_Library** (官方固件库)。
 - **`peripheral/`**：**底层硬件适配层**。
   - 这部分允许包含特定硬件的初始化代码（`hal*.c/h`）。
   - **核心文件 `gdi_peripheral.c/h`**：GDI 移植层。工程师在这里负责将杂乱的厂商库函数，统一封装为标准的 GDI `gdi_gpio_t`、`gdi_adc_t`、`gdi_pwm_t` 等对象，并向外暴露全局级别的硬件资源池实例 `HW`。
 - **`src/` & `class/`**：**纯上层应用业务层**。
   - 包含主入口 `main.c` 以及各种高阶的抽象控制类。
   - **开发禁令**：在这个层级**绝对禁止**包含诸如 `at32f421.h` 这类厂商专有库，也**禁止**直接调用 `gpio_bits_write` 这类函数。
   - 所有业务逻辑必须通过引入 `gdi_peripheral.h` 得到全局 `HW` 对象，随后完全通过 `GDI` 宏接口与底层交握。
 - **`build/`**：LLVM 工具链构建输出目录，生成最终的固件 (`.elf`, `.bin`, `.hex`)。
 
 ---
 
 ## 2. 命名规范与风格 (Naming Conventions)
 
 为了保持代码库的高度一致性和可读性，本项目强制推行以下 C 语言命名规范（主要影响全局/局部变量及结构体成员）：
 
 ### 变量前缀风格 (匈牙利命名法变体)
 
 所有变量必须添加类型前缀，以小写字母开头，后接大驼峰（CamelCase），明确指示变量的基础类型或性质：
 
 | 前缀 | 代表类型 | 示例 | 说明 |
 |------|---------|------|------|
 | `ch` | `char` 或 `uint8_t` | `chState` | 单字节字符、状态字或微小计数器 |
 | `hw` | `uint16_t` (Half-Word) | `hwBufferSize` | 16 位整数 |
 | `w`  | `uint32_t` (Word) | `wEvent` | 32 位无符号整数 |
 | `n`  | `int32_t` / `int` | `nResult` | 32 位有符号整数，通常用于带负数的运算或错误码 |
 | `l`  | `int64_t` (Long) | `lLastHeartbeat`| 64 位宽整数，如系统时间戳 |
 | `f`  | `float` | `fAngle` | 单精度浮点数 |
 | `d`  | `double` | `dTarget` | 双精度浮点数 |
 | `b`  | `bool` | `bIsRunning` | 布尔值标志 |
 | `p`  | Pointer (一般指针) | `pBuffer` | 一般类型的指针（基准前缀） |
 | `pch`| `char *` / `uint8_t *` | `pchData` | 指向单字节/字符串的指针 |
 | `phw`| `uint16_t *` | `phwBuffer` | 指向 16 位数据的指针 |
 | `pw` | `uint32_t *` | `pwAdcRaw`| 指向 32 位数据的指针 |
 | `pq` | `q_type *` | `pqResult` | 指向定/浮点数的指针 |
 | `pt` | Pointer to Type | `ptThis`, `ptMotor`| 指向特定 `struct` 或 `typedef` 的对象指针 |
 | `pfcn`| Function Pointer | `pfcnSetDuty` | 函数指针（较 `fn` 更强调指针属性，或直接用 `fn`） |
 | `e`  | Enum | `eRunState` | 枚举类型的变量 |
 | `q`  | `q_type` (本项目特有) | `qSpeedRef` | FOC 数学库的多态定/浮点数 |
 
 ### 修饰前缀
 - `s_`：静态变量 (Static)，只在文件内部可见，必须与类型前缀组合使用（例：`s_lTimestamp`, `s_tFocAppBase`）。
 - `g_`：全局变量 (Global)，必须与类型前缀组合使用（例：`g_chSystemData`）。
 - `t` / `_t`：结尾表示 Type（如 `motor_handle_t`），作为变量名前缀表示这是一个实例化对象本身而非指针（特例，视习惯而定，如 `tGmsi`）。
 
 ### 函数风格
 - **模块/类前缀**：大写或首字母大写下划线，如 `motor_Init()` 或 `foc_app_Start()`。
 - **私有函数**：文件内 `static` 函数，正常小写下划线或前缀名加内部动作。
 
 ---
 
 ## 3. 如何使用本模板进行开发 (How to Use)
 
 基于本模板开发应用程序，你只需要遵循“**配置外设 -> 包装 GDI -> 纯应用态编写**”的三步走开发规范：
 
 ### 步骤 1：底层硬件实例化与适配 (GDI Porting)
 1. 初始化你的硬件外设（可以由 AT32 Work Bench 生成的基础代码转移到 `peripheral/hal*.c`，或者手写外设初始化）。
 2. 打开 **`peripheral/gdi_peripheral.h`**，在 `gdi_hardware_t` 结构体中为你的全新硬件设施声明标准 GDI 接口。例如增加一个继电器：
    ```c
    typedef struct {
        // ... 其他已有外设 ...
        gdi_gpio_t *ptRelay1;    // 新增一个继电器声明
    } gdi_hardware_t;
    ```
 3. 打开 **`peripheral/gdi_peripheral.c`**，编写厂商底层库的包裹层，并将其实例化挂载到 `HW` 全局资源池中：
    ```c
    // 1. 编写对雅特力固件库的包裹包装
    static int32_t relay_Set(void *pPriv, gdi_gpio_level_t eLevel) {
        gpio_bits_write(GPIOA, GPIO_PINS_1, (eLevel == GDI_GPIO_HIGH) ? TRUE : FALSE);
        return 0;
    }
    
    // 2. 根据包装好的函数，实例化 GDI_GPIO 对象
    static gdi_gpio_t s_tRelay1 = { .fnSet = relay_Set };
    
    // 3. 在最下方的 const gdi_hardware_t HW 池中对外暴露：
    const gdi_hardware_t HW = {
        // ... 其他外设挂载 ...
        .ptRelay1 = &s_tRelay1,
    };
    ```
 
 ### 步骤 2：上层应用开发 (Application Logic)
 1. 在你的业务逻辑文件（如 `src/main.c` 或自定义的电机控制模块 `class`）中，**只需要 `#include "gdi_peripheral.h"`**，不要引入任何 HAL 库头文件。
 2. 使用完全正交统管的多态宏 `GDI_Write`、`GDI_Read`、`GDI_Toggle` 操纵硬件：
    ```c
    // 打开继电器（应用层工程师完全感知不到底层是 AT32 还是 NXP，它挂在哪个引脚）
    GDI_Write(HW.ptRelay1, GDI_GPIO_HIGH); 
    
    // 读取电机某一相的 ADC 模数转换值
    uint16_t adcVal = (uint16_t)GDI_Read(HW.ptMotorAdcU);
    ```

 3. **状态机开发规范**：本工程所有模块的状态机统一采用 **`perf_counter` 库的 PT（Proto-Thread）状态机**框架，禁止使用裸 `switch-case` 状态机。

    **结构体**：对象控制块中必须包含一个 `uint8_t chState` 字段：
    ```c
    typedef struct {
        uint8_t  chState;   /* PT 状态机状态字，必须为第一个或独立字段 */
        /* ... 其他成员 ... */
    } my_module_t;
    ```

    **函数框架**：
    ```c
    #undef  this
    #define this (*ptThis)   /* 启用 this 语法糖 */

    fsm_rt_t my_module_Run(my_module_t *ptThis)
    {
    PERFC_PT_BEGIN(this.chState)

        /* 步骤 1：等待某个条件 */
        PERFC_PT_WAIT_UNTIL(some_condition_is_true())

        /* 步骤 2：执行动作后挂起，下次从这里继续 */
        PERFC_PT_ENTRY(
            do_something();
        )
        PERFC_PT_YIELD(fsm_rt_on_going);

        /* 步骤 3：延时 500ms */
        PERFC_PT_DELAY_MS(500)

    PERFC_PT_END()
        return fsm_rt_cpl;
    }
    ```

    | 宏 | 作用 |
    |----|------|
    | `PERFC_PT_BEGIN(chState)` | 状态机入口，恢复上次挂起点 |
    | `PERFC_PT_ENTRY(...)` | 定义一个挂起/恢复锚点，括号内为进入时执行一次的初始化代码 |
    | `PERFC_PT_YIELD(val)` | 挂起并返回 `val`，下次从此处继续 |
    | `PERFC_PT_WAIT_UNTIL(cond)` | 轮询直到 `cond` 为真时才继续 |
    | `PERFC_PT_DELAY_MS(ms)` | 非阻塞延时 `ms` 毫秒 |
    | `PERFC_PT_RETURN(val)` | 提前结束状态机并返回 `val` |
    | `PERFC_PT_END()` | 状态机结束，重置 `chState=0` |

    > 参考实现：`foc/app/foc_app.c::foc_app_Run()`，完整宏定义见 `gmsi/lib/perf_counter/perfc_task_pt.h`。
 
 ### 步骤 3：项目依赖与功能伸缩调整 (Makefile Toggling)
 工程采用了灵活自由的 Makefile 管理。如果你在开发中启用了诸如 I2C 或 SPI 这种原本关闭的内置外设：
 - 请在根目录下的 **`makefile`** 中，搜寻并把对应的 `USE_DRV_XXX` 标志置为 `1`。
 - Makefile 系统会自动引入对应厂家的源文件参与最终编译。
 ```makefile
 USE_DRV_SPI    ?= 1
 USE_DRV_I2C    ?= 1
 ```
 
 ---
 
 ## 4. 编译、烧录与手动调试 (Compilation, Flashing & Debugging)
 
 本工程默认使用 **LLVM 交叉编译工具链**进行高优化构建，并通过 **OpenOCD 配合 CMSIS-DAP DAP-Link** 仿真器进行烧录和基于 **SEGGER RTT** 的实时极速日志输出。
 
 ### 1. 编译并烧录
 每次修改源码保存后，在开发终端下执行：
 ```powershell
 make clean; make flash
 ```
 
 ### 2. 挂载 RTT 调试通道
 固件烧录完毕并开始运作时，可以利用下述命令拉起一个隐藏形态的服务进程（释放本地 `9090` 端口）：
 ```powershell
 Start-Process powershell -ArgumentList "-WindowStyle", "Minimized", "-Command", "make rtt"
 ```
 
 ### 3. 日志捕获监控
 为防止实时跳动的庞大日志刷屏遮挡你的正常 Makefile 操作屏，推荐采用独立监视器脚本：
 ```powershell
 .\.agent\workflows\rtt_viewer.ps1
 ```
 此时将弹出一个绿色字体的独立监控视窗，所有 `LOG_OUT()` 等重定向底层日志将倾泻于此。
 
 ---
 
 ## 5. Git 仓库与子模块管理 (Git Submodules Configuration)
 
 正如开篇所述，为了实现优雅的底层基础设施隔离更新（包括 CMSIS、原厂芯片库及 GMSI 框架），本工程核心依赖 `git submodule`。
 
 **首次克隆仓库（全家桶式下载）**：
 务必带上 `--recursive` 参数，这一步动作会自动帮你连同 `chiplib` (CMSIS / HAL) 和 `gmsi` 等所有被挂载的嵌套子代码库一同获取：
 ```bash
 git clone --recursive https://your-repo-url/template-project.git
 ```
 
 **“我忘了带参数”的自救指令**：
 如果克隆时忘记加 `--recursive` 参数，导致 `gmsi` 等文件夹徒有其表却空无一物，请立刻在工程根目录执行：
 ```bash
 git submodule update --init --recursive
 ```
 
 **更新特定嵌套核心库（厂商库或底层组件库版本迭代）**：
 假设官方开源组把 `gmsi` 仓库切去了新一代的 `dev` 分支，你想令这个工程同步跟进这一项基础核心底座升级：
 ```bash
 cd gmsi
 git checkout dev 
 git pull origin dev
 git submodule update --init --recursive
 cd ..
 git add gmsi
 git commit -m "chore: bump gmsi framework to latest dev branch"
 ```
