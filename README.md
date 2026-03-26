# GMSI Bare-Metal Template

本工程是一个面向 ARM Cortex-M（当前以 AT32F421 为例，支持多芯片无缝扩展）的纯裸机抽象开发模板。

此外，本模板采用 **Git Submodule (子模块)** 机制来统一管理第三方依赖（包括 CMSIS 核心库、厂家芯片库代码、以及 GMSI 本身）。这也是目前业界的主流演进趋势：**后续所有原厂固件库和第三方组件都直接通过 Git 仓库的形式被项目引用和同步更新**，告别手工复制粘贴代码。

---

## 1. 目录组织架构 (Directory Structure)
整个工程的目录被严格划分为以下几个层次，以保证架构的芯片解耦与高内聚：

- **`gmsi/`**：GMSI 核心框架模块。包含了 GDI（通用设备接口 Generic Device Interface）定义、底层协程、队列、以及内嵌的 `plooc`（面向对象 C 宏）、`perf_counter`（性能测量库）和 `Segger RTT`（日志输出库）。
- **`chiplib/`**：提供芯片启动文件及各厂商底层驱动库。
  - 为了便于同步原厂的漏洞热修复和更新，这里通过 Git Submodule 形式引入了官方的 **CMSIS** 核心架构依赖和对应芯片（如 **AT32F421_Firmware_Library** 等）的官方固件库。
- **`peripheral/`**：**底层硬件终极适配层**（*核心护城河*）。
  - 该目录及其子目录负责**隔离所有特定的芯片依赖**，上层业务代码不再直接与芯片固件库打交道。
  - **`peripheral.h` & `gdi_hw.h`** (顶层接口)：向应用层暴露跨平台的系统初始化入口 `peripheral_Init()`、时钟查询以及全局外设资源池 `HW` 实例。
  - **`peripheral/<芯片代号>/`** (如 `at32f421/`)：芯片的具体驱动包裹代码全在这里。
    - `port_sys.c`：处理时钟树初始化 (`SystemClock_Config`)、中断向量配置等系统级别底层动作。
    - `port_gdi.c`：将杂乱的厂商库 API 统一包装为 GDI 标准的 `gdi_gpio_t`、`gdi_pwm_t` 等多态对象，并最终实例化外部全局变量 `HW` 供上层使用。
- **`src/` & `class/`**：**纯上层应用业务层**。
  - 包含主入口 `main.c` 以及各种高阶的抽象控制逻辑。
  - **开发禁令**：在这个层级**绝对禁止**包含诸如 `at32f421.h`、`stm32xxx.h` 这类厂商专有库，也**禁止**直接调用 `gpio_bits_write` 这类特有底层函数！
  - 业务逻辑初始化只需调用 `peripheral_Init()`，外设交互完全通过引入 `gdi_hw.h` 得到全局 `HW` 对象后，利用统一的 `GDI_Write/Read` 宏接口与底层交握。
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

## 3. 跨平台框架如何开发？ (How to Develop)
基于本模板开发应用程序，你只需要遵循“**配置外设 -> 包装 GDI -> 纯应用态编写**”规范：

### 步骤 1：针对目标芯片实现底层细节 (GDI Porting)
1. 设计系统的共用对象：在 **`peripheral/gdi_hw.h`** 中的 `gdi_hardware_t` 里声明当前应用到底用到了哪些设备（它将作为抽象的“插座”使用）。
   ```c
   typedef struct {
       gdi_gpio_t *ptLedStatus;  // 状态灯
       gdi_gpio_t *ptRelay1;     // 新增继电器
   } gdi_hardware_t;
   ```
2. 进入特定芯片适配目录（例如 `peripheral/at32f421/`）：
   - 在 **`port_sys.c`** 里调用厂商库初始化好系统的核心频率（如 `HICK 48MHz`）及开放特定的 GPIO 口时钟等。
   - 在 **`port_gdi.c`** 里编写厂商库的包壳层（驱动实现）：
     ```c
     // 1. 实现对接特定厂家的控制逻辑
     static int32_t at32_relay_Set(void *pPriv, gdi_gpio_level_t eLevel) {
         gpio_bits_write(GPIOA, GPIO_PINS_1, (eLevel == GDI_GPIO_HIGH) ? TRUE : FALSE);
         return 0;
     }
     
     // 2. 将其关联实例化到静态的 GDI 对象
     static gdi_gpio_t s_tRelay1 = { .fnSet = at32_relay_Set };
     
     // 3. 在最后实例化对外暴露的硬件资源池 HW
     const gdi_hardware_t HW = {
         .ptLedStatus = &s_tLedGpio,
         .ptRelay1    = &s_tRelay1,
     };
     ```

### 步骤 2：上层应用业务逻辑 (Application Logic)
1. 在你的业务逻辑文件（如 `src/main.c` 或自定义应用 `class`）中，**只需要包含 `peripheral.h` 或 `gdi_hw.h`**。
2. 业务第一行无脑调用 `peripheral_Init()` 加载一切芯片底层。
3. 利用正交多态宏操控硬件：'GDI_Write'、'GDI_Read'、'GDI_Toggle'操作硬件：
   ```c
   // 操控继电器（应用层对引脚/厂家毫无感知，完全解耦）
   GDI_Write(HW.ptRelay1, GDI_GPIO_HIGH); 
   ```
4. **状态机开发规范**：应用层模块的状态机统一采用 **`perf_counter` 库的 PERF_COUNTER_FSM**框架，禁止使用裸 `switch-case`。
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
TARGET_CHIP ?= at32f421
```
当后续添加如 `stm32g4` 时，只需：
1. 建一个 `peripheral/stm32g4/` 放入它的两个 `port_xx.c` 文件。
2. 在 `makefile` 中增加一段 `ifeq ($(TARGET_CHIP),stm32g4)` 用来限定包含了它的驱动底层源文件路径及宏定义。
3. **只需在命令行调用 `make TARGET_CHIP=stm32g4` 即可一键拉起编译，所有应用层代码完美重用！**

同时，若在使用 AT32 期间启用了诸如 I2C/SPI，只需在 Makefile 内的专属外设开关寻找 `USE_DRV_XXX=0` 修改为 `1` 即可链接对应厂商源文件。

---

## 4. 编译、烧录与手动调试 (Compilation, Flashing & Debugging)

本工程默认使用 **LLVM 交叉编译工具链** 进行构建，基于 **OpenOCD + CMSIS-DAP/DAP-Link** 进行固件烧录。为缓解每次必须敲长命令之苦，根目录默认放置了一个快捷入口 `make.bat`。

### 1. 编译与烧录
在任何终端（PowerShell 或 CMD）直接执行：
```powershell
.\make.bat clean
.\make.bat             # 执行构建
.\make.bat flash       # 将产生的 hex/bin 直接烧录进入目标板
```

### 2. 挂载 RTT 调试通道
一旦成功烧制并开始工作，拉起本地 GDB 服务器：
```powershell
.\make.bat rtt
```

### 3. 日志捕获监控
使用 RTT 时为防止日志直接乱刷主终端，推荐使用准备好的监视器脚本：
```powershell
.\.agent\workflows\rtt_viewer.ps1
```
此时将弹出独立监控视窗，所有 `LOG_OUT()` 等通过 `SEGGER_RTT_WriteString` 输送的心跳日志将清晰呈现在这里。

---

## 5. Git 子模块管理 (Git Submodules Configuration)
由于外设底层、核心 CMSIS、GMSI 均属于独立外链依赖仓库。

**首次克隆本仓库架构：**
务必带上 `--recursive` 参数，这一步帮助你连同依赖仓库代码一同自动拉下来。
```bash
git clone --recursive https://your-repo-url/template-project.git
```
**忘记带参的自救指令：**
```bash
git submodule update --init --recursive
```

**对底层库版本的强制追更：**
```bash
cd gmsi
git checkout dev 
git pull origin dev
 git submodule update --init --recursive
cd ..
git add gmsi
git commit -m "chore: bump gmsi framework to latest dev branch"
```
