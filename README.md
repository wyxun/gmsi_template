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

### 步骤 2：上层应用业务逻辑与模块化 (Application Logic & Modularization)

本工程采用 **面向对象 C 语言 (OOPC)** 的思想。每个业务模块（如 `template_class.c`）都被视为一个独立功能的“类”。

#### 1. 核心接口分工 (Lifecycle)
每个业务模块应遵循标准的接口实现规范：
- **`Init`**: 初始化入口。在 `gmsi_Init()` 时被自动调用，用于挂载配置、初始化私有变量等。
- **`Run`**: 轮询入口。在 `main` 循环的 `gmsi_Run()` 中被调用。适合执行非阻塞的状态机或后台任务。
- **`Clock`**: 定时入口。由 `SysTick` (1ms) 驱动，在 `gmsi_Clock()` 中被调用。适合处理高精度计数或定时逻辑。

#### 2. OOPC 语法糖：使用 `this` 指针
为了提高代码可读性，建议在模块 `.c` 文件头部定义 `this` 宏：
```c
#undef  this
#define this (*ptThis)   /* 启用语法糖：this.member 等同于 ptThis->member */
```

#### 3. 模块自动加载 (GMSI_DECLARE_OBJECT)
**禁止在 `main.c` 中显式创建大量业务对象。** 统一在模块内部通过 `GMSI_DECLARE_OBJECT` 宏完成实例化与注册。
```c
// 在模块末尾一键加载：系统启动时会自动调用 template_class_Init
GMSI_DECLARE_OBJECT(template_class, TemplateClass, 
    .pchRingBuffer = s_chBuffer, 
    .hwRingSize = 128
)
```
- **机制**：通过 `init_infos` 内存段实现自动注册，无需修改 `main.c` 即可增加新功能模块。

#### 4. 状态机开发规范 (FSM)
应用层模块的状态机**必须**采用 **`perf_counter` 库的 PERF_COUNTER_FSM** 框架，**严禁使用裸 `switch-case`**。
```c
fsm_rt_t template_class_Run(template_class_t *ptThis)
{
    PERFC_PT_BEGIN(this.chState)    /* 恢复上次挂起点 */
    
    PERFC_PT_DELAY_MS(500)         /* 非阻塞延时 500ms */
    
    PERFC_PT_ENTRY(
        do_work();                  /* 进入该状态时执行一次 */
    )
    PERFC_PT_WAIT_UNTIL(is_done()); /* 等待条件成立 */
    
    PERFC_PT_YIELD(fsm_rt_on_going);/* 挂起并让出 CPU */
    
    PERFC_PT_END()                  /* 结束重置状态并返回 fsm_rt_cpl */
    return fsm_rt_cpl;
}
```

#### 5. 硬件调用：GDI 接口
业务层与硬件交互时，必须通过 `gdi_hw.h` 获取全局资源池 `HW`，并使用通用 GDI 宏：
```c
// 示例：操控继电器或串口（与特定芯片引脚解耦）
GDI_Write(HW.ptRelay1, GDI_GPIO_HIGH); 
GDI_Write(HW.ptSerial, "Hello", 5);
```

| 常用宏 | 对应操作 |
|----|------|
| `GDI_Write(ptObj, val)` | 写入状态/数据 |
| `GDI_Read(ptObj)` | 读取状态/接收数据 |
| `GDI_Toggle(ptObj)` | 翻转 GPIO 状态 |
| `GDI_IsBusy(ptObj)` | 查询外设是否忙碌（针对 UART/I2C 等） |

> 完整宏定义参考 `gmsi/gmsi/gdi/gdi.h`。
 
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

### 4. 自动化测试
如果你希望一次性完成“清理、编译、烧录、启动 RTT 服务器、打开 RTT 监视器”的全流程，可以使用一键自动化指令：
```powershell
.\make.bat auto
```
该脚本将严格按照以下顺序执行：
1. **clean**: 清除旧的编译文件。
2. **build**: 重新编译生成最新的固件。
3. **flash**: 自动通过 OpenOCD 烧录到目标板。
4. **rtt**: 在后台静默启动 `openocd` 作为 RTT 代理。
5. **rtt_viewer**: 自动弹出 PowerShell 监视窗口显示实时日志。
6. **auto**: 一键执行以上所有步骤。
7. **release**: 生成 release 版本(-os 无调试信息)。

---

## 5. VSCode Cortex-Debug 调试说明

本工程已完美适配 VSCode 的 **Cortex-Debug** 插件，通过集成 RTT console 与 SVD 寄存器描述文件，实现极其便捷的图形化断点调试。

### 1. 环境准备
- **插件安装**：请从 VSCode 插件市场安装 [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)。
- **路径确认**：确保 `.vscode/launch.json` 中的 `serverpath` (OpenOCD 路径) 与 `armToolchainPath` (LLVM 环境路径) 指向你电脑上的实际安装目录。

### 2. 开发与调试流程
1. **启动调试**：按下快捷键 **`F5`**。
2. **自动构建**：系统会自动触发 `Build ELF` 任务（调用 `make.bat build`）确保代码是最新的。
3. **固件植入**：底层自动启动 OpenOCD 将编译好的 `.elf` 固件下载至 AT32F421 芯片，并默认停在 `main` 函数入口。
4. **实时日志**：在 VSCode 的 **DEBUG CONSOLE (调试控制台)** 或 **Output (输出)** 面板的 RTT 通道中，可以直接实时查看 `LOG_OUT` 产生的日志。

### 3. 特色高级功能
- **外设寄存器查看**：在调试模式下，左侧侧边栏底部会出现 **CORTEX PERIPHERALS** 面板。依靠项目自带的 `AT32F421xx_v2.svd` 文件，你可以直观查看所有外设（如 CRM, TMR, USART, GPIO）的实时寄存器位。
- **变量监控与断点**：支持标准的断点调试、全路径调用堆栈跟踪以及变量 Watch 监视。

---

## 6. Git 子模块管理 (Git Submodules Configuration)
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
