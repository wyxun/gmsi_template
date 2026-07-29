# ATC 自动换刀功能设计规范

**日期**: 2026-07-22  
**版本**: 1.0  
**目标芯片**: AT32F407（GRBLHAL_FULL_FEATURES）

---

## 1. 概述

### 1.1 目标

为 grblHAL CNC 控制器固件添加自动换刀（ATC）功能，支持刀架式（Rack）刀库下的全自动 M6 换刀流程。

### 1.2 硬件方案

| 属性 | 规格 |
|---|---|
| 刀库类型 | 单排直线刀架（Rack） |
| 刀位数 | 5-8 个（默认 8） |
| 松夹刀 | 主轴气动/电磁阀控制，高电平松刀 |
| 传感器 | 最简配置——无在位检测，纯延时控制 |
| 换刀流程 | 标准两步走——先还旧刀再取新刀 |

### 1.3 关键约束

- 不修改 grblHAL 上游核心源代码
- 不使用 SD 卡或宏文件
- 所有安全依赖运动精度和延时

---

## 2. 架构设计

### 2.1 插件注册路径

`plugins_init.h` 在 grblHAL 核心中预留给驱动作者在 `driver_init()` 末尾 include，
本项目当前未使用该机制。改为在 `my_plugin_init()` 中调用 `atc_init()`，
该函数由 `grblhal_driver_setup()` 显式调用，确定在 `tc_init()` 之前执行。

`atc_init()` 负责：
1. 设置 `hal.driver_cap.atc = 1`（告知核心 ATC 硬件存在）
2. 注册 `hal.tool.change`、`hal.tool.select`、`hal.tool.atc_get_state` 三个回调
3. 初始化 PB0/PB1 为 GPIO 输出，默认低电平

设置 `hal.driver_cap.atc` 后，核心 `grbllib.c` 跳过了 `tc_init()` 调用，
手动换刀逻辑不再注册。同时 `gcode.c` 在 M6 时选择 ATC 路径而非手动路径。

### 2.2 调用链路

```
gcode.c (M6)
  → 检查 hal.driver_cap.atc → ATC_Online
  → 调用 hal.tool.change(parser_state)  ← ATC 插件执行换刀序列
  → 如果失败 → 返回 Status_Reset → Alarm

gcode.c (Tn)
  → 检查 hal.driver_cap.atc → ATC_Online
  → 调用 hal.tool.select(tool, true)     ← ATC 插件记录目标刀号
```

### 2.3 文件清单

| 文件 | 类型 | 职责 |
|---|---|---|
| `grblhal_adapt/atc_plugin.h` | 新增 | ATC 结构体、刀位配置、宏定义 |
| `grblhal_adapt/atc_plugin.c` | 新增 | ATC 状态机、运动序列、回调实现、shell 命令 |
| `grblhal_adapt/grblhal_driver.h` | 修改 | 追加 PB0/PB1 引脚定义 |
| `grblhal_adapt/grblhal_stubs.c` | 修改 | driver_init 中 GPIO 初始化、设置 `driver_cap.atc=1` |
| `grblhal_config.h` | 修改 | 追加 `ATC_ENABLE`、`N_TOOLS`、编译期默认值 |
| `docs/cnc_pin_mapping.md` | 修改 | 追加 ATC 引脚分配 |

---

## 3. I/O 引脚分配

### 3.1 当前已占用引脚（摘要）

| 引脚 | 功能 | 方向 |
|---|---|---|
| PA2 | TX (USART2) | 输出 |
| PA3 | RX (USART2) | 输入 |
| PA4 | SPI Flash CS | 输出 |
| PA5 | SpinDir (D13) | 输出 |
| PA6 | SpinEnable / TMR3_CH1 PWM (D12) | 输出/PWM |
| PA7 | Z-EndStop (D11) | 输入 |
| PA8 | Z-DIR (D7) | 输出 |
| PA9 | EN / Stepper Enable (D8) | 输出 |
| PA10 | X-STEP (D2) | 输出 |
| PA15 | Y-EndStop (D10) | 输入 |
| PB3 | Y-STEP (D3) | 输出 |
| PB4 | X-DIR (D5) | 输出 |
| PB5 | Z-STEP (D4) | 输出 |
| PB6 | Y-EndStop (D10 alt) | 输入 |
| PB10 | Y-DIR (D6) | 输出 |
| PB13-PB15 | SPI (SCK/MISO/MOSI) | I/O |
| PC5 | Probe (A5) | 输入 |
| PC7 | X-EndStop (D9) | 输入 |
| PD13 | Status LED | 输出 |

### 3.2 ATC 新增引脚

选择 **PB0** 和 **PB1**——B 口最干净的空闲引脚，紧邻在一起，无复用冲突，均为 5V 耐压 IO。

| 引脚 | 信号名 | 方向 | 空闲状态 | 说明 |
|---|---|---|---|---|
| **PB0** | `ATC_DRAW_BAR` | 输出 | 低电平 | 松夹刀电磁阀，高电平=松刀 |
| **PB1** | `ATC_AIR_BLAST` | 输出 | 低电平 | 吹气清理锥孔，高电平=吹气 |

### 3.3 宏定义（`grblhal_driver.h`）

```c
/* ATC I/O */
#define ATC_DRAW_BAR_PORT       GPIOB
#define ATC_DRAW_BAR_PIN        GPIO_PINS_0
#define ATC_AIR_BLAST_PORT      GPIOB
#define ATC_AIR_BLAST_PIN       GPIO_PINS_1
```

---

## 4. 配置参数

### 4.1 编译期宏（`grblhal_config.h`，仅 GRBLHAL_FULL_FEATURES 段追加）

```c
#define ATC_ENABLE              1       // 启用 ATC 插件
#define N_TOOLS                 8       // 工具表容量
#define ATC_DEFAULT_POCKETS     8       // 默认刀架刀位数
#define DEFAULT_ATC_DRAW_DELAY_MS     500  // 松夹刀电磁阀动作延时 (ms)
#define DEFAULT_ATC_POCKET_PITCH      50.0f  // 相邻刀位间距 (mm)
```

### 4.2 运行时设置

使用 grblHAL 预留的 **900-999 范围**：

| 编号 | 名称 | 类型 | 范围 | 默认值 | 说明 |
|---|---|---|---|---|---|
| $900 | ATC 刀位数 | uint8 | 1-16 | 8 | 刀架总刀位数 |
| $901 | 刀位间距 | float | 1-500 mm | 50.0 | 相邻刀位 X 轴间距 |
| $902 | 刀架基准 X | float | -9999..9999 mm | 0.0 | 1号刀位绝对 X 坐标 |
| $903 | 刀架基准 Y | float | -9999..9999 mm | 0.0 | 刀架 Y 坐标 |
| $904 | Z 安全高度 | float | -9999..9999 mm | 0.0 | 刀架上方 XY 快移的 Z 高度 |
| $905 | Z 取刀深度 | float | -9999..9999 mm | 0.0 | 取/放刀柄的 Z 坐标 |
| $906 | 松刀延时 | uint16 | 10-5000 ms | 500 | 松夹刀电磁阀稳定时间 |
| $907 | ATC 选项 | bitfield | - | 0 | bit0=换刀时吹气, bit1=换刀后回原 XY |

### 4.3 刀位坐标自动计算

每个刀位 i（1-based，i=1..N）的坐标：

```
Pocket[i].X = $902 + (i - 1) × $901
Pocket[i].Y = $903
Pocket[i].Z_pickup = $905
Pocket[i].Z_clear  = $904
```

### 4.4 设置注册代码

在 `atc_init()` 中调用 `settings_register()` 注册 6 个自定义设置（$900-$905）。$906-$907 使用两个字段从 `atc_settings_t` 结构体直接映射。

---

## 5. 换刀流程

### 5.1 状态机

ATC 不使用异步状态机。换刀全程在 M6 回调中**同步执行**，因为：
- grblHAL 的 `mc_line()` 运动 API 天然支持同步等待（`protocol_buffer_synchronize()`）
- 换刀期间 g-code 解析器自然暂停（`parser_state->tool_change = true`）
- 与其他模块（冷却液、主轴）交互简单

### 5.2 主流程（伪代码）

```
tool_change(parser_state):
    1. 参数校验
       ├─ next_tool == NULL         → Status_GCodeToolError
       └─ current_tool == next_tool  → Status_OK（空操作）

    2. 获取刀位坐标
       old_pos = 当前工具刀位（从工具表查 tool_id → pocket）
       new_pos = 目标工具刀位（从工具表查 next->tool_id → pocket）

    3. 停止主轴 + 冷却液

    4. 保存当前 WCS 位置
       切换到机床绝对坐标

    5. 还旧刀（如主轴有刀）:
       a. Z 抬至 $904（安全高度）             → mc_line()
       b. XY 快移至旧刀刀位上方                → mc_line()
       c. Z 降至 $905（取刀深度）               → mc_line()
       d. 松刀（DRAW_BAR=1），延时 $906         → delay
       e. 如启用吹气，打开 AIR_BLAST             → GPIO
       f. Z 抬至 $904                            → mc_line()
       g. 关闭吹气 / 松刀                       → GPIO
       h. 同步 → protocol_buffer_synchronize()

    6. 取新刀（如新刀非空刀位）:
       a. XY 快移至新刀刀位上方                → mc_line()
       b. Z 降至 $905（取刀深度）               → mc_line()
       c. 如启用吹气，打开 AIR_BLAST             → GPIO
       d. 夹刀（DRAW_BAR=0），延时 $906         → delay
       e. 关闭吹气                               → GPIO
       f. Z 抬至 $904                            → mc_line()
       g. 同步 → protocol_buffer_synchronize()

    7. 可选：恢复 XY 到换刀前位置（$907 bit1）

    8. 更新 current_tool = next_tool

    9. parser_state->tool_change = true
       设置 EXEC_TOOL_CHANGE

    10. 返回 Status_OK
```

### 5.3 空刀位处理

- 如果 `next_tool->tool_id == 0`（T0）：只还旧刀不取新刀，主轴空转
- 如果当前主轴无刀（`current_tool.tool_id == 0`）：跳过还刀步骤，直接取新刀

### 5.4 异常处理

- 任何 `mc_line()` 返回 `false` → 立即返回 `Status_Reset` → 核心进入 Alarm
- 急停事件 → `grblhal_emergency_stop()` → 步进定时器关闭、主轴 PWM 归零、电磁阀失电（弹簧复位夹紧）
- 限位触发 → grblHAL 核心已处理，停止运动
- 电磁阀失电默认夹紧（fail-safe），即使断电刀具不会掉落

---

## 6. 三个 HAL 回调实现

| 回调 | 实现 |
|---|---|
| `atc_get_state()` | 始终返回 `ATC_Online` |
| `atc_tool_select(tool, next)` | next=true 时记录目标刀号到静态变量 `s_pending_tool_id`；next=false 时直接写入当前刀具（用于 M61 的手动设定） |
| `atc_tool_change(parser_state)` | 执行 5.2 完整换刀序列 |

---

## 7. Shell 命令

通过 `MODUS_SHELL_CMD` 注册调试命令：

```
atc_draw on         手动松刀（测试用）
atc_draw off        手动夹刀（测试用）
atc_blast on        手动吹气（测试用）
atc_blast off       手动关吹气
```

---

## 8. 测试计划

### 8.1 硬件基础测试

1. `atc_draw on/off` 验证电磁阀响应
2. `atc_blast on/off` 验证吹气电磁阀（如连接）
3. M62/M63 辅助输出测试备用访问路径

### 8.2 刀位坐标验证

1. Z 安全高度、Z 取刀深度、各刀位 XY 的手动验证（先切空气不装刀）

### 8.3 换刀流程测试

1. T0 M6（还刀，主轴空）——无物理刀具时的安全测试
2. Tn M6（正常换刀 n≥1）——新刀在刀位
3. 重复两次 M6 验证同一刀具空操作
4. 急停中换刀验证 fail-safe

---

## 9. 附录：AT32F407 GPIO 快速参考

AT32F407 有 5 个 GPIO 端口（PA-PE），每个 16 脚。

PB0-PB1 特性：
- 5V 耐压
- 默认 GPIO（非 JTAG/SWD/TMR 复用）
- 相邻布局，便于排线
