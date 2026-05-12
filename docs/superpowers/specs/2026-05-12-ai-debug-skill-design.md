# AI Debug Skill & AITrace CLI — Design Spec

**Date:** 2026-05-12
**Status:** Draft

## 1. Motivation

Modus Template 已有完善的 MCU 端调试基础设施（RTT Shell、波形、OpenOCD、GDB）
和 PC 端可视化工作台 mstudio。但 mstudio 是 GUI 应用，AI 无法读取其界面内容。

底层三个 TCP 通道对 AI 完全开放：
- RTT Ch0 (TCP 9090)：Shell 文本 + MLOG 日志
- RTT Ch1 (TCP 9091)：波形二进制帧
- OpenOCD Telnet (TCP 4444)：寄存器/内存读写

需要构建一个 CLI 工具 (`aitrace`) 将这些通道的数据转为 stdout 文本，供 AI 消费。
同时提供一个 Claude Code Skill (`aitrace-skill`) 封装诊断决策逻辑。

## 2. Design Goals

- AI 能通过 CLI 获取所有 MCU 调试数据（shell、波形、寄存器、内存、符号解析）
- 支持三种侵入层级，AI 默认走无侵入路径
- 带负载运行时不能暂停 CPU
- 半自动决策：AI 分析输出诊断报告，工程师确认后改代码
- 复用 mstudio 现有核心类，避免重复实现

## 3. Directory Layout

```
E:\Project\mstudio\                    # PC 调试工作台（独立 git repo）
├── core/                              # 静态库 libmstudiocore
│   ├── network_mgr.h/cpp              #   TCP 连接管理，双通道线程
│   ├── protocol_parser.h/cpp          #   波形二进制帧 → 采样数据
│   ├── ocd_client.h/cpp               #   OpenOCD telnet 客户端
│   ├── elf_parser.h/cpp               #   ELF 符号表解析
│   └── map_parser.h/cpp               #   .map 文件解析
├── src/                               # mstudio GUI（链接 core）
│   ├── main.cpp
│   ├── gui_layer.*
│   └── panels/
├── aitrace/                           # CLI 工具（链接 core）
│   ├── Makefile
│   └── src/
│       ├── main.cpp                   #   入口，subcommand 路由
│       ├── shell_cmd.h/cpp            #   aitrace shell xxx
│       ├── wave_cmd.h/cpp             #   aitrace wave xxx
│       ├── ocd_cmd.h/cpp              #   aitrace ocd xxx
│       ├── gdb_cmd.h/cpp              #   aitrace gdb xxx
│       ├── map_cmd.h/cpp              #   aitrace map xxx
│       └── crash_cmd.h/cpp            #   aitrace crash xxx
├── Makefile                           # 顶层：先编 core，再编 mstudio 和 aitrace
└── mstudio.cfg
```

`E:\Project\modus_template` 侧仅放 Skill 定义文件。

## 4. Three Intrusion Levels

```
                        侵入性      适用场景
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
A. 纯被动（无侵入）      无        带负载运行、电机控制
   只通过 RTT Ch0/Ch1 收数据
   shell 命令在 MCU 主循环内执行

B. halt 暂停（侵入）     中        可短暂停 CPU 的场景
   ocd halt → regs/mdw → resume

C. GDB 调试（侵入）      高        开发阶段断点单步
   需工程师显式启用
```

AI 默认走路径 A。需要 B 或 C 时，AI 提示工程师确认。

## 5. CLI Command Set

### 无侵入 — RTT Shell (TCP 9090)

```
aitrace shell <cmd>              # 发任意 shell 命令，打印返回
aitrace shell regs               # MSP/PSP/CONTROL/PRIMASK/FAULTMASK/BASEPRI/LR
aitrace shell peek <hex_addr>    # 读 uint32
aitrace shell stack [depth]      # SP 附近栈 + LR/PC 候选标记
aitrace shell cfsr               # 故障状态寄存器（CFSR/HFSR/MMAR/BFAR）及解码
aitrace shell list               # MODUS 对象和事件掩码
aitrace shell log [-E] [-W] [-I] [-D] [-T]   # 运行时切换 MLOG 级别
```

### 无侵入 — 波形 (TCP 9091)

```
aitrace wave list                # 列出当前通道
aitrace wave start               # 开始采集
aitrace wave stop                # 停止
aitrace wave rate <n>            # 调整抽取率（n=0 外部驱动，n>=1 每 n 次 Step 发一帧）
aitrace wave capture <seconds>   # 采集 N 秒，CSV 输出到 stdout
aitrace wave capture <seconds> --output <file.csv>
```

### 侵入 — OpenOCD (TCP 4444)

```
aitrace ocd halt                 # 暂停 CPU  ⚠ 需确认
aitrace ocd resume               # 恢复
aitrace ocd regs                 # 全部核心寄存器
aitrace ocd peek <hex_addr>      # 读 uint32
aitrace ocd mdw <addr> [count]   # 连续内存转储
aitrace ocd stack [depth]        # 栈转储 + LR/PC 候选
```

### 侵入 — GDB MI2（需工程师显式启用）

```
aitrace gdb connect [--port 3333]      # 启动 GDB MI2 长连接  ⚠ 需确认
aitrace gdb break <location>           # 设断点
aitrace gdb continue                   # 继续运行
aitrace gdb step                       # 单步
aitrace gdb print <expr>               # 读变量值
aitrace gdb bt                         # 调用栈
aitrace gdb detach                     # 断开
```

### 符号解析

```
aitrace map resolve <elf_or_map> <addr1> [addr2 ...]   # 地址 → 符号 + 偏移
aitrace map info <elf_or_map>                           # 段大小/空间统计
```

### 崩溃分析

```
aitrace crash report --pc=<hex> --lr=<hex> --sp=<hex> --elf=<path> [--stack=<hex_vals>]
# 组合：map resolve + CFSR 解码 → 人类可读报告
```

## 6. GDB MI2 Module Design

唯一需要全新编写的核心模块。

```
GdbMiClient
├── Start(gdb_path, elf_path) → 启动 gdb -i=mi2 <elf>
├── Send(cmd)                 → 发 MI 命令
├── Recv(timeout_ms)          → 收解析后的 MI 记录（阻塞/超时）
└── Stop()                    → -gdb-exit
```

- 单次长连接，状态机：`idle → target-connected → running → stopped → ...`
- MI 命令超时默认 5s，不阻塞 CLI
- GDB 进程崩溃 → 检测 EOF，自动清理
- gdb 模式与 ocd 模式互斥（GDB 通过自身的 OpenOCD GDB server 端口 3333 控制 MCU）
- `gdb connect` 时自动检测 OpenOCD 是否已启动（连 4444 端口试探），未启动则报错

## 7. Skill Workflow

```
工程师触发（/aitrace debug 或自动检测到异常）
                │
                ▼
┌───────────────────────────────────┐
│ Skill 判断场景                      │
│  - 有 crash dump? → crash-analyze │
│  - 行为异常?     → 无侵入诊断      │
│  - 性能/波形问题? → 波形采集       │
└───────────────┬───────────────────┘
                │
                ▼
        aitrace 命令组合
                │
                ▼
┌───────────────────────────────────┐
│ AI 分析 stdout                     │
│  - 地址 → map resolve → 符号名    │
│  - 寄存器值 → 判断故障类型         │
│  - 栈回溯 → 重构调用链             │
│  - 波形 CSV → 趋势/异常检测        │
└───────────────┬───────────────────┘
                │
                ▼
         输出诊断报告
        (可能原因 + 证据)
                │
                ▼
         工程师确认
         (纠偏 / 采纳 / 补充方向)
                │
                ▼
         AI 修改代码
                │
                ▼
         make.bat auto
         验证修复
```

### 三个固化诊断流程

**HardFault 分析：**
RTT Ch0 已有自动转储（core_debug_cm_fault.c 在故障时打印异常帧 + CFSR）。
Skill 从 RTT 日志解析 → `map resolve` → 输出 "PC 在 foo()+0x12 处，foo() 第 18 行疑似无效指针访问"。

**无侵入运行时诊断：**
`wave capture` 采波形 CSV → `shell regs`/`shell list` → AI 综合分析。

**变量探查：**
无侵入走 `shell peek`，侵入需确认后走 `ocd peek`，断点需确认后走 `gdb break + print`。

## 8. Safety Boundaries

```
Skill 自动执行（无需确认）:
  ✓ TCP 9090/9091 连接（纯被动）
  ✓ shell 命令（MCU 主循环内执行）
  ✓ 波形采集
  ✓ 日志/寄存器/栈解析
  ✓ map 地址解析
  ✓ 生成诊断报告

需要工程师确认:
  ⚠ aitrace ocd halt        （暂停 CPU）
  ⚠ aitrace gdb connect     （断点调试）
  ⚠ 修改源代码
  ⚠ make.bat flash          （重烧固件）
```

## 9. Dependency Graph

```
libmstudiocore
├── network_mgr    → Winsock (ws2_32)
├── protocol_parser → （纯计算，无外部依赖）
├── ocd_client     → Winsock
├── elf_parser     → （纯解析，无外部依赖）
└── map_parser     → （纯解析，无外部依赖）

aitrace.exe → libmstudiocore + GDB subprocess (gdb-multiarch)
mstudio.exe → libmstudiocore + SDL2 + imgui + implot

modus_template/tools/ → 不变
modus_template/.claude/skills/aitrace-skill.md → Skill 定义
```

## 10. Resolved Decisions

- Skill 文件放在 `modus_template/.claude/skills/aitrace-skill.md`，与 template 绑定
- `aitrace gdb connect` 自动检测 OpenOCD 是否已启动（连 4444 探测），未启动则报错提示
- 支持 `aitrace wave rate <n>`，对应 MCU 端 `mwaveform.SetRate(n)`，动态调整抽取率
