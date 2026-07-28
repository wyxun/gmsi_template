# Modus 框架评估与优化建议

> 2026-07-27 · 基于 foc_app 实际挂载经验

## 1. 概述

Modus 是一个轻量级 MCU 应用框架（4 文件，~700 行），提供模块注册、1ms 周期性调度、基于链表的 Clock/Run 分发和 RingBuffer/Event IPC。项目目前挂载 foc_app、template_class（以及其他可选模块），运行在 Cortex-M4 @ 168MHz 上。

整体评价：**框架内核正确，功能够用，但在类型安全、内存布局和不必要抽象上存在可优化的设计债。**

## 2. 当前架构

```
┌──────────────────────────────────────────────────┐
│  modus_Init()                                    │
│    → 遍历 linker section 注册表                   │
│    → 逐一调用 module_Init(wObjectAddr, wCfgAddr)  │
│    → 填充全局链表 tListObject                      │
├──────────────────────────────────────────────────┤
│  modus_Clock()   [SysTick 1ms]                   │
│    → 遍历链表 → pFcnInterface->Clock(wParent)     │
│    → foc_app_Clock((uintptr_t)&tFocApp)           │
│       → motor_LowFrequencyStep()                 │
├──────────────────────────────────────────────────┤
│  modus_Run()     [主循环, ~100μs 空闲]            │
│    → 遍历链表 → pFcnInterface->Run(wParent)       │
│    → foc_app_Run((uintptr_t)&tFocApp)             │
│       → 按钮处理 / Shell / 事件Drain / 1Hz心跳     │
└──────────────────────────────────────────────────┘
```

每个模块持有独立的 `modus_base_t`（通过指针引用），框架通过全局链表管理所有实例。

## 3. 问题清单（按优先级排列）

### 问题 1：指针碎片化——`modus_base_t` 应嵌入而非指针引用

**现状**：

```c
// foc_app.h — 两层分离
typedef struct foc_app_s {
    modus_base_t    *ptBase;     // 指针指向另一个 static 变量
    uint32_t         wLastHeartbeatTick;
    // ...
} foc_app_t;

// foc_app.c — 两块独立存储
static modus_base_t  s_tFocAppBase;      // ~40 bytes 额外分配
foc_app_t            tFocApp;            // MODUS_DECLARE_OBJECT 展开

// Init 中手动绑定 + 回填
ptThis->ptBase = &s_tFocAppBase;
s_tFocAppBaseCfg.wParent = wObjectAddr;  // 双向绑定
mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
```

**问题**：
- `foc_app_t` 和 `modus_base_t` 是两块堆外独立内存，通过互相引用维持关联
- 访问路径多一次指针解引用（`ptThis->ptBase->wEvent`）
- `wParent = (uintptr_t)ptThis` 是隐式契约——类型安全性由程序员人工保证
- mbase_Init 内 `mlist_Insert` 操作的是 `s_tFocAppBase`，但 `wParent` 指向的是 `tFocApp`——同一个人两份身份证

**建议方案**：嵌入 `modus_base_t` 为第一个成员

```c
typedef struct foc_app_s {
    modus_base_t     tBase;     // 嵌入，内存连续
    uint32_t         wLastHeartbeatTick;
    // ...
} foc_app_t;

// Init:
mbase_Init(&ptThis->tBase, &s_tFocAppBaseCfg);
```

调用 `modus_Clock()` 时通过 `container_of(ptBaseDes, foc_app_t, tBase)` 拿到 `foc_app_t*`。

**收益**：
- 省掉独立 static 变量（每个模块 ~40 bytes），内存连续，cache 友好
- 消除 `ptBase` 判 NULL 分支
- 消除 wParent 手动回填（container_of 不需要 wParent）

**迁移代价**：中等
- 需要修改 Modus 框架层的 Clock/Run 分派（目前传 `wParent`，改为传 `modus_base_t*` 再用 `container_of`）
- 或者引入 `offsetof` 宏实现 `MODUS_CONTAINER_OF`
- 所有模块 `foc_app_Clock/Run` 签名需从 `(uintptr_t)` 改为 `(void*)` 或直接从 `foc_app_t*` 开始
- 影响范围：foc_app、template_class、以及所有使用 MODUS_DECLARE_OBJECT 的模块

---

### 问题 2：`wParent` 往返——类型不安全的 `uintptr_t` 传参

**现状**：

```c
// modus_base_cfg_t
typedef struct {
    uint32_t   wId;
    uintptr_t  wParent;     // "父对象地址"——uintptr_t
    modus_interface_t FcnInterface;
} modus_base_cfg_t;

// Clock/Run 回调签名
int (*Clock)(uintptr_t wObjectAddr);
int (*Run)(uintptr_t wObjectAddr);

// 调用侧：
int foc_app_Clock(uintptr_t wObjectAddr) {
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;  // 裸 cast
    // ...
}
```

**问题**：
- `uintptr_t` 抹掉了所有类型信息。`foc_app_Clock` 拿到的可能是任何地址，编译器无法检查
- 每个回调第一行都是相同的 `(MyType *)wObjectAddr` 模板——但类型对错没人知道
- 与 Modus 自己的 `void *pvOwner`（在 `mlist_item_t` 中）不一致——框架内部不一致

**建议方案**：

如果问题 1 被采纳，最优解是直接传 `modus_base_t*`：

```c
// Clock/Run 回调签名改为
int (*Clock)(modus_base_t *ptBase);

// 框架侧调用
ptBaseDes->pFcnInterface->Clock(ptBaseDes);

// 模块侧用 container_of
int foc_app_Clock(modus_base_t *ptBase) {
    foc_app_t *ptThis = MODUS_CONTAINER_OF(ptBase, foc_app_t, tBase);
    // ...
}
```

如果问题 1 不被采纳，退一步：把 `uintptr_t` 改为 `void*`（与 `pvOwner` 一致）。

**收益**：
- 编译器能做类型检查
- 消除每个回调的裸 cast
- 框架内部 API 风格一致（`void*` / 特定类型而非 `uintptr_t`）

**迁移代价**：取决于问题 1
- 跟随问题 1 一起做：零额外成本
- 单独做：改框架层的所有 `uintptr_t → void*`，以及所有模块的回调签名

---

### 问题 3：链表 IO 开销——为 2 个模块做 O(n) 遍历

**现状**：

```c
void modus_Clock(void) {
    for (每个链表节点) {
        ptBase = 从 pvOwner 取 modus_base_t*;
        ptBase->pFcnInterface->Clock(ptBase->wParent);
    }
}
```

**问题**：
- 项目挂载的模块 ≤ 3 个，链表深度恒定为 2~3
- 每 1ms 的遍历+虚表+回调链是 O(n) 但 n 从未 > 3
- 链表的核心价值（动态插拔、运行时增删模块）在当前项目中从未被使用
- 同样的问题也出现在 `modus_Run()`、`mbase_EventPost()`、`mbase_MessagePostToRing()`——每一个 IPC 操作都是 O(n) 链表遍历

**CPU 影响评估**（168MHz M4，2 个模块）：

| 操作 | 频率 | 链表遍历耗时 | 占比 |
|---|---|---|---|
| modus_Clock 遍历 | 1ms (1kHz) | ~15 cycles | < 0.01% |
| modus_Run 遍历 | ~100μs 空闲 | ~15 cycles | < 0.01% |
| EventPost 遍历 | 偶发 | ~10 cycles | 忽略 |

**结论**：**CPU 开销完全不构成问题**。但结构上是一种过度设计——为一个不需要动态性的系统引入了动态数据结构的复杂度。

**建议**：**不动**。

改为静态数组调度（`static_module_dispatch_table[]`）会让框架丧失通用性，而实际性能收益为零。这是"审美上不够简洁但运行上没有危害"的典型，不值得为它动一动。

---

### 问题 4：Event 机制——位掩码丢失顺序和优先级

**现状**：

```c
// 发布端：按 ID 找到模块，叠加事件位
ptBaseDes->wEvent |= wEvent;           // uint32_t 位掩码

// 消费端：一次性取走并清零
uint32_t wEvent = ptBase->wEvent;
ptBase->wEvent = 0;                    // 只能拿全部或没有
```

**问题**：
- 多个事件在同一帧内到达时，消费者只知道"有哪些类别"，不知道顺序，不知道有几份
- 没有优先级的概念——紧急故障和普通通知混在同一个位掩码里
- 消费是 all-or-nothing 的——无法先处理高优先级、再回来处理剩下的

**实际影响评估**：

当前项目中通过 `Modus_Event_Transition` 的用法只有一处：`foc_app_Run()` 内 `mbase_EventPend()` → 丢弃返回值。也就是说，当前 Event 机制**实际上没有在被使用**。foc_app 的真实事件驱动机理是 `motor_DebugReadEvent()` 的环形缓冲区，和 Modus Event 无关。

**建议**：**不动**。

Event 机制当前是死代码——它不是 bug，它只是没有在被用。如果将来需要模块间通知，届时再改成 FIFO 队列或 FreeRTOS 任务通知。现在去优化一个不用的子系统没有意义。

---

### 问题 5：可观测性——Debug 输出不可消费

**现状**：

```c
void mbase_DebugListBase(void) {
    MLOG(D, "List all object:\n");
    for (...) {
        MLOGF(D, "    item id: 0x%x\n", (unsigned)ptListItemDes->wItemValue);
    }
}
```

**问题**：
- 只在 `modus_Init()` 结束前调用一次，仅打印到串口
- 格式是人读的字符串，自动化测试/CI 无法解析
- `mbase_GetBaseList()` 暴露了链表头指针，但对外部来说类型是 `mlist_t*`——不包含模块元数据

**建议**：**低优先级，不动**。

如果未来需要系统健康检查（例如 UART 命令 `$modus/stats`），可以提供一组 struct 化的查询 API：

```c
typedef struct {
    uint32_t wId;
    uint32_t wEvent;
    bool     bClockRegistered;
} modus_module_info_t;

size_t modus_GetModuleList(modus_module_info_t *ptOut, size_t wMaxCount);
```

但现在不需要——串口打印足够调试用。

---

## 4. 总结

| # | 问题 | 影响 | 迁移成本 | 建议 |
|---|---|---|---|---|
| 1 | `ptBase` 指针碎片 | 内存浪费，一次额外寻址 | 中 | **值得做** |
| 2 | wParent uintptr_t | 类型不安全 | 跟随 #1 零额外 | **跟随 #1** |
| 3 | 链表遍历 O(n) | 无实际影响 | — | 不动 |
| 4 | Event 位掩码 | 当前未使用 | — | 不动 |
| 5 | Debug 输出 | 低 | 低 | 不动（将来再说） |

**核心建议**：只做 #1（嵌入 modus_base_t），顺带解决 #2。这是唯一有实际收益的改动——省内存、消除类型不安全的 cast、让模块从 "has-a base pointer" 变成 "is-a Modus module"。其余问题不影响正确性和性能，优化它们属于为审美的 engineering over-engineering。
