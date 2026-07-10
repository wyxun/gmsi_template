# grblHAL Library Patches

> **重要说明**：`third_party/grblhal/` 目录是 **只读子库（submodule/vendor copy）**，不随项目提交更新。
> 本文档记录了为适配 STM32G431 128KB Flash 限制而对库文件所做的 **手动修改（patches）**，
> 当升级 grblHAL 库时，必须重新应用这些修改。

---

## 修改目的

STM32G431xB 仅有 **128KB Flash**，grblHAL 默认配置下固件超出该限制约 15KB。  
通过在库源码中添加条件编译宏，将以下非核心内容从二进制中裁剪掉：

| 宏定义                        | 控制内容                                | 预估节省  |
| :---------------------------- | :-------------------------------------- | :-------- |
| `NO_SETTINGS_DESCRIPTIONS`    | 设置项帮助文字字符串（`setting_descr`） | ~8.8 KB   |
| `NO_ERROR_DESCRIPTIONS`       | 错误/报警详细描述字符串                 | ~3.5 KB   |
| `NO_TOOL_CHANGE_SUPPORT`      | 手动换刀/自动对刀全套功能               | ~2.1 KB   |

这些宏统一在 **`grblhal_adapt/grblhal_config.h`** 中定义，该文件通过编译系统的 `-include` 机制全局注入。

---

## 修改清单

### 1. `third_party/grblhal/core/settings.c`

**位置**：`setting_descr` 静态数组（约第 2452 行）及其在 `global_settings` 中的注册（约第 2666–2683 行）

**修改方式**：在数组定义和注册代码外围添加 `#ifndef NO_SETTINGS_DESCRIPTIONS` 编译开关。

```c
// 原始代码（数组起始处）
static const setting_descr_t setting_descr[] = {
    ...
};

// 修改后
#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t setting_descr[] = {
    ...
};
#endif

// 原始代码（注册处，global_settings 结构体内）
.descriptions = setting_descr,
.n_descriptions = ...

// 修改后
#ifndef NO_SETTINGS_DESCRIPTIONS
.descriptions = setting_descr,
.n_descriptions = ...
#endif
```

---

### 2. `third_party/grblhal/core/errors.c`

**位置**：`status_detail` 静态数组内，第 30 行（`Status_OK` 条目之后）至第 113 行（数组结束）

**修改方式**：在所有带描述字符串的条目外围添加 `#ifndef NO_ERROR_DESCRIPTIONS` 编译开关。

```c
// 修改后结构
PROGMEM static const status_detail_t status_detail[] = {
    { Status_OK, NULL },
#ifndef NO_ERROR_DESCRIPTIONS
    { Status_ExpectedCommandLetter, "G-code words consist of..." },
    ...
    { Status_NoToolInSPindle, "No tool in spindle." }
#endif
};
```

---

### 3. `third_party/grblhal/core/tool_change.c`

**位置**：整个文件

**修改方式**：在文件首行添加 `#ifndef NO_TOOL_CHANGE_SUPPORT`，在文件末尾添加 `#else` 空桩实现和 `#endif`。

```c
// 文件开头
#ifndef NO_TOOL_CHANGE_SUPPORT
/*
  tool_change.c - ...
*/
...（原始完整实现）...
    return ok ? Status_OK : Status_GCodeToolError;
}
#else
// 空桩实现，满足链接器需求
#include "grbl.h"
#include "tool_change.h"
FLASHMEM void tc_init (void) {}
FLASHMEM status_code_t tc_probe_workpiece (void) { return Status_SettingDisabled; }
#endif
```

---

### 4. `third_party/grblhal/core/system.c`

**位置 A**：`set_tool_reference` 和 `tool_probe_workpiece` 函数（约第 584–610 行）  
**位置 B**：命令注册表中的 `$TLR` 和 `$TPW` 条目（约第 995–996 行）

**修改方式**：用 `#ifndef NO_TOOL_CHANGE_SUPPORT` 包裹上述函数定义及命令表条目。

```c
// 位置 A：函数定义
#ifndef NO_TOOL_CHANGE_SUPPORT
FLASHMEM static status_code_t set_tool_reference (...) { ... }
FLASHMEM static status_code_t tool_probe_workpiece (...) { ... }
#endif

// 位置 B：命令表
#ifndef NO_TOOL_CHANGE_SUPPORT
    { "TLR", set_tool_reference, ... },
    { "TPW", tool_probe_workpiece, ... },
#endif
```

---

## 升级库时的操作步骤

当需要更新 `third_party/grblhal/` 到新版本时，请按以下步骤重新应用补丁：

1. 检查上述 4 个文件在新版本中的改动，确认行号和上下文是否变化。
2. 依次按本文档描述，将 `#ifndef`/`#else`/`#endif` 编译开关重新插入对应位置。
3. 执行 `make.bat clean && make.bat BUILD=release`，确认最终 `text + data` ≤ 131072 字节（128KB）。
4. 如需开关某个特性，只需修改 `grblhal_adapt/grblhal_config.h` 中对应的 `#define`，无需再改库文件。

---

## 当前 Flash 使用情况（STM32G431xB，128K 限制）

优化后最终编译结果（Release / -Oz）：

```
   text     data      bss      dec
 127500     2072    11784   141356
```

- **Flash 使用**：127,500 + 2,072 = **129,572 bytes（126.5 KB）**，剩余约 **1.5 KB**
- **RAM 使用**：2,072 + 11,784 = **13,856 bytes（13.5 KB）**，剩余约 **8.5 KB**
