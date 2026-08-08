# motor 模块成员命名规范与缩写清理设计规范 (阶段 C)

## 1. 摘要与背景

针对 `docs/todo.md` 中的“事项 3：motor 头文件成员命名一致性”，本规范旨在清理 motor 核心模块内部隐晦、语义不明确的旧缩写，同时正式建立并文档化全库统一的命名约束与类型前缀约定。

为保证代码的稳定性与 MCU 嵌入式资源的极简，本次实施遵循“收窄范围”的评估结论：不进行全局机械式的泛化改动，只针对最具误导性的缩写（`tRt`）进行重命名，并正式文档化现有的 `ch/e` 内存优化前缀规范。

---

## 2. 重构与规范细节

### 2.1 隐晦缩写更名 (`tRt → tRuntime`)

在 [motor_private.h](file:///e:/Project/modus_template/foc/motor/motor_private.h) 及底层相关模块中，将表示“运行时状态快照”的结构体成员 `tRt` 全量更名为 `tRuntime`：

```c
typedef struct motor_impl_s {
    uint32_t                    wMagic;             /**< 魔数 MOTOR_IMPL_MAGIC */
    motor_state_t               tRuntime;           /**< 运行时状态快照（母线电压、故障、运行态） */
    ...
} motor_impl_t;
```

**访问形式变化**：
- `impl->tRuntime.eRunState`
- `impl->tRuntime.wFaults`
- `impl->tRuntime.qVbus`

---

### 2.2 `ch/e` 类型前缀与 RAM 优化约定文档化

项目内部存在形如 `chStartupPhase`（内部私有状态）与 `eStartupPhase`（外部导出快照）的前缀差异。

**正式约定规则**：
- **`ch` 前缀**：表示成员在私有结构体侧被显式收窄为 `uint8_t` 字节存储（例如 `uint8_t chStartupPhase`），用于在 MCU 上精简 `motor_impl_t` 的 RAM 占用，减少 32-bit 对齐空洞。
- **`e` 前缀**：表示在公共 API / 快照结构体中暴露的标准 32-bit `enum` 类型（例如 `motor_startup_phase_e eStartupPhase`）。

该差异属于有意的 RAM 优化设计，保持现状并予以标准文档化。

---

### 2.3 物理量语义词缀规范

为了在物理量命名中保持连贯与清晰，全局统一遵循以下词缀使用规则：

1. **`Reference`**：表示控制量的给定参考值（例：`qSpeedReference` / `tVoltageReference`）。
2. **`Active`**：表示当前运行回路中实际生效的物理量（例：`tActiveAngle` / `qActiveSpeed`）。
3. **`Candidate`**：表示多位置源无缝切换过程中候选观察者的输出（例：`tCandidateAngle`）。
4. **`Electrical` / `Mechanical`**：分别区分电角度/电速度与机械角度/机械速度。

---

## 3. 验证计划

更名 `tRt → tRuntime` 后，必须执行以下验证：
- `.\make.bat` (AT32F413 目标编译)
- `mingw32-make TARGET_CHIP=stm32g431` (STM32G431 目标编译)
- `mingw32-make -C tests/foc` (单元测试全量 Pass)
