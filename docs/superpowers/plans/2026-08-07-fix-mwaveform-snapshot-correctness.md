# mwaveform Snapshot 数据正确性修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 mwaveform snapshot 的 mask、采样节拍、有效样本数、pending 生命周期和实际深度查询，使 20kHz snapshot 输出可被 MStudio 正确解析。

**Architecture:** snapshot 不再复用 stream 的 `Step()` 局部 mask 和 `wSampleIndex`，改为独立采样节拍：每个 `SnapshotFeed()` 调用读取当前变量/缓存值，维护独立样本序号和有效样本计数；触发时只发送已写入的有效样本；停止时保留已触发但未发送的 pending 帧。

**Tech Stack:** C（STM32G431 / MODUS）、SEGGER RTT、MStudio ProtocolParser C++ 测试。

---

## File Structure

- Modify: `modus/src/mdebug/mwaveform.c`
- Modify: `modus/src/mdebug/mwaveform.h`
- Modify: `modus/docs/mdebug/mwaveform.md`
- Modify: `E:\Project\mstudio\tests\protocol_parser_test.cpp`
- Verify: `build/template.elf`

---

### Task 1: 增加 Snapshot 独立状态

**Files:**
- Modify: `modus/src/mdebug/mwaveform.c:37-56`

- [ ] **Step 1: 在 `mwaveform_cb_t` 的 snapshot 字段中增加有效样本数和独立样本序号**

在：

```c
volatile uint16_t       hwSnapshotWrite;
volatile uint16_t       hwSnapshotDepth;
```

之后增加：

```c
volatile uint16_t       hwSnapshotValidCount;
volatile uint32_t       wSnapshotSampleIndex;
```

- [ ] **Step 2: 在 `SnapshotStart()` 中初始化新增状态**

修改：

```c
s_tWave.hwSnapshotWrite    = 0u;
s_tWave.hwSnapshotDepth    = depth;
```

为：

```c
s_tWave.hwSnapshotWrite       = 0u;
s_tWave.hwSnapshotValidCount  = 0u;
s_tWave.wSnapshotSampleIndex  = 0u;
s_tWave.hwSnapshotDepth       = depth;
```

- [ ] **Step 3: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected: 编译通过。

- [ ] **Step 4: 提交**

```bash
git add modus/src/mdebug/mwaveform.c
git commit -m "feat: add snapshot valid count and sample index state"
```

---

### Task 2: Snapshot 独立采样，不再复用 stream mask

**Files:**
- Modify: `modus/src/mdebug/mwaveform.c:618-633`

- [ ] **Step 1: 增加 `mwaveform_SnapshotCapture()`**

在 `mwaveform_SnapshotFeed()` 前增加：

```c
static void mwaveform_SnapshotCapture(void)
{
    uint8_t abMask[MASK_BYTES];

    memset(abMask, 0, MASK_BYTES);

    for (uint8_t i = 0; i < s_tWave.chCount; i++) {
        uint8_t bBit = (uint8_t)(1u << (i % 8));

        if (s_tWave.apvVariables[i] != NULL ||
            (s_tWave.abMask[i / 8] & bBit) != 0u) {
            abMask[i / 8] |= bBit;

            if (s_tWave.apvVariables[i] != NULL) {
                if (s_tWave.achVariableTypes[i] == MWAVEFORM_VAR_FLOAT) {
                    s_tWave.ahwSamples[i] = (int16_t)(
                        *(volatile float *)s_tWave.apvVariables[i] *
                        s_tWave.atChannels[i].fScale);
                } else {
                    s_tWave.ahwSamples[i] =
                        *(volatile int16_t *)s_tWave.apvVariables[i];
                }
            }
        }
    }

    if (s_tWave.hwSnapshotWrite >= s_tWave.hwSnapshotDepth) {
        s_tWave.hwSnapshotValidCount = s_tWave.hwSnapshotDepth;
    } else {
        s_tWave.hwSnapshotValidCount =
            (uint16_t)(s_tWave.hwSnapshotWrite + 1u);
    }

    uint16_t hwIdx = (uint16_t)(s_tWave.hwSnapshotWrite %
                                s_tWave.hwSnapshotDepth);
    s_tWave.atSnapshotSamples[hwIdx].wSampleIndex =
        s_tWave.wSnapshotSampleIndex++;
    memcpy(s_tWave.atSnapshotSamples[hwIdx].abMask, abMask, MASK_BYTES);
    memcpy(s_tWave.atSnapshotSamples[hwIdx].ahwSamples,
           s_tWave.ahwSamples,
           sizeof(s_tWave.ahwSamples[0]) * s_tWave.chCount);
    s_tWave.hwSnapshotWrite++;
}
```

- [ ] **Step 2: 将 `SnapshotFeed()` 改为调用独立采样**

替换 `SnapshotFeed()` 中以下三行：

```c
uint16_t hwIdx = (uint16_t)(s_tWave.hwSnapshotWrite %
                            s_tWave.hwSnapshotDepth);
s_tWave.atSnapshotSamples[hwIdx].wSampleIndex = s_tWave.wSampleIndex;
memcpy(s_tWave.atSnapshotSamples[hwIdx].abMask, s_tWave.abMask,
       MASK_BYTES);
```

为：

```c
mwaveform_SnapshotCapture();
```

最终函数为：

```c
static void mwaveform_SnapshotFeed(void)
{
    if (!s_tWave.bSnapshotArmed || s_tWave.bSnapshotReady ||
        !s_tWave.bIsRunning || s_tWave.hwSnapshotDepth == 0u) {
        return;
    }

    mwaveform_SnapshotCapture();
}
```

- [ ] **Step 3: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected: 编译通过。

- [ ] **Step 4: 提交**

```bash
git add modus/src/mdebug/mwaveform.c
git commit -m "feat: snapshot capture uses independent mask and sample index"
```

---

### Task 3: 触发和发送只使用有效样本数

**Files:**
- Modify: `modus/src/mdebug/mwaveform.c:635-649`
- Modify: `modus/src/mdebug/mwaveform.c:414-432`

- [ ] **Step 1: `SnapshotTrigger()` 拒绝无有效样本的触发**

在 `SnapshotTrigger()` 的条件中增加 `hwSnapshotValidCount == 0u`：

```c
if (!s_tWave.bSnapshotArmed || s_tWave.bSnapshotReady ||
    s_tWave.bSnapshotPending || s_tWave.hwSnapshotValidCount == 0u) {
    return MODUS_EBUSY;
}
```

- [ ] **Step 2: `Poll()` 使用 `hwSnapshotValidCount` 发送**

将：

```c
uint16_t hwStart = (s_tWave.hwSnapshotWrite >=
                    s_tWave.hwSnapshotDepth)
    ? (uint16_t)(s_tWave.hwSnapshotWrite %
                 s_tWave.hwSnapshotDepth)
    : 0u;
s_tWave.hwSnapshotFrameLen = mwaveform_pack_snapshot(
    s_tWave.achSnapshotFrame, s_tWave.atSnapshotSamples,
    s_tWave.chCount, s_tWave.hwSnapshotDepth, hwStart,
    s_tWave.hwSnapshotDepth, s_tWave.wSnapshotPeriodNs,
    s_tWave.wSnapshotId++);
```

替换为：

```c
uint16_t hwValid = s_tWave.hwSnapshotValidCount;
uint16_t hwStart = 0u;

if (hwValid > s_tWave.hwSnapshotDepth) {
    hwValid = s_tWave.hwSnapshotDepth;
}
if (s_tWave.hwSnapshotWrite >= s_tWave.hwSnapshotDepth) {
    hwStart = (uint16_t)(s_tWave.hwSnapshotWrite %
                         s_tWave.hwSnapshotDepth);
}
s_tWave.hwSnapshotFrameLen = mwaveform_pack_snapshot(
    s_tWave.achSnapshotFrame, s_tWave.atSnapshotSamples,
    s_tWave.chCount, s_tWave.hwSnapshotDepth, hwStart,
    hwValid, s_tWave.wSnapshotPeriodNs, s_tWave.wSnapshotId++);
```

- [ ] **Step 3: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected: 编译通过。

- [ ] **Step 4: 提交**

```bash
git add modus/src/mdebug/mwaveform.c
git commit -m "fix: snapshot sends only valid samples"
```

---

### Task 4: `SnapshotStop()` 不丢弃 pending 帧

**Files:**
- Modify: `modus/src/mdebug/mwaveform.c:652-659`

- [ ] **Step 1: 修改 `SnapshotStop()`**

替换：

```c
static void mwaveform_SnapshotStop(void)
{
    uint32_t wState = perfc_port_disable_global_interrupt();
    s_tWave.bSnapshotArmed   = false;
    s_tWave.bSnapshotReady   = false;
    s_tWave.bSnapshotPending = false;
    perfc_port_resume_global_interrupt(wState);
}
```

为：

```c
static void mwaveform_SnapshotStop(void)
{
    uint32_t wState = perfc_port_disable_global_interrupt();
    s_tWave.bSnapshotArmed = false;
    perfc_port_resume_global_interrupt(wState);
}
```

说明：只停止后续采集；已触发但尚未打包/发送的 `bSnapshotReady` 和 `bSnapshotPending` 保留，由 `Poll()` 继续完成发送。

- [ ] **Step 2: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected: 编译通过。

- [ ] **Step 3: 提交**

```bash
git add modus/src/mdebug/mwaveform.c
git commit -m "fix: snapshot stop preserves pending frame"
```

---

### Task 5: 暴露实际 snapshot 深度

**Files:**
- Modify: `modus/src/mdebug/mwaveform.h:94-119`
- Modify: `modus/src/mdebug/mwaveform.h:128-152`
- Modify: `modus/src/mdebug/mwaveform.c:678-704`
- Modify: `modus/src/mdebug/mwaveform.c:730-770`

- [ ] **Step 1: API 增加 `GetSnapshotDepth()`**

在 `mwaveform_api_t` 的 snapshot 区域增加：

```c
uint16_t (*GetSnapshotDepth)(void);
```

启用和禁用分支都增加。

- [ ] **Step 2: 实现查询函数**

在 `mwaveform_SnapshotIsArmed()` 后增加：

```c
static uint16_t mwaveform_GetSnapshotDepth(void)
{
    return s_tWave.hwSnapshotDepth;
}
```

禁用分支实现：

```c
static uint16_t dummy_GetSnapshotDepth(void) { return 0u; }
```

并在两个 `const mwaveform_api_t` 初始化中加入：

```c
.GetSnapshotDepth = mwaveform_GetSnapshotDepth,
```

或：

```c
.GetSnapshotDepth = dummy_GetSnapshotDepth,
```

- [ ] **Step 3: Shell `wave snap status` 输出实际深度和有效数**

将：

```c
} else if (strncmp(args, "status", 6) == 0) {
    MLOGF(I, "Snapshot armed: %d\r\n", mwaveform.SnapshotIsArmed());
```

替换为：

```c
} else if (strncmp(args, "status", 6) == 0) {
    MLOGF(I, "Snapshot armed: %d depth: %u valid: %u\r\n",
          mwaveform.SnapshotIsArmed(),
          mwaveform.GetSnapshotDepth(),
          s_tWave.hwSnapshotValidCount);
```

- [ ] **Step 4: 编译验证**

Run: `mingw32-make BUILD=debug-rel`

Expected: 编译通过。

- [ ] **Step 5: 提交**

```bash
git add modus/src/mdebug/mwaveform.c modus/src/mdebug/mwaveform.h
git commit -m "feat: expose actual snapshot depth and valid count"
```

---

### Task 6: MStudio 协议解析器回归测试

**Files:**
- Modify: `E:\Project\mstudio\tests\protocol_parser_test.cpp`

- [ ] **Step 1: 增加 snapshot 有效样本数测试**

在 `MakeSnapshot()` 之后增加：

```cpp
static std::vector<uint8_t> MakeSnapshotOneSample() {
    std::vector<uint8_t> f;
    f.push_back(0xAA); f.push_back(0x55); f.push_back(0xFA);
    f.push_back(1);
    f.push_back(2);
    PutU32(f, 30u);
    PutU16(f, 1);
    PutU32(f, 50000u);
    PutU32(f, 8u);

    f.push_back(0x01);
    PutU16(f, 42);

    PutU16(f, Crc16(f, 2));
    return f;
}
```

- [ ] **Step 2: 在 `main()` 中调用并断言**

在 `parser.Feed(MakeSnapshot(), samples);` 断言之后增加：

```cpp
    samples.clear();
    parser.Feed(MakeSnapshotOneSample(), samples);
    assert(samples.size() == 1);
    assert(samples[0].sample_index == 30u);
    assert(samples[0].ch_values.at(0) == 42.0f);
```

- [ ] **Step 3: 运行测试**

Run: `mingw32-make test`

Expected: `protocol_parser_test: OK`

- [ ] **Step 4: 提交**

```bash
git -C E:/Project/mstudio add tests/protocol_parser_test.cpp
git -C E:/Project/mstudio commit -m "test: parse one-sample snapshot frame"
```

---

### Task 7: 固件手工回归验证

**Files:**
- Verify: `build/template.elf`

- [ ] **Step 1: 烧录并启动 RTT**

Run: `.\make.bat auto`

Expected: OpenOCD 启动，MStudio Ch1 连接。

- [ ] **Step 2: 验证空快照不会发送无效样本**

```text
wave snap start 32 50000
wave snap trigger
wave snap status
```

Expected: `valid: 0`，且 `wave snap trigger` 返回 busy，不发送帧。

- [ ] **Step 3: 验证只写 1 点后触发只发 1 点**

在电机运行后执行：

```text
wave snap start 32 50000
wave snap trigger
wave snap status
```

若触发前只经过 1 个 ISR，Expected: `depth: 16 valid: 1`，MStudio snapshot 只解析到 1 个样本。

- [ ] **Step 4: 验证 Stop 不丢弃 pending**

```text
wave snap start 32 50000
wave snap trigger
wave snap stop
```

Expected: snapshot 帧仍然发送到 MStudio。

- [ ] **Step 5: 验证 20kHz snapshot 序号不重复**

通过 MStudio snapshot 面板观察样本序号，Expected: 序号单调递增，无重复。

---

### Task 8: 更新 mwaveform 文档

**Files:**
- Modify: `modus/docs/mdebug/mwaveform.md`

- [ ] **Step 1: 更新 snapshot 行为说明**

在 `docs/mdebug/mwaveform.md` 的 snapshot 章节补充：

```text
- Snapshot 使用独立采样节拍，不受 stream 分频影响。
- Snapshot 保存每个样本自己的 mask、数值和样本序号。
- 未写满时触发只发送已写入的有效样本。
- SnapshotStop 只停止后续采集，不丢弃已触发但尚未发送的快照。
- wave snap status 会显示实际 depth 和 valid 数。
```

- [ ] **Step 2: 提交**

```bash
git add modus/docs/mdebug/mwaveform.md
git commit -m "docs: describe snapshot correctness behavior"
```

---

## Self-Review

Spec 覆盖：

- 问题 1：Task 2。
- 问题 2：Task 2。
- 问题 3：Task 3。
- 问题 4：Task 4。
- 问题 5：Task 5。
- 回归验证：Task 6、Task 7。
- 文档同步：Task 8。

无占位符。API 名称在 Task 1-8 中保持一致：`hwSnapshotValidCount`、`wSnapshotSampleIndex`、`GetSnapshotDepth()`。

## 硬件实测补充

实测发现 `Step()` 在采样后会清除 `abMask`，若 `SnapshotCapture()` 只依赖 `abMask`，已 `Push` 的通道会漏进 snapshot。

最终实现增加 `abEverMask`：

- `Push()` / `PushRaw()` 设置 `abMask` 和 `abEverMask`。
- `AddVariable()` 设置 `abEverMask`。
- `SnapshotCapture()` 使用 `apvVariables[i] != NULL || abEverMask` 生成快照 mask。

这样 snapshot 不再受 `Step()` 与 `SnapshotFeed()` 调用顺序影响。
