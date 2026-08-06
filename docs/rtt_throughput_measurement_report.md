# RTT 吞吐实测报告

## 测试环境

- 目标：STM32G431
- 调试器：CMSIS-DAP
- OpenOCD：0.12.0
- RTT 轮询：`rtt polling_interval 1`
- Waveform RTT 缓冲：`MWAVEFORM_RTT_BUFFER_SIZE = 4096`
- 实时流率：10 kHz
- 通道：`TestSin`、`Iu`、`Iv`、`Iw` 以 10 kHz 进入 stream，`Angle` 以 1 kHz 进入 stream

## 测试方法

1. 烧录固件并启动 OpenOCD RTT 服务。
2. 启动电机，使 20 kHz ADC/控制中断持续调用 `mwaveform.Step()`。
3. 执行 `wave drop clear` 清零 MCU 侧计数。
4. 使用 `aitrace wave stat 60` 监控 60 秒 host 接收质量。
5. 结束后执行 `wave drop` 和 `wave rtt` 读取 MCU 侧统计。

## 实测结果

| 项目 | 结果 |
| --- | --- |
| MCU 侧 Drop | 0 |
| MCU 侧 Total | 611732 |
| MCU 侧丢帧率 | 0.0000% |
| RTT full | 9 |
| Host 有效帧数 | 9265 |
| Host CRC 错误 | 1 |
| Host seq_lost | 0 |
| 平均帧率 | 约 156 frames/s |
| 估算样本率 | 约 9,983 samples/s |

## 结论

- 当前 10 kHz 连续流在 4096 字节 RTT 缓冲 + OpenOCD 1ms 轮询下，MCU 侧可以做到 0 丢帧。
- 60 秒监控中出现 1 次 host 侧 CRC 错误，属于偶发 RTT 读取竞争，不是 MCU 丢帧。
- 20 kHz 连续流仍不承诺无损，高频微观观察应使用 snapshot。

## 推荐配置

```makefile
MWAVEFORM_RTT_BUFFER_SIZE = 4096
```

```text
rtt polling_interval 1
rtt server start 9090 0
rtt server start 9091 1
```
