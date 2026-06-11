# CH592 调试与日志输出限制总结

在打通 CH592 的编译与 `wchisp` 命令行烧录后，当前阶段在尝试通过 `aitrace` / RTT 调试时遇到了连接失败的问题。本篇总结用于记录问题原因及后续处理方案。

---

## 1. 问题现象与原因分析

### 现象
运行 `.\tools\aitrace.exe shell list` 时提示：
```text
Failed to connect to RTT Ch0 (TCP 9090). Is OpenOCD running?
```
尝试通过 `D:\software\msys64\mingw64\bin\openocd.exe` 启动 OpenOCD 时报错：
```text
Error: The specified debug interface was not found (wlink)
```

### 根本原因
1. **调试探针与协议限制**：
   - SEGGER RTT 调试协议依赖物理调试探针（如 **WCH-LinkE**）通过 SWD/SDI 调试引脚实时读取 MCU 内部 RAM。
   - 当前板子仅通过一根 USB 线连接芯片的原生 USB 接口（用于 USB ISP 烧录）。芯片复位运行后，固件并未初始化 USB 堆栈，因此原生 USB 接口处于静默状态，无法提供任何调试通道。
2. **OpenOCD 驱动缺失**：
   - 沁恒的 RISC-V 芯片（QingKe 内核）调试依赖其专有的 `wlink` 驱动协议。
   - 电脑中安装的为 MSYS2 标准版 OpenOCD，未集成官方的 `wlink` 适配代码（该代码通常仅随官方 MounRiver Studio 安装包分发）。

---

## 2. 后续处理方案

后续如果需要查看固件日志（`MLOG` / 调试输出），可选择以下三种方案进行处理：

### 方案 A：重定向日志至物理串口（最便捷，无需调试器）

固件在 [port_sys.c](file:///e:/Project/modus_template/peripheral/ch592/port_sys.c) 中已经完成了 **UART0（波特率 115200）** 的初始化，但目前的 `MLOG` 日志仅定向到了 RTT。

#### 实施步骤：
1. **修改代码**：
   修改 [main.c](file:///e:/Project/modus_template/src/main.c) 中的 `user_trace_output` 函数，在输出 RTT 的同时向硬件串口发送数据：
   ```c
   void user_trace_output(const char *str)
   {
       SEGGER_RTT_WriteString(0, str);
       
       // 同时输出到 UART0 硬件串口
       if (str && HW.ptSerial) {
           MDI_Write(HW.ptSerial, (const uint8_t *)str, strlen(str));
       }
   }
   ```
2. **硬件接线**：
   - 准备一个常见的 USB 转串口模块（如 CH340、FT232、CP2102）。
   - 将串口模块的 **`RX`** 端连接到开发板的 **`PB7`**（TX0）引脚。
   - 将串口模块的 **`GND`** 端连接到开发板的 **`GND`**。
3. **日志监听**：
   将串口模块插入电脑后，会识别到对应的 `COMx` 端口。无需运行 OpenOCD，直接使用 `aitrace` 监听：
   ```powershell
   .\tools\aitrace.exe serial --port COMx --baud 115200 --duration 10
   ```

---

### 方案 B：使用 WCH-LinkE 硬件调试器进行 RTT 仿真

若后续需要进行单步断点调试（GDB）或使用 RTT：
1. **硬件接线**：准备一个 WCH-LinkE，将其 SDI、GND、3V3 引脚与开发板对应的调试引脚连接。
2. **获取官方定制版 OpenOCD**：
   - 安装官方 MounRiver Studio（MRS）集成开发环境。
   - 找到 MRS 安装路径下的 `toolchain\OpenOCD\bin\openocd.exe`，将其路径替换到主 Makefile 的 `OPENOCD_BIN` 中。
3. **启动调试**：
   - 运行 `.\make.bat rtt` 启动后台 OpenOCD RTT 服务。
   - 运行 `.\tools\aitrace.exe shell list` 等命令交互。

---

### 方案 C：固件中实现 USB CDC 虚拟串口

若希望**仅用一根原生 USB 线**就能读取日志：
- 需要在固件中实现 USB 设备栈及 CDC 虚拟串口协议（Virtual COM Port）。
- 程序运行后，原生 USB 接口会被主机识别为一个虚拟 COM 端口，通过常规串口助手或 `aitrace serial` 直接监听。此方案需要集成 USB 驱动库，开发工作量较大。
