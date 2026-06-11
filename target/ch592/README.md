# CH592 固件编译与命令行烧录指南

本项目已打通 CH592 芯片的 LLVM GCC/Clang 裸机编译，并整合了基于开源工具 `wchisp` 的命令行一键烧录流程。本指南介绍如何在 Windows 环境下配置环境并进行烧录部署。

---

## 1. 准备工具

在进行命令行烧录前，你需要准备以下两款工具：

1. **`wchisp.exe`**：开源社区（ch32-rs 团队）开发的 WCH ISP 命令行烧录工具。
   - **下载链接**：[ch32-rs/wchisp Releases](https://github.com/ch32-rs/wchisp/releases)
   - **部署方式**：下载 `wchisp-win-x64.zip` 并解压，将得到的 `wchisp.exe` 放入你已添加至环境变量（PATH）的目录中（例如 `D:\software\msys64\mingw64\bin` 目录）。
2. **`Zadig`**：Windows 通用 USB 驱动替换工具，用于将官方驱动切换为 `WinUSB` 驱动。
   - **下载链接**：[Zadig 官网](https://zadig.akeo.ie/) 或 [直接下载 zadig-2.9.exe](https://github.com/pbatard/libwdi/releases/download/v1.5.1/zadig-2.9.exe)
   - **说明**：免安装绿色版，下载后直接双击运行。

---

## 2. 驱动配置 (Windows 独有步骤)

由于沁恒官方工具 `WchIspStudio` 使用的是私有驱动（如 `CH375WDM.sys`），而 `wchisp` 依赖通用的 `WinUSB` 接口访问设备，因此必须使用 `Zadig` 切换驱动，该操作只需在首次使用时配置一次。

### 详细步骤：

1. **使芯片进入 BOOT（下载）模式**：
   - 按住开发板上的 **BOOT 按键**不要松开。
   - 按一下开发板上的 **RST（复位）按键**（或者重新插拔一下 USB 线）。
   - 松开 **BOOT 按键**。此时板载 MCU 即进入 USB ISP 烧录模式。
2. **运行 Zadig 并列出设备**：
   - 打开 `zadig-2.9.exe`。
   - 点击菜单栏的 **Options -> 勾选 List All Devices**。
3. **定位目标芯片设备**：
   - 在下拉菜单中找到代表 CH592 的 USB 设备（通常显示为 `USB Module`、`CH592` 或 `wlink`）。
   - **关键确认项**：确认下方框中的 **USB ID 必须为 `4348 55E0`**。
4. **替换驱动为 WinUSB**：
   - 在右侧绿色箭头指向的选择框中，选择 **`WinUSB (v6.1.7600.16385)`**。
   - 点击下方大按钮 **`Replace Driver`**（或 `Install Driver` / `Reinstall Driver`）。
   - 等待进度条走完，提示成功后即可关闭 Zadig。

---

## 3. 命令行编译与烧录

完成上述工具准备与驱动配置后，烧录流程便已完全打通。

### 1. 验证设备连接
确保开发板处于 **BOOT 模式**连接电脑，在命令行/终端中运行：
```powershell
wchisp probe
```
若能看到如下输出，说明连接与驱动正常：
```text
[INFO] Found 1 USB device
[INFO] Opening USB device #0
[INFO]         Device #0: CH592[0x9222]
```

### 2. 一键编译与烧录
我们已在项目的 [makefile](file:///e:/Project/modus_template/makefile) 与 [target.mk](file:///e:/Project/modus_template/target/ch592/target.mk) 中完成了烧录配方整合。

每次烧录前，确保板子处于 **BOOT 模式**，直接运行以下命令：
```powershell
# 清理旧的编译缓存（首次切换目标芯片或清理缓存时必须执行）
mingw32-make TARGET_CHIP=ch592 clean

# 编译并一键烧录
mingw32-make TARGET_CHIP=ch592 flash
```
项目将自动进行增量编译、生成 `.hex` 固件，并自动调用 `wchisp` 写入芯片、校验并向芯片发送复位信号运行。

---

## 4. 技术修改说明（维护注意）

本模板项目针对 CH592 裸机编译做了以下几点修改，后续修改工程配置时需注意：

1. **链接脚本 (LDSCRIPT)**：
   在 [CH592_FLASH.ld](file:///e:/Project/modus_template/target/ch592/CH592_FLASH.ld) 中，`.stack` 空间必须标记为 **`(NOLOAD)`**，例如：
   ```ld
   .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size (NOLOAD) :
   ```
   如果未加 `(NOLOAD)`，链接器会将其视为 `DATA` 数据段，生成包含全 0 的大体积 Hex/Bin 文件（因为 RAM 地址以 `0x20000000` 开头，会导致烧录工具误认为有 512MB 固件），引起烧录异常。
2. **编译器内置函数补丁**：
   在裸机 `-nostdlib` 下编译，一些 64 位移位和浮点比较函数缺少支持。我们在 [port_sys.c](file:///e:/Project/modus_template/peripheral/ch592/port_sys.c) 中手动补充了 `__lshrdi3`、`__ashldi3`、`__nesf2`、`__eqdf2` 桩函数，避免了链接时的未定义错误。
