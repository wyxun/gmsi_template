# AT32F421 注意事项

## OpenOCD

AT32 使用 ArteryTek 定制版 OpenOCD 0.11，与 STM32 的 OpenOCD 0.12 互相独立。

### 新电脑部署步骤

```bash
# 1. 安装 MSYS2（https://www.msys2.org/）
# 2. 在 MSYS2 中安装 OpenOCD v0.12
pacman -S mingw-w64-x86_64-openocd
#    得到 openocd.exe + share\openocd\scripts\

# 3. 下载 ArteryTek 定制版 openocd-at32 v0.11
#    下载地址：https://github.com/ArteryTek/openocd/releases
#    将 openocd-at32.exe 放到 MSYS2 的 bin 目录

# 4. 将 AT32 target cfg 文件复制到 OpenOCD 脚本目录
#    cfg 文件在 AT32 OpenOCD 源码仓库的 tcl/target/ 下：
#    https://github.com/ArteryTek/openocd/tree/master/tcl/target
#    将 at32f421xx.cfg 等 AT32 相关配置下载到：
#    cp at32f4* <MSYS2_ROOT>/msys64/mingw64/share/openocd/scripts/target/
```

### 文件清单

```
msys64\mingw64\bin\
├── openocd.exe        # v0.12，MSYS2 pacman 安装（供 STM32 使用）
└── openocd-at32.exe   # v0.11，手动下载（供 AT32 使用）

msys64\mingw64\share\openocd\scripts\target\
├── at32f421xx.cfg     # AT32 target 配置（从 AT32 OpenOCD 发布包复制）
└── ...                # 其他 AT32 系列 cfg
```

### 编译与烧录

target.mk 中已按芯片配置好 OpenOCD 路径，`make` 自动选择对应版本：
- `TARGET_CHIP=stm32g431` → 调用 `openocd.exe` (v0.12)
- `TARGET_CHIP=at32f421`  → 调用 `openocd-at32.exe` (v0.11)
