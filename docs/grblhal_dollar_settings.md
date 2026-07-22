# grblHAL $N 设置 — 完整宏定义默认值对照表

> 基于 `third_party/grblhal/core/config.h` (grblHAL core)
> 项目中 `grblhal_adapt/grblhal_config.h` 仅覆写部分关键宏，其余全部沿用下表默认值。

---

## 编译时选项（非 $N 设置）

| 宏定义 | 默认值 | 说明 |
|---|---|---|
| `N_AXIS` | `3` | 轴数 (3~8) |
| `N_SPINDLE` | `1` | 主轴数 (1~32) |
| `N_SYS_SPINDLE` | `1` | 同时活跃主轴数 (1~8) |
| `COMPATIBILITY_LEVEL` | `0` | 兼容等级 (0=GrblHAL, 1=Grbl, 10=严格) |
| `N_TOOLS` | `0` | 刀具表容量 (0~32) |
| `PLANNER_ADD_MOTION_MODE` | `Off` | 向 planner 传递运动模式 |
| `ENABLE_SPINDLE_LINEARIZATION` | `0` | 主轴 RPM 线性化 |
| `SPINDLE_NPWM_PIECES` | `4` | 主轴线性化分段数 (max 4) |
| `ENABLE_BACKLASH_COMPENSATION` | `Off` | 背隙补偿 |
| `ENABLE_ACCELERATION_PROFILES` | `Off` | GCode 切换加速度曲线 |
| `ENABLE_JERK_ACCELERATION` | `Off` | 3 阶加速度 (需 FPU) |
| `ENABLE_PATH_BLENDING` | `Off` | 路径混合 (实验性) |
| `SPINDLE_SYNC_ENABLE` | `Off` | 主轴同步 (G33/G76) |
| `NGC_EXPRESSIONS_ENABLE` | `Off` | NGC 表达式 |
| `NGC_PARAMETERS_ENABLE` | `On` | NGC 参数 |
| `NGC_N_ASSIGN_PARAMETERS_PER_BLOCK` | `10` | 每行最大参数数 |
| `LATHE_UVW_OPTION` | `Off` | 车床 UVW (非模态相对) |
| `WALL_PLOTTER` | `Off` | 壁挂绘图仪运动学 |
| `DELTA_ROBOT` | `Off` | Delta 运动学 |
| `POLAR_ROBOT` | `Off` | 极坐标运动学 |
| `COREXY` | `Off` | CoreXY 运动学 |
| `AXIS_REMAP_ABC2UVW` | `Off` | ABC 映射到 UVW |
| `BUILD_INFO` | `""` | `$I`/`$I+` 附加字符串 |
| `CHECK_MODE_DELAY` | `0` (ms) | 校验模式延时 |
| `DEBOUNCE_DELAY` | `40` (ms) | 输入去抖延时 |
| `MAX_TOOL_NUMBER` | `2147483647` | 最大刀具号 |
| `ACCELERATION_TICKS_PER_SECOND` | `100` | 加速度子系统时间分辨率 |
| `MINIMUM_JUNCTION_SPEED` | `0.0f` (mm/min) | 最小结点速度 |
| `MINIMUM_FEED_RATE` | `1.0f` (mm/min) | 最小进给速度 |
| `N_ARC_CORRECTION` | `12` | 圆弧修正迭代次数 (1~255) |
| `ARC_ANGULAR_TRAVEL_EPSILON` | `5E-7f` (rad) | 全圆判定阈值 |
| `BEZIER_MIN_STEP` | `0.002f` | 样条最小步长 |
| `BEZIER_MAX_STEP` | `0.1f` | 样条最大步长 |
| `BEZIER_SIGMA` | `0.1f` | 样条拟合系数 |
| `DWELL_TIME_STEP` | `50` (ms) | 驻留时间步长 (1~255) |
| `SEGMENT_BUFFER_SIZE` | `10` | 步进段缓冲区大小 |
| `SLEEP_DURATION` | `5.0f` (min) | 休眠等待时长 |
| `TOOLSETTER_RADIUS` | `5.0f` (mm) | 对刀仪识别半径 |
| `HOMING_AXIS_SEARCH_SCALAR` | `1.5f` | 回零搜索比例 (>1) |
| `HOMING_AXIS_LOCATE_SCALAR` | `10.0f` | 回零定位比例 (>1) |
| `REPORT_ECHO_LINE_RECEIVED` | `Off` | 回显接收行 (调试用) |
| `TOOL_LENGTH_OFFSET_AXIS` | `-1` | 刀长偏移轴 (-1=全部轴) |
| `SET_CHECK_MODE_PROBE_TO_START` | `Off` | 校验模式探测后回到起点 |
| `HARD_LIMIT_FORCE_STATE_CHECK` | `Off` | 硬限位强制状态检查 |
| `REPORT_OVERRIDE_REFRESH_BUSY_COUNT` | `20` | 忙时覆盖倍率刷新间隔 |
| `REPORT_OVERRIDE_REFRESH_IDLE_COUNT` | `10` | 闲时覆盖倍率刷新间隔 |
| `REPORT_WCO_REFRESH_BUSY_COUNT` | `30` | 忙时 WCO 刷新间隔 |
| `REPORT_WCO_REFRESH_IDLE_COUNT` | `10` | 闲时 WCO 刷新间隔 |

### NVS 操作开关（编译时）

| 宏定义 | 默认值 | 对应命令 |
|---|---|---|
| `NVSDATA_BUFFER_ENABLE` | `On` | NVS 缓冲区使能 |
| `ENABLE_RESTORE_NVS_WIPE_ALL` | `On` | `$RST=*` |
| `ENABLE_RESTORE_NVS_DEFAULT_SETTINGS` | `On` | `$RST=$` |
| `ENABLE_RESTORE_NVS_CLEAR_PARAMETERS` | `On` | `$RST=#` |
| `ENABLE_RESTORE_NVS_DRIVER_PARAMETERS` | `On` | `$RST=&` |
| `SETTINGS_RESTORE_DEFAULTS` | `On` | 版本升级时恢复默认设置 |
| `SETTINGS_RESTORE_PARAMETERS` | `On` | 版本升级时恢复参数 |
| `SETTINGS_RESTORE_STARTUP_LINES` | `On` | 版本升级时恢复启动行 |
| `SETTINGS_RESTORE_BUILD_INFO` | `On` | 版本升级时恢复 BUILD_INFO |
| `SETTINGS_RESTORE_DRIVER_PARAMETERS` | `On` | 版本升级时恢复驱动参数 |
| `DISABLE_BUILD_INFO_WRITE_COMMAND` | `Off` | 禁用 `$I=` 写命令 |

---

## $10 — StatusReportMask（状态报告掩码）

| 位 | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| bit 0 | `DEFAULT_REPORT_MACHINE_POSITION` | `On` | 报告位置 (含全部偏移) |
| bit 1 | `DEFAULT_REPORT_BUFFER_STATE` | `On` | 包含 `\|Bf:` 缓冲区状态 |
| bit 2 | `DEFAULT_REPORT_LINE_NUMBERS` | `On` | 包含 `\|Ln:` 行号 |
| bit 3 | `DEFAULT_REPORT_CURRENT_FEED_SPEED` | `On` | 包含 `\|FS:` 进给/转速 |
| bit 4 | `DEFAULT_REPORT_PIN_STATE` | `On` | 包含 `\|Pn:` 输入引脚状态 |
| bit 5 | `DEFAULT_REPORT_WORK_COORD_OFFSET` | `On` | 包含 `\|WCO:` 坐标偏移 |
| bit 6 | `DEFAULT_REPORT_OVERRIDES` | `On` | 包含覆盖倍率 |
| bit 7 | `DEFAULT_REPORT_PROBE_COORDINATES` | `On` | 探测成功后自动报告坐标 |
| bit 8 | `DEFAULT_REPORT_SYNC_ON_WCO_CHANGE` | `On` | WCO 改变时 flush planner 同步 |
| bit 9 | `DEFAULT_REPORT_PARSER_STATE` | `Off` | 自动报告解析器状态 ($G) |
| bit 10 | `DEFAULT_REPORT_ALARM_SUBSTATE` | `Off` | ALARM 状态附加子状态码 |
| bit 11 | `DEFAULT_REPORT_RUN_SUBSTATE` | `Off` | Run 状态附加子状态码 |
| bit 12 | `DEFAULT_REPORT_WHEN_HOMING` | `Off` | 回零时也发送状态报告 |
| bit 13 | `DEFAULT_REPORT_DISTANCE_TO_GO` | `Off` | 包含 `\|DTG:` 剩余距离 |

---

## 系统通用设置 (Group_General)

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $11 | `DEFAULT_JUNCTION_DEVIATION` | `0.01f` | mm | 结点偏差 |
| $12 | `DEFAULT_ARC_TOLERANCE` | `0.002f` | mm | 圆弧公差 |
| $13 | `DEFAULT_REPORT_INCHES` | `Off` | — | 英寸显示 (含 4 位小数) |
| $28 | `DEFAULT_G73_RETRACT` | `0.1f` | mm | G73 回退距离 |
| $32 | `DEFAULT_LASER_MODE` | `Off` | — | 激光模式 (与车床互斥) |
| $32 | `DEFAULT_LATHE_MODE` | `Off` | — | 车床模式 (与激光互斥) |
| $39 | `DEFAULT_LEGACY_RTCOMMANDS` | `On` | — | 传统 ASCII 实时命令 |
| $60 | `DEFAULT_RESET_OVERRIDES` | `Off` | — | 复位时清除覆盖倍率 |
| $62 | `DEFAULT_SLEEP_ENABLE` | `Off` | — | 休眠功能使能 |
| $63 | `DEFAULT_DISABLE_LASER_DURING_HOLD` | `On` | — | 进给保持时关闭激光 |
| $63 | `DEFAULT_RESTORE_AFTER_FEED_HOLD` | `On` | — | 进给保持恢复后还原主轴/冷却液 |
| $64 | `DEFAULT_FORCE_INITIALIZATION_ALARM` | `Off` | — | 上电强制 ALARM (需回零) |
| $384 | `DEFAULT_DISABLE_G92_PERSISTENCE` | `Off`/`On` | — | G92 断电保持 (level≤1 为 Off) |
| $398 | `DEFAULT_PLANNER_BUFFER_BLOCKS` | `100` | — | Planner 缓冲区块数 |
| $676 | `DEFAULT_HOMING_KEEP_STATUS_ON_RESET` | `Off` | — | 软复位后保留回零状态 (bit 6) |
| $676 | `DEFAULT_KEEP_OFFSETS_ON_RESET` | `Off` | — | 软复位后保留偏移 (bit 17) |
| $676 | `DEFAULT_KEEP_RAPIDS_OVR_ON_RESET` | `Off` | — | 软复位后保留快移覆盖 (bit 21) |
| $676 | `DEFAULT_KEEP_FEED_OVR_ON_RESET` | `Off` | — | 软复位后保留进给覆盖 (bit 22) |

---

## 控制信号 (Group_ControlSignals)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $14 | `DEFAULT_CONTROL_SIGNALS_INVERT_MASK` | `0` | 控制信号反相掩码 (bitmask) |
| $17 | `DEFAULT_DISABLE_CONTROL_PINS_PULL_UP_MASK` | `0` | 禁用控制脚内部上拉 (bitmask) |

信号位定义:

| 位 | 宏 | 值 |
|---|---|---|
| 0 | `SIGNALS_RESET_BIT` | `1` |
| 1 | `SIGNALS_FEEDHOLD_BIT` | `2` |
| 2 | `SIGNALS_CYCLESTART_BIT` | `4` |
| 3 | `SIGNALS_SAFETYDOOR_BIT` | `8` |
| 4 | `SIGNALS_BLOCKDELETE_BIT` | `16` |
| 5 | `SIGNALS_STOPDISABLE_BIT` | `32` |
| 6 | `SIGNALS_ESTOP_BIT` | `64` |
| 7 | `SIGNALS_PROBE_CONNECTED_BIT` | `128` |
| 8 | `SIGNALS_MOTOR_FAULT_BIT` | `256` |
| 9 | `SIGNALS_MOTOR_WARNING_BIT` | `512` |
| 10 | `SIGNALS_LIMITS_OVERRIDE_BIT` | `1024` |
| 11 | `SIGNALS_SINGLE_BLOCK_BIT` | `2048` |
| 12 | `SIGNALS_TLS_OVERTRAVEL_BIT` | `4096` |
| 13 | `SIGNALS_PROBE_OVERTRAVEL` | `8192` |
| 14 | `SIGNALS_PROBE_TRIGGERED_BIT` | `16384` |

---

## 限位 (Group_Limits)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $5 | `DEFAULT_LIMIT_SIGNALS_INVERT_MASK` | `0` | 限位信号反相 (axismask) |
| $18 | `DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK` | `0` | 禁用限位脚上拉 (axismask) |
| $20 | `DEFAULT_SOFT_LIMIT_ENABLE` | `Off` | 软限位使能 |
| $21 | `DEFAULT_HARD_LIMIT_ENABLE` | `Off` | 硬限位使能 |
| $21 | `DEFAULT_CHECK_LIMITS_AT_INIT` | `Off` | 上电时检查限位状态 |
| $21 | `DEFAULT_HARD_LIMITS_DISABLE_FOR_ROTARY` | `Off` | 旋转轴忽略硬限位 |
| $347 | `DEFAULT_DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT` | `5.0f` (%) | 双轴回零报警: 行程百分比 |
| $348 | `DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN` | `2.5f` (mm) | 双轴回零报警: 最小距离 |
| $348 | `DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX` | `25.0f` (mm) | 双轴回零报警: 最大距离 |

---

## 冷却液 (Group_Coolant)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $15 | `DEFAULT_INVERT_COOLANT_FLOOD_PIN` | `Off` | 反相 Flood 输出脚 |
| $15 | `DEFAULT_INVERT_COOLANT_MIST_PIN` | `Off` | 反相 Mist 输出脚 |
| $673 | `DEFAULT_COOLANT_ON_DELAY` | `0` (ms) | 冷却液开启延时 (500~20000) |

---

## 主轴 (Group_Spindle)

### 主轴 0 设置

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $9 (b0) | `DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED` | `Off` | — | 零速时关闭 Enable |
| $9 (b1) | `DEFAULT_PWM_SPINDLE_DISABLE_LASER_MODE` | `Off` | — | PWM 主轴禁用激光模式 |
| $9 (b2) | `DEFAULT_PWM_SPINDLE_ENABLE_RAMP` | `Off` | — | PWM 使能斜坡 |
| $9 (b3) | `DEFAULT_PWM_SPINDLE_IGNORE_DELAYS` | `Off` | — | PWM 忽略延时 |
| $16 | `DEFAULT_INVERT_SPINDLE_ENABLE_PIN` | `Off` | — | 反相 Enable 脚 |
| $16 | `DEFAULT_INVERT_SPINDLE_CCW_PIN` | `Off` | — | 反相方向脚 |
| $16 | `DEFAULT_INVERT_SPINDLE_PWM_PIN` | `Off` | — | 反相 PWM 脚 |
| $30 | `DEFAULT_SPINDLE_RPM_MAX` | `1000.0f` | rpm | 主轴最高转速 |
| $31 | `DEFAULT_SPINDLE_RPM_MIN` | `0.0f` | rpm | 主轴最低转速 |
| $33 | `DEFAULT_SPINDLE_PWM_FREQ` | `5000` | Hz | PWM 频率 |
| $34 | `DEFAULT_SPINDLE_PWM_OFF_VALUE` | `0.0f` | % | PWM 关断值 |
| $35 | `DEFAULT_SPINDLE_PWM_MIN_VALUE` | `0.0f` | 0~255 | PWM 最小占空比值 |
| $36 | `DEFAULT_SPINDLE_PWM_MAX_VALUE` | `100.0f` | % | PWM 最大占空比 |
| $38 | `DEFAULT_SPINDLE_PPR` | `0` | — | 主轴编码器脉冲/转 (0=禁用同步) |
| $340 | `DEFAULT_SPINDLE_AT_SPEED_TOLERANCE` | `0.0f` | % | 主轴达到转速公差 (0=不检查) |
| $394 | `DEFAULT_SPINDLE_ON_DELAY` | `0` | ms | 主轴启动延时 (500~20000) |
| $395 | `DEFAULT_SPINDLE` | `SPINDLE_PWM0` | — | 主轴类型编号 |
| $539 | `DEFAULT_SPINDLE_OFF_DELAY` | `0` | ms | 主轴关断延时 (500~20000) |

### 主轴 1 设置 (PWM1, 第二主轴)

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $709 | `DEFAULT_SPINDLE1_ENABLE_OFF_WITH_ZERO_SPEED` | `Off` | — | 零速关闭 Enable |
| $709 | `DEFAULT_SPINDLE1_DISABLE_LASER_MODE` | `Off` | — | 禁用激光模式 |
| $709 | `DEFAULT_SPINDLE1_ENABLE_RAMP` | `Off` | — | PWM 斜坡 |
| $709 | `DEFAULT_SPINDLE1_IGNORE_DELAYS` | `Off` | — | 忽略延时 |
| $716 | `DEFAULT_INVERT_SPINDLE1_ENABLE_PIN` | `Off` | — | 反相 Enable |
| $716 | `DEFAULT_INVERT_SPINDLE1_CCW_PIN` | `Off` | — | 反相方向 |
| $716 | `DEFAULT_INVERT_SPINDLE1_PWM_PIN` | `Off` | — | 反相 PWM |
| $730 | `DEFAULT_SPINDLE1_RPM_MAX` | `1000.0f` | rpm | 最高转速 |
| $731 | `DEFAULT_SPINDLE1_RPM_MIN` | `0.0f` | rpm | 最低转速 |
| $733 | `DEFAULT_SPINDLE1_PWM_FREQ` | `5000` | Hz | PWM 频率 |
| $734 | `DEFAULT_SPINDLE1_PWM_OFF_VALUE` | `0.0f` | % | 关断值 |
| $735 | `DEFAULT_SPINDLE1_PWM_MIN_VALUE` | `0.0f` | — | 最小占空比值 |
| $736 | `DEFAULT_SPINDLE1_PWM_MAX_VALUE` | `100.0f` | % | 最大占空比 |

### 闭环主轴 PID (Group_Spindle_ClosedLoop)

| 宏定义 | 默认值 | 说明 |
|---|---|---|
| `DEFAULT_SPINDLE_P_GAIN` | `1.0f` | P 增益 |
| `DEFAULT_SPINDLE_I_GAIN` | `0.01f` | I 增益 |
| `DEFAULT_SPINDLE_D_GAIN` | `0.0f` | D 增益 |
| `DEFAULT_SPINDLE_I_MAX` | `10.0f` | I 积分上限 |

### 主轴 RPM 线性化表 ($66~$69, 需 `ENABLE_SPINDLE_LINEARIZATION`)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $66 | `DEFAULT_RPM_POINT01` | `NAN` | 第1段 rpm 转折点 (NAN=禁用) |
| $66 | `DEFAULT_RPM_LINE_A1` | `3.197101e-03f` | 第1段斜率 |
| $66 | `DEFAULT_RPM_LINE_B1` | `-3.526076e-1f` | 第1段截距 |
| $67 | `DEFAULT_RPM_POINT12` | `NAN` | 第2段 rpm 转折点 |
| $67 | `DEFAULT_RPM_LINE_A2` | `1.722950e-2f` | 第2段斜率 |
| $67 | `DEFAULT_RPM_LINE_B2` | `1.0f` | 第2段截距 |
| $68 | `DEFAULT_RPM_POINT23` | `NAN` | 第3段 rpm 转折点 |
| $68 | `DEFAULT_RPM_LINE_A3` | `5.901518e-02f` | 第3段斜率 |
| $68 | `DEFAULT_RPM_LINE_B3` | `4.881851e+02f` | 第3段截距 |
| $69 | `DEFAULT_RPM_POINT34` | `NAN` | 第4段 rpm 转折点 |
| $69 | `DEFAULT_RPM_LINE_A4` | `1.203413e-01f` | 第4段斜率 |
| $69 | `DEFAULT_RPM_LINE_B4` | `1.151360e+03f` | 第4段截距 |

---

## 换刀 (Group_Toolchange)

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $341 | `DEFAULT_TOOLCHANGE_MODE` | `0` | — | 0=Normal, 1=Manual, 2=Manual@G59.3, 3=SemiAutomatic, 4=Ignore M6 |
| $342 | `DEFAULT_TOOLCHANGE_PROBING_DISTANCE` | `30` | mm | 换刀探测最大距离 (mode 3) |
| $343 | `DEFAULT_TOOLCHANGE_FEED_RATE` | `25.0f` | mm/min | 换刀探测进给速度 |
| $344 | `DEFAULT_TOOLCHANGE_SEEK_RATE` | `200.0f` | mm/min | 换刀寻找速度 |
| $345 | `DEFAULT_TOOLCHANGE_PULLOFF_RATE` | `200.0f` | mm/min | 换刀回退速度 |
| $346 (b0) | `DEFAULT_TOOLCHANGE_NO_RESTORE_POSITION` | `Off` | — | 不恢复换刀前位置 |
| $346 (b1) | `DEFAULT_TOOLCHANGE_AT_G30` | `Off` | — | 在 G30 位置换刀 |
| $346 (b2) | `DEFAULT_TOOLCHANGE_FAST_PROBE_PULLOFF` | `Off` | — | 探测快速回退 |
| $485 | `DEFAULT_PERSIST_TOOL` | `Off` | — | 断电保持刀具号 |
| $675 (b0) | `DEFAULT_MACRO_ATC_OPTION_EXECUTEM6T0` | `Off` | — | T0 也执行宏 |
| $675 (b1) | `DEFAULT_MACRO_ATC_ERROR_NO_MACRO` | `Off` | — | 无宏时报错 |
| $675 (b2) | `DEFAULT_MACRO_ATC_RANDOM_TOOLCHANGER` | `Off` | — | 随机刀库模式 |

---

## 回零 (Group_Homing)

### $22 — HomingEnable 位定义

| 位 | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| bit 0 | `DEFAULT_HOMING_ENABLE` | `Off` | 回零总使能 |
| bit 1 | `DEFAULT_HOMING_SINGLE_AXIS_COMMANDS` | `Off` | 启用单轴回零 ($HX/$HY/$HZ...) |
| bit 2 | `DEFAULT_HOMING_INIT_LOCK` | `Off` | 上电强制 ALARM 需回零 |
| bit 3 | `DEFAULT_HOMING_FORCE_SET_ORIGIN` | `Off` | 回零位置强行设为原点 |
| bit 4 | `DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES` | `Off` | 单轴并联双限位开关 |
| bit 5 | `DEFAULT_HOMING_ALLOW_MANUAL` | `Off` | 允许手动设零点 ($H 命令) |
| bit 6 | `DEFAULT_HOMING_OVERRIDE_LOCKS` | `Off` | 软复位覆盖回零锁定 |
| bit 8 | `DEFAULT_HOMING_USE_LIMIT_SWITCHES` | `Off` | 使用限位开关回零 |
| bit 10 | `DEFAULT_RUN_STARTUP_SCRIPTS_ONLY_ON_HOMED` | `Off` | 回零完成后才运行启动脚本 |

### 回零参数

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $23 | `DEFAULT_HOMING_DIR_MASK` | `0` | — | 回零方向 (axismask) |
| $24 | `DEFAULT_HOMING_FEED_RATE` | `25.0f` | mm/min | 回零慢速进给 |
| $25 | `DEFAULT_HOMING_SEEK_RATE` | `500.0f` | mm/min | 回零快速寻的 |
| $26 | `DEFAULT_HOMING_DEBOUNCE_DELAY` | `250` | ms | 回零去抖延时 (0~65535) |
| $27 | `DEFAULT_HOMING_PULLOFF` | `1.0f` | mm | 回零后回退距离 |
| $43 | `DEFAULT_N_HOMING_LOCATE_CYCLE` | `1` | — | 回零定位循环次数 (1~127) |
| $671 | `DEFAULT_HOME_SIGNALS_INVERT_MASK` | `0` | — | 回零信号反相 (axismask) |

### 回零循环定义 ($44 ~ $49)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $44 | `DEFAULT_HOMING_CYCLE_0` | `Z_AXIS_BIT` | 第一段: 抬 Z 轴清空 |
| $45 | `DEFAULT_HOMING_CYCLE_1` | `X_AXIS\|Y_AXIS` (或 corexy: `X_AXIS`) | 第二段 |
| $46 | `DEFAULT_HOMING_CYCLE_2` | `0` (或 corexy: `Y_AXIS`) | 第三段 |
| $47 | `DEFAULT_HOMING_CYCLE_3` | `0` (N_AXIS>3 时) | 第四段 |
| $48 | `DEFAULT_HOMING_CYCLE_4` | `0` (N_AXIS>4 时) | 第五段 |
| $49 | `DEFAULT_HOMING_CYCLE_5` | `0` (N_AXIS>5 时) | 第六段 |

轴位定义: `X_AXIS=1`, `Y_AXIS=2`, `Z_AXIS=4`, `A_AXIS=8`, `B_AXIS=16`, `C_AXIS=32`, `U_AXIS=64`, `V_AXIS=128`

---

## 探测 (Group_Probing)

| $N | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| $6 | `DEFAULT_PROBE_SIGNAL_INVERT` | `Off` | 探针信号反相 |
| $6 | `DEFAULT_TOOLSETTER_SIGNAL_INVERT` | `Off` | 对刀仪信号反相 |
| $19 | `DEFAULT_PROBE_SIGNAL_DISABLE_PULLUP` | `Off` | 禁用探针脚上拉 |
| $19 | `DEFAULT_TOOLSETTER_SIGNAL_DISABLE_PULLUP` | `Off` | 禁用对刀仪脚上拉 |
| $65 (b0) | `DEFAULT_ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES` | `Off` | 探测时允许进给倍率覆盖 |
| $65 (b1) | `DEFAULT_SOFT_LIMIT_PROBE_CYCLES` | `Off` | 探测时软限位生效 |

---

## 安全门 / 泊车 (Group_SafetyDoor)

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $41 (b0) | `DEFAULT_PARKING_ENABLE` | `Off` | — | 泊车功能使能 |
| $41 (b1) | `DEFAULT_DEACTIVATE_PARKING_UPON_INIT` | `Off` | — | 上电默认禁用泊车 (M56 P0) |
| $41 (b2) | `DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL` | `Off` | — | 泊车覆盖控制 |
| $42 | `DEFAULT_PARKING_AXIS` | `Z_AXIS` | — | 泊车运动轴 |
| $56 | `DEFAULT_PARKING_PULLOUT_INCREMENT` | `5.0f` | mm | 泊车拔出增量 |
| $57 | `DEFAULT_PARKING_PULLOUT_RATE` | `100.0f` | mm/min | 泊车拔出速度 |
| $58 | `DEFAULT_PARKING_TARGET` | `-5.0f` | mm | 泊车目标 (机床坐标) |
| $59 | `DEFAULT_PARKING_RATE` | `500.0f` | mm/min | 泊车快速移动速度 |
| $61 (b0) | `DEFAULT_DOOR_IGNORE_WHEN_IDLE` | `Off` | — | IDLE 时忽略开门信号 |
| $61 (b1) | `DEFAULT_DOOR_KEEP_COOLANT_ON` | `Off` | — | 开门时保留冷却液 |
| $392 | `DEFAULT_SAFETY_DOOR_SPINDLE_DELAY` | `4.0f` | s | 关门后主轴恢复延时 |
| $393 | `DEFAULT_SAFETY_DOOR_COOLANT_DELAY` | `1.0f` | s | 关门后冷却液恢复延时 |

---

## 步进 (Group_Stepper)

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $0 | `DEFAULT_STEP_PULSE_MICROSECONDS` | `5.0f` | µs | 步进脉冲宽度 |
| $1 | `DEFAULT_STEPPER_IDLE_LOCK_TIME` | `25` | ms | 空闲后锁住延时 (255=不关断) |
| $2 | `DEFAULT_STEP_SIGNALS_INVERT_MASK` | `0` | — | Step 信号反相 (axismask) |
| $3 | `DEFAULT_DIR_SIGNALS_INVERT_MASK` | `0` | — | Dir 信号反相 (axismask) |
| $4 | `DEFAULT_ENABLE_SIGNALS_INVERT_MASK` | `AXES_BITMASK` | — | Enable 信号反相 |
| $8 | `DEFAULT_GANGED_DIRECTION_INVERT_MASK` | `0` | — | 双轴方向反相 (axismask) |
| $29 | `DEFAULT_STEP_PULSE_DELAY` | `0.0f` | µs | 脉冲延时 |
| $37 | `DEFAULT_STEPPER_DEENERGIZE_MASK` | `0` | — | 不关断的轴 (axismask) |
| $680 | `DEFAULT_STEPPER_ENABLE_DELAY` | `0` | ms | Enable 延时 (0~250) |
| $742 | `DEFAULT_MOTOR_WARNING_SIGNALS_ENABLE` | `0` | — | 电机报警信号使能 (axismask) |
| $743 | `DEFAULT_MOTOR_WARNING_SIGNALS_INVERT` | `0` | — | 电机报警信号反相 (axismask) |
| $744 | `DEFAULT_MOTOR_FAULT_SIGNALS_ENABLE` | `0` | — | 电机故障信号使能 (axismask) |
| $745 | `DEFAULT_MOTOR_FAULT_SIGNALS_INVERT` | `0` | — | 电机故障信号反相 (axismask) |

---

## 各轴参数 ($10x ~ $14x, $22x)

### Steps/mm — $100 ~ $108

| $N | 宏定义 | 默认值 | 单位 |
|---|---|---|---|
| $100 | `DEFAULT_X_STEPS_PER_MM` | `250.0f` | steps/mm |
| $101 | `DEFAULT_Y_STEPS_PER_MM` | `250.0f` | steps/mm |
| $102 | `DEFAULT_Z_STEPS_PER_MM` | `250.0f` | steps/mm |
| $103 | `DEFAULT_A_STEPS_PER_MM` | `250.0f` | steps/mm |
| $104 | `DEFAULT_B_STEPS_PER_MM` | `250.0f` | steps/mm |
| $105 | `DEFAULT_C_STEPS_PER_MM` | `250.0f` | steps/mm |
| $106 | `DEFAULT_U_STEPS_PER_MM` | `250.0f` | steps/mm |
| $107 | `DEFAULT_V_STEPS_PER_MM` | `250.0f` | steps/mm |
| $108 | `DEFAULT_W_STEPS_PER_MM` | `250.0f` | steps/mm |

### 最大速度 — $110 ~ $118

| $N | 宏定义 | 默认值 | 单位 |
|---|---|---|---|
| $110 | `DEFAULT_X_MAX_RATE` | `500.0f` | mm/min |
| $111 | `DEFAULT_Y_MAX_RATE` | `500.0f` | mm/min |
| $112 | `DEFAULT_Z_MAX_RATE` | `500.0f` | mm/min |
| $113 | `DEFAULT_A_MAX_RATE` | `500.0f` | mm/min |
| $114 | `DEFAULT_B_MAX_RATE` | `500.0f` | mm/min |
| $115 | `DEFAULT_C_MAX_RATE` | `500.0f` | mm/min |
| $116 | `DEFAULT_U_MAX_RATE` | `500.0f` | mm/min |
| $117 | `DEFAULT_V_MAX_RATE` | `500.0f` | mm/min |
| $118 | `DEFAULT_W_MAX_RATE` | `500.0f` | mm/min |

### 加速度 — $120 ~ $128

| $N | 宏定义 | 默认值 | 单位 |
|---|---|---|---|
| $120 | `DEFAULT_X_ACCELERATION` | `10.0f` | mm/s² |
| $121 | `DEFAULT_Y_ACCELERATION` | `10.0f` | mm/s² |
| $122 | `DEFAULT_Z_ACCELERATION` | `10.0f` | mm/s² |
| $123 | `DEFAULT_A_ACCELERATION` | `10.0f` | mm/s² |
| $124 | `DEFAULT_B_ACCELERATION` | `10.0f` | mm/s² |
| $125 | `DEFAULT_C_ACCELERATION` | `10.0f` | mm/s² |
| $126 | `DEFAULT_U_ACCELERATION` | `10.0f` | mm/s² |
| $127 | `DEFAULT_V_ACCELERATION` | `10.0f` | mm/s² |
| $128 | `DEFAULT_W_ACCELERATION` | `10.0f` | mm/s² |

### 最大行程 — $130 ~ $138

| $N | 宏定义 | 默认值 | 单位 |
|---|---|---|---|
| $130 | `DEFAULT_X_MAX_TRAVEL` | `200.0f` | mm |
| $131 | `DEFAULT_Y_MAX_TRAVEL` | `200.0f` | mm |
| $132 | `DEFAULT_Z_MAX_TRAVEL` | `200.0f` | mm |
| $133 | `DEFAULT_A_MAX_TRAVEL` | `200.0f` | mm |
| $134 | `DEFAULT_B_MAX_TRAVEL` | `200.0f` | mm |
| $135 | `DEFAULT_C_MAX_TRAVEL` | `200.0f` | mm |
| $136 | `DEFAULT_U_MAX_TRAVEL` | `200.0f` | mm |
| $137 | `DEFAULT_V_MAX_TRAVEL` | `200.0f` | mm |
| $138 | `DEFAULT_W_MAX_TRAVEL` | `200.0f` | mm |

### 步进电流 — $140 ~ $148

| $N | 宏定义 | 默认值 | 单位 |
|---|---|---|---|
| $140 | `DEFAULT_X_CURRENT` | `500.0f` | mA RMS |
| $141 | `DEFAULT_Y_CURRENT` | `500.0f` | mA RMS |
| $142 | `DEFAULT_Z_CURRENT` | `500.0f` | mA RMS |
| $143 | `DEFAULT_A_CURRENT` | `500.0f` | mA RMS |
| $144 | `DEFAULT_B_CURRENT` | `500.0f` | mA RMS |
| $145 | `DEFAULT_C_CURRENT` | `500.0f` | mA RMS |
| $146 | `DEFAULT_U_CURRENT` | `500.0f` | mA RMS |
| $147 | `DEFAULT_V_CURRENT` | `500.0f` | mA RMS |
| $148 | `DEFAULT_W_CURRENT` | `500.0f` | mA RMS |

### Jerk — $220 ~ $228

| $N | 宏定义 | 默认值 (推导) | 单位 |
|---|---|---|---|
| $220 | `DEFAULT_X_JERK` | `X_ACCEL * 10` = 100.0f | mm/s³ |
| $221 | `DEFAULT_Y_JERK` | `Y_ACCEL * 10` = 100.0f | mm/s³ |
| $222 | `DEFAULT_Z_JERK` | `Z_ACCEL * 10` = 100.0f | mm/s³ |
| $223 | `DEFAULT_A_JERK` | `A_ACCEL * 10` = 100.0f | mm/s³ |
| $224 | `DEFAULT_B_JERK` | `B_ACCEL * 10` = 100.0f | mm/s³ |
| $225 | `DEFAULT_C_JERK` | `C_ACCEL * 10` = 100.0f | mm/s³ |
| $226 | `DEFAULT_U_JERK` | `U_ACCEL * 10` = 100.0f | mm/s³ |
| $227 | `DEFAULT_V_JERK` | `V_ACCEL * 10` = 100.0f | mm/s³ |
| $228 | `DEFAULT_W_JERK` | `W_ACCEL * 10` = 100.0f | mm/s³ |

> Jerk 默认值 = 对应轴加速度 × 10.0

---

## 其他设置

| $N | 宏定义 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| $40 | `DEFAULT_JOG_LIMIT_ENABLE` | `Off` | — | Jog 运动软限位 (需回零 + 正确行程) |
| $374 | `DEFAULT_MODBUS_STREAM_BAUD` | `3` | — | ModBus 波特率 (0=2400,1=4800,2=9600,**3=19200**,4=38400,5=115200) |
| $376 | `DEFAULT_AXIS_ROTATIONAL_MASK` | 自动计算 | — | 旋转轴标识 (axismask, 由轴字母 A/B/C 决定) |
| $481 | `DEFAULT_AUTOREPORT_INTERVAL` | `0` | ms | 自动状态报告间隔 (100~1000, 0=禁用) |
| $482 | `DEFAULT_TIMEZONE_OFFSET` | `0.0f` | h | UTC 时区偏移 (-12.0 ~ 12.0) |
| $484 | `DEFAULT_NO_UNLOCK_AFTER_ESTOP` | `Off` | — | E-Stop 后无需 $X 解锁 (逻辑反相) |
| $536 | `DEFAULT_RGB_STRIP0_LENGTH` | `0` | — | NeoPixel/WS2812 灯带 0 LED 数 |
| $537 | `DEFAULT_RGB_STRIP1_LENGTH` | `0` | — | NeoPixel/WS2812 灯带 1 LED 数 |
| $538 | `DEFAULT_AXIS_ROTARY_WRAP_MASK` | `0` | — | 旋转轴 wrap (axismask) |
| $681 | `DEFAULT_MODBUS_STREAM_DATA_BITS` | `0` | — | ModBus 数据位 (0=8, 1=7) |
| $681 | `DEFAULT_MODBUS_STREAM_STOP_BITS` | `0` | — | ModBus 停止位 (0=1, 1=1.5, 2=2, 3=0.5) |
| $681 | `DEFAULT_MODBUS_STREAM_PARITY` | `0` | — | ModBus 校验 (0=None, 1=Even, 2=Odd) |

### 文件系统选项 — $650

| 位 | 宏定义 | 默认值 | 说明 |
|---|---|---|---|
| bit 0 | `DEFAULT_FS_SD_AUTOMOUNT` | `Off` | SD 卡自动挂载 |
| bit 1 | `DEFAULT_FS_LITTLEFS_HIDDEN` | `Off` | 隐藏 LittleFS |
| bit 2 | `DEFAULT_FS_HIERACHICAL_LISTING` | `Off` | 目录分级列表 |

---

## 项目覆写 (grblhal_adapt/grblhal_config.h)

项目中 **[grblhal_config.h](../grblhal_adapt/grblhal_config.h)** 对以下宏进行了覆写，覆写后的值优先生效：

### AT32F407 (`GRBLHAL_FULL_FEATURES`)

| 宏 | 项目中覆写值 | config.h 默认值 |
|---|---|---|
| `COMPATIBILITY_LEVEL` | `10` | `0` |
| `N_AXIS` | `3` | `3` (等同) |
| `HOMING_CYCLE_0` | `(1 << 0)` = Z | `Z_AXIS_BIT` (等同) |
| `HOMING_CYCLE_1` | `(1 << 1)` = Y | `X\|Y` (**不同!**) |
| `HOMING_CYCLE_2` | `(1 << 2)` = X | `0` (**不同!**) |
| `RX_BUFFER_SIZE` | `256` | — |
| `TX_BUFFER_SIZE` | `256` | — |
| `BLOCK_BUFFER_SIZE` | `36` | — |
| `NVS_BUFFER_SIZE` | `512` | — |
| `SPINDLE_ENABLE` | `(1 << SPINDLE_PWM0)` | 无此宏 (driver level) |
| `COOLANT_ENABLE` | `1` | 无此宏 (driver level) |
| `PROBE_ENABLE` | `1` | 无此宏 (driver level) |
| `LIMITS_ENABLE` | `1` | 无此宏 (driver level) |
| `CONTROL_ENABLE` | `1` | 无此宏 (driver level) |
| `ENABLE_SAFETY_DOOR_INPUT_PIN` | `1` | 无此宏 (driver level) |
| `DEFAULT_SPINDLE_ON_DELAY` | `3000` | `0` (**不同!**) |
| `DEFAULT_SPINDLE_OFF_DELAY` | `2000` | `0` (**不同!**) |
| `NGC_PARAMETERS_ENABLE` | `1` | `On` (等同) |
| `NVSDATA_BUFFER_ENABLE` | `1` | `On` (等同) |
| `ENABLE_RESTORE_NVS_WIPE_ALL` | `1` | `On` (等同) |
| `ENABLE_RESTORE_NVS_DEFAULT_SETTINGS` | `1` | `On` (等同) |
| `ENABLE_RESTORE_NVS_CLEAR_PARAMETERS` | `1` | `On` (等同) |
| `ENABLE_RESTORE_NVS_DRIVER_PARAMETERS` | `1` | `On` (等同) |
| `SETTINGS_RESTORE_DEFAULTS` | `1` | `On` (等同) |
| `SETTINGS_RESTORE_PARAMETERS` | `1` | `On` (等同) |
| `SETTINGS_RESTORE_STARTUP_LINES` | `1` | `On` (等同) |
| `SETTINGS_RESTORE_BUILD_INFO` | `1` | `On` (等同) |
| `SETTINGS_RESTORE_DRIVER_PARAMETERS` | `1` | `On` (等同) |
| `SDCARD_ENABLE` | `0` | 无此宏 (driver level) |
| `EEPROM_ENABLE` | `0` | 无此宏 (driver level) |
| `BLUETOOTH_ENABLE` | `0` | 无此宏 (driver level) |
| `ETHERNET_ENABLE` | `0` | 无此宏 (driver level) |
| `WIFI_ENABLE` | `0` | 无此宏 (driver level) |
| `ENABLE_SOFTWARE_DEBOUNCE` | `0` | 无此宏 (driver level) |

### STM32G431 (非 FULL_FEATURES)

| 宏 | 项目中覆写值 | config.h 默认值 |
|---|---|---|
| `SPINDLE_ENABLE` | `0` | 无此宏 (driver level) |
| `COOLANT_ENABLE` | `0` | 同上 |
| `PROBE_ENABLE` | `0` | 同上 |
| `LIMITS_ENABLE` | `0` | 同上 |
| `CONTROL_ENABLE` | `0` | 同上 |
| `NGC_PARAMETERS_ENABLE` | `0` | `On` (**不同!**) |
| `NVSDATA_BUFFER_ENABLE` | `0` | `On` (**不同!**) |
| `NO_SAFETY_DOOR_SUPPORT` | `1` | — |
| `NO_SETTINGS_DESCRIPTIONS` | `1` | — |
| `NO_ERROR_DESCRIPTIONS` | `1` | — |
| `NO_TOOL_CHANGE_SUPPORT` | `1` | — |

> **关键差异**: AT32F407 HOMING_CYCLE 改变了回零顺序，DEFAULT_SPINDLE_ON/OFF_DELAY 从 0ms 改为 3000/2000ms；STM32G431 则关闭了 NGC 和 NVS 以节省 flash。
