# FOC Motor 对象封装与运行框架设计

日期：2026-07-18

## 1. 目标

把 `foc/` 作为可移植的 FOC 库维护，使应用层只通过 API 操作 motor
对象，不直接访问其运行状态、控制器、采样值、HAL 或校准数据。

本次设计必须同时满足：

- motor 对象高内聚、应用与实现低耦合；
- `motor_handle_t` 多实例、静态分配、无动态内存；
- 流程式操作继续使用 `fsm_rt_t` 状态机；
- 高频与低频实时控制保持确定性；
- 支持统一的位置源接口；
- 支持开环、传感器闭环和开环拖动后切入闭环；
- 保持足够的可观测性，不能因封装形成调试黑盒；
- 第一版保持精简，同时保留清晰的扩展位置。

## 2. 非目标

第一版不实现以下能力：

- 飞车启动；
- 多位置源优先级和自动故障接管；
- 闭环失败后自动退回开环；
- 运行中任意控制模式切换；
- 任意数量位置源的动态注册；
- 通用在线参数热更新框架；
- 覆盖所有危险测试的通用诊断状态机。

上述能力只保留明确扩展点，不进入本次核心状态机。

## 3. 总体架构

职责划分如下：

- `foc/app`：按钮、Shell、日志、产品策略和调度；
- `foc/motor`：生命周期、故障、安全约束、位置源选择和控制编排；
- `foc/control`：通用控制器算法；
- `foc/observer`：霍尔及无感观测算法和位置源适配；
- `foc/middleware`、`foc/modulation`：坐标变换和调制等通用算法；
- `foc/hal`：硬件能力抽象，不持有产品流程。

应用层不得执行角度推进、Clarke/Park、调制、电流重构、PWM 使能或
motor 状态迁移。

## 4. 不透明静态对象

应用继续静态声明对象：

```c
static motor_handle_t s_tMotor;
```

`motor_handle_t` 的公开定义只提供固定大小、正确对齐的私有存储，不公开
业务成员。内部实现类型仅存在于 motor 私有头文件。编译期断言必须验证：

- 私有实现大小不超过公开存储；
- 公开存储满足私有实现的对齐要求。

禁止使用 `malloc`。不得向外返回内部结构指针。

## 5. 正交运行模型

不能用一个“开环/闭环”状态同时表示生命周期、控制环和角度来源。

### 5.1 生命周期

```c
typedef enum {
    MOTOR_STATE_UNINITIALIZED = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_FAULT,
} motor_state_e;
```

### 5.2 控制模式

```c
typedef enum {
    MOTOR_CONTROL_VOLTAGE = 0,
    MOTOR_CONTROL_CURRENT,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
} motor_control_mode_e;
```

### 5.3 位置来源

位置来源由统一接口对象表达。空初始位置源表示使用 motor 内部开环角度
发生器；非空位置源可以是霍尔、编码器或无感观测器。

“开环控制 D/Q 电流”的准确语义是“开环角度 + D/Q 电流闭环”。电流闭环
仍必须执行相电流采样、坐标变换和 Id/Iq 控制。

## 6. 精简公共 API

```c
foc_result_t motor_Init(motor_handle_t *ptMotor,
                        const motor_config_t *ptConfig);
void motor_Reset(motor_handle_t *ptMotor);

foc_result_t motor_Start(motor_handle_t *ptMotor,
                         const motor_run_config_t *ptConfig);
foc_result_t motor_Stop(motor_handle_t *ptMotor);
fsm_rt_t motor_RunFSM(motor_handle_t *ptMotor);

foc_result_t motor_HighFrequencyStep(motor_handle_t *ptMotor);
foc_result_t motor_LowFrequencyStep(motor_handle_t *ptMotor);

foc_result_t motor_SetVoltageReference(motor_handle_t *ptMotor,
                                       q_type qD, q_type qQ);
foc_result_t motor_SetCurrentReference(motor_handle_t *ptMotor,
                                       q_type qD, q_type qQ);
foc_result_t motor_SetSpeedReference(motor_handle_t *ptMotor,
                                     q_type qSpeed);
foc_result_t motor_SetPositionReference(motor_handle_t *ptMotor,
                                        q_type qPosition);

foc_result_t motor_GetSnapshot(const motor_handle_t *ptMotor,
                               motor_snapshot_t *ptSnapshot);
void motor_EmergencyStop(motor_handle_t *ptMotor,
                         motor_fault_e eFault);
foc_result_t motor_ClearFault(motor_handle_t *ptMotor);
```

不提供 `motor_SetState()`。普通 API 只提交经过校验的命令，状态迁移由
`motor_RunFSM()` 决定。`motor_EmergencyStop()` 是唯一绕过 FSM、立即关闭
功率输出的公共接口。

## 7. 启动配置和使用场景

```c
typedef struct {
    motor_control_mode_e eControlMode;

    const foc_position_source_if_t *ptInitialPositionSource;
    const foc_position_source_if_t *ptTargetPositionSource;

    foc_angle_t tInitialAngle;
    q_type qOpenLoopSpeed;
    q_type qOpenLoopAcceleration;

    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    q_type qSpeedReference;
    q_type qPositionReference;
} motor_run_config_t;
```

组合规则：

- 初始源为空、目标源为空：持续使用开环角度；
- 初始源非空、目标源为空：直接使用传感器或观测器闭环；
- 初始源为空、目标源非空：开环拖动后平滑切入目标源；
- 初始源与目标源相同：直接闭环，不执行切换；
- 两个不同的非空源：第一版拒绝，避免暗含未实现的源间切换语义。

`motor_Start()` 必须验证控制器绑定、引用值范围、位置源接口和组合规则。

## 8. 统一位置源接口

霍尔、光栅、磁编码器、AB/ABZ 编码器及无感观测器统一实现：

```c
typedef struct {
    void *pContext;
    void (*fnReset)(void *pContext);
    foc_result_t (*fnStep)(
        void *pContext,
        const foc_position_source_input_t *ptInput,
        foc_position_source_output_t *ptOutput);
} foc_position_source_if_t;
```

统一输入包含电压、电流、时间戳和采样周期。物理传感器可忽略电压和电流，
观测器按需使用。

统一输出通过有效位表达可用能力，至少可包含：

- 电角度和电角速度；
- 单圈机械角和机械速度；
- 可选多圈计数；
- 置信度；
- Index/对齐状态；
- 位置源故障和时间戳。

机械传感器的极对数、方向、机械零位和电角度偏移由 motor 统一管理。
若位置源天然只提供电角度，则通过有效位明确标识，motor 不重复转换。

第一版 motor 同时只管理当前源和候选目标源，不实现注册表。

## 9. 状态机和实时数据流

motor 的私有启动子状态为：

```c
typedef enum {
    MOTOR_START_CALIBRATION = 0,
    MOTOR_START_DELAY,
    MOTOR_START_OPEN_LOOP,
    MOTOR_START_QUALIFY_SOURCE,
    MOTOR_START_BLEND_ANGLE,
    MOTOR_START_COMPLETE,
} motor_start_state_e;
```

`motor_RunFSM()` 处理校准、延时、启动、资格判定、切换、停止和故障流程。
它保留 `fsm_rt_t` 用法，不承担硬实时控制计算。

`motor_HighFrequencyStep()` 的固定数据流为：

```text
接管引用值快照
→ 电流采样
→ 更新当前和候选位置源
→ 选择或融合控制角度
→ Clarke/Park
→ 电流控制
→ 反 Park
→ 调制
→ PWM 输出
→ 更新诊断快照
```

`motor_LowFrequencyStep()` 负责位置环、速度环和引用值斜坡。

高频函数禁止阻塞、延时、文本日志和动态内存操作。多字段引用值通过内部
暂存和周期边界接管，避免主循环与 ISR 之间产生撕裂读取。

## 10. 开环到闭环切换

候选位置源只有在以下条件连续满足配置周期数后才能接管：

- 输出有效且时间戳未过期；
- 置信度达到阈值；
- 速度达到该位置源的最低有效速度；
- 与开环角度的最短环绕误差不超过阈值；
- 旋转方向一致。

切换在高频 Step 中融合开环角和候选角。融合系数从 0 平滑增长到 1，角度
误差必须使用归一化圆周上的最短路径。融合期间保持 D/Q 引用连续。

若候选源失效或切换超时，第一版默认急停，不自动退回开环。

当控制模式也需要从电流切换到速度或位置时，必须在角度接管完成后进行，
并预置外环控制器输出以避免 Iq 阶跃。第一版只实现启动配置要求的必要
无扰接管，不提供运行中的任意模式切换。

## 11. 可调试性

封装后的 motor 必须保持可观测，第一版提供：

1. `motor_GetSnapshot()`：一次获得状态、模式、活动位置源、角度、速度、
   D/Q 引用与反馈、D/Q 电压、三相电流、占空比、校准值、融合比例和故障；
2. 固定大小事件环：记录命令接受/拒绝、状态迁移、位置源有效性变化、切换
   开始/完成/超时和故障；
3. 应用侧 AITrace 适配：从快照导出稳定波形信号，motor 库不直接依赖
   AITrace。

事件记录不得在高频路径格式化文本。详细快照和事件环可由构建选项裁剪，
基础状态及故障查询始终保留。

固定占空比、相序检查等危险测试放入可选的
`foc/diagnostic/motor_diagnostic.c`，默认不进入量产构建。诊断操作必须验证
IDLE、无故障、超时及电流/电压/占空比限制，不能访问 motor 私有成员。

## 12. 故障原则

故障至少区分硬件、电流采样、位置源、位置源不一致、切换超时、控制计算、
非法命令、过流和超速。

- PWM、采样或控制关键路径失败时立即急停；
- 候选位置源资格不足时继续资格判定，超时后急停；
- 活动位置源失效时第一版急停；
- 非法但无危险的外部命令只拒绝，不强制进入故障；
- 只有 PWM 已关闭且处于 FAULT 时才能清除故障；
- ISR 内只置位和执行必要急停，日志在主循环输出。

## 13. 应用层改造

`foc_app_RunFSM()` 只驱动 `motor_RunFSM()` 并处理产品事件。按钮和 Shell 使用
`motor_Start()`、引用值 API、快照 API 和 `motor_Stop()`。

以下代码必须从 app 删除：

- 对 `tRt`、`tControl`、`tCurrent`、`tParams` 的直接访问；
- 电角度推进；
- `foc_ipark()`、`foc_svpwm()`；
- 电流采样与重构；
- PWM 使能和直接占空比写入；
- 电流校准细节访问。

README 和所有示例必须同步改为只使用公共 API。

## 14. 测试与验收

至少验证：

- 应用代码无法访问 motor 私有成员；
- 多 motor 实例完全独立且不使用动态内存；
- D/Q 电压开环；
- 开环角度下 D/Q 电流闭环；
- 霍尔和编码器直接闭环；
- 开环拖动后切入观测器；
- 0/1 圈边界的最短路径角度融合；
- 资格不足、源失效和切换超时；
- 切换期间 Id/Iq 和占空比连续性；
- 停止、重复启动、非法命令和故障清除；
- 位置源故障传播；
- 浮点和定点后端；
- README 公共示例可编译；
- 项目规定的目标构建通过。

## 15. 延后扩展项

以下 TODO 是有条件的扩展路线，不属于第一版验收范围。

### TODO: 飞车启动

触发条件：产品需要在转子已旋转时接管。实现思路是在 STARTING 中增加
速度和方向检测子流程，先资格判定位置源，再以当前估算速度预置角度发生器
与控制器，最后执行无扰接管。不得复用静止启动的强制对齐流程。

### TODO: 多位置源管理器

触发条件：同一 motor 需要三个及以上位置源，或需要自动后备。实现思路是
新增独立的 position manager，负责固定容量注册、优先级、健康度、交叉诊断
和切换请求；motor 仍只消费 manager 输出，不在生命周期 FSM 中堆叠源类型
分支。

### TODO: 闭环故障降级

触发条件：具体产品完成高速回切安全验证。实现思路是先同步开环发生器到
当前闭环角度和速度，再反向执行角度融合，并对电流、速度和母线电压设置
更严格限制。默认策略继续保持急停。

### TODO: 运行中控制模式切换

触发条件：产品确实需要 CURRENT/SPEED/POSITION 在线切换。实现思路是新增
独立 handover 模块，为每一级外环提供输出跟踪或积分预置，按从内到外启用、
从外到内退出，禁止直接修改模式枚举。

### TODO: 在线参数更新

触发条件：需要运行中整定。实现思路是按参数组提供版本化双缓冲，在明确的
周期边界原子接管；拓扑、接口绑定和存储布局类参数仍只允许 IDLE 更新。

### TODO: 扩展诊断 FSM

触发条件：独立诊断模块出现三种以上共享流程。实现思路是复用安全门控、
超时和限幅基础设施，将校准、固定 D/Q、电角度扫描等操作建成单独 FSM；
不得让正常 motor FSM 增加诊断专用状态。

