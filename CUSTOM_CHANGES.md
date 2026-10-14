# Betaflight 自主飞行代码说明

本文说明当前自主飞行实现的代码位置和功能。行号以当前工作区代码为准。

代码来源标识：

- `[自定义新增]`：项目自行添加的文件或独立模块，Betaflight 官方基础代码中不存在。
- `[官方代码修改]`：在 Betaflight 官方文件中加入或调整的集成逻辑。

## 涉及文件

与其他 Betaflight 代码融合时，自定义新增文件可以直接加入；官方代码修改文件需要逐个合并差异。

### [自定义新增] 代码与工具

- `src/main/flight/autonomous_mode.c`
- `src/main/flight/autonomous_mode.h`
- `gazebo_pose.py`
- `sitl_rc.py`
- `tx16s_sitl_rc.py`

### [官方代码修改] 需要合并差异

- `mk/source.mk`
- `src/main/fc/core.c`
- `src/main/fc/core.h`
- `src/main/fc/rc.c`
- `src/main/fc/rc_controls.c`
- `src/main/fc/tasks.c`
- `src/main/flight/failsafe.c`
- `src/main/flight/failsafe.h`
- `src/main/flight/mixer.c`
- `src/main/flight/pid.c`
- `src/main/msp/msp_box.c`
- `src/platform/SIMULATOR/sitl.c`
- `src/platform/SIMULATOR/target/SITL/target.h`
- `src/test/unit/flight_failsafe_unittest.cc`

### 说明文档

- `CUSTOM_CHANGES.md`

## 功能概览

自主流程使用 ARM、USER1、USER2 和 USER3 四类开关输入：

- ARM 是飞手允许自主解锁的许可，同时也是 USER3 投掷流程的触发边沿输入。
- USER2 上升沿作为一次性触发，要求 ARM 已打开且 USER1 能读到有效位置；触发后请求 synthetic failsafe 接收链路锁定。failsafe 完成处置、飞机解除 ARM 并恢复有效 RX 后，再等待代码头部定义的 1 秒延时，进入自主解锁和自由落体检测。当前 USER2 高电平可以继续完成本次流程；下一次触发前必须重新检测到低电平。
- USER3 是投掷机专用的备用触发路径：USER2 为低、USER3 为高且 ARM 出现上升沿时触发自主流程，同样要求 USER1 初始位置有效。USER3 触发边沿后释放 synthetic RX 锁，等待自由落体；USER3 高电平期间禁止普通 ARM 解锁。
- USER1 用于读取三段拨杆初始位置并检测位置变化。自主流程开始时记录初始位置，运行期间位置持续变化 30 ms 会中止自主流程。

USER1 需要在地面站的 Modes 页面配置，以便固件从 `modeActivationCondition` 取得对应的 AUX 通道索引。配置的激活范围只用于让该映射有效；代码随后直接读取该通道的原始 PWM：

- 小于 1300：`AUTONOMOUS_USER1_POSITION_LOW`
- 1300 至 1700：`AUTONOMOUS_USER1_POSITION_MIDDLE`
- 大于等于 1800：`AUTONOMOUS_USER1_POSITION_HIGH`
- 1701 至 1799：`AUTONOMOUS_USER1_POSITION_INVALID`

三个 USER1 位置不会直接启用 `ALT_HOLD_MODE` 或 `POS_HOLD_MODE`。自主状态机进入 `AUTONOMOUS_MODE_READY` 后，高度保持和最终位置保持由状态机自身请求，与 USER1 当前档位无关。

自主状态按以下顺序流转：

```text
IDLE
  -> WAITING_FOR_ARM
  -> ACCEL_WAITING
  -> FLIP（机体倒置时）
  -> HOVER_ENTRY
  -> BRAKING
  -> READY
```

机体未倒置时，`ACCEL_WAITING` 可直接进入 `HOVER_ENTRY`。`HOVER_ENTRY` 使用文件头部定义的入口油门比例；当姿态、角速度和 Z 高度估计连续稳定后进入 `BRAKING`。`BRAKING` 从实测位置和速度启动 jerk/加速度受限的竖直、水平减速轨迹，高度闭环在此阶段接管油门；XY 估计无效时只执行定高减速。速度、加速度和姿态稳定约 250 ms 后进入 `READY`，锁定最终高度，XY 有效时锁定最终位置并申请最终 `POS_HOLD`。自主翻转最多持续 1 秒，结束阈值和油门闭环参数均在 `autonomous_mode.c` 文件头部定义。

## 自主状态机

### [自定义新增] `src/main/flight/autonomous_mode.h`

- 第 8-13 行：`autonomousUser1Position_e`，定义 USER1 的低、中、高和无效位置。
- 第 15-23 行：`autonomousModeState_e`，定义空闲、等待 ARM、自由落体检测、翻转、悬停入口、自主减速和 READY 状态。
- 第 25-50 行：声明自主流程更新、解锁接管、油门覆盖、翻转输出、控制模式请求、状态读取和中止接口。
- 第 48-50 行：USER1 高度/位置保持兼容接口固定返回 `false`。

### [自定义新增] `src/main/flight/autonomous_mode.c`

- 第 25-50 行：自由落体、悬停、USER1 防抖、USER2 延时、翻转闭环和自主减速轨迹参数。
- 第 81 行：`autonomousModeResetFreefallDetection()`，复位自由落体候选检测。
- 第 89 行：`autonomousModeResetHoverStability()`，复位悬停稳定候选检测。
- 第 97 行：`autonomousModeReadUser1Position()`，通过 BOXUSER1 配置取得 AUX 通道并读取原始拨杆位置。
- 第 122 行：`autonomousModeResetToIdle()`，复位自主状态机、检测状态、翻转输出和 USER1 位置状态；触发开关仍为高电平时保留对应锁定。
- 第 143 行：`autonomousModeStartAccelerationDetection()`，进入自由落体检测阶段。
- 第 162 行：`autonomousModeAbort()`，中止当前流程并回到空闲状态。
- 第 168-198 行：提供自主解锁接管、飞手输入屏蔽、油门覆盖和自主油门 PWM 输出。
- 第 202-220 行：提供自主翻转状态、角速度和油门输出。
- 第 224-330 行：提供 ANGLE、高度保持、位置保持、飞手保持模式屏蔽请求和自主减速轨迹目标接口。
- 第 253-274 行：读取自主状态和 USER1 位置；USER1 高度/位置保持兼容接口固定返回 `false`。
- 第 350 行：`autonomousModeUpdate()`，处理 USER2/USER3/ARM 边沿、failsafe 恢复、USER1 位置变化、自由落体检测、自动解锁、翻转、入口调平、BRAKING 轨迹和最终保持状态流转；SITL 下每秒输出姿态和加速度诊断。
- 第 545 行：`autonomousModeResetFlip()`，复位翻转轴、方向、角速度和油门控制量。
- 第 557 行：`autonomousModeStartFlip()`，根据重力投影选择翻转轴和方向，并复位 PID 积分。
- 第 587 行：`autonomousModeUpdateFlipControl()`，根据倾角和实际角速度闭环更新翻转角速度及油门，并限制油门变化速率。

## 飞控集成点

### [官方代码修改] 构建与调度

- `mk/source.mk:169`：将 `flight/autonomous_mode.c` 编入固件。
- `src/main/fc/tasks.c:128`：`taskMain()` 周期调用 `autonomousModeUpdate()`；原先在任务中单独处理 USER1/USER2/USER3 边沿的逻辑已移入自主状态机。

### [官方代码修改] 解锁与飞行模式

- `src/main/fc/core.c:567`：`tryArmInternal()` 合并普通和自主解锁检查；自主解锁额外忽略 ANGLE、THROTTLE 和 ARM_SWITCH 三类阻止标志。
- `src/main/fc/core.c:708`：`tryArm()` 保留普通飞手解锁入口。
- `src/main/fc/core.c:715`：`tryArmAutonomous()` 提供自主解锁入口并返回结果。
- `src/main/fc/core.h:80`：声明 `tryArmAutonomous()`。
- `src/main/fc/core.c:850`：`processRx()` 在自主翻转期间保持 PID 稳定化。
- `src/main/fc/core.c:992`：`processRxModes()` 应用飞手输入屏蔽及 ANGLE、高度和位置控制请求；USER1 不再直接启用高度或位置保持。
- `src/main/fc/core.c:1345`：`subTaskPidController()` 在自主翻转期间抑制失控起飞保护误触发。

### [官方代码修改] RC 输入与油门

- `src/main/fc/rc.c:108`：`getFeedforward()` 在自主翻转期间关闭飞手前馈。
- `src/main/fc/rc.c:623`：`processRcCommand()` 在翻转期间输出自主角速度，其他自主阶段屏蔽飞手姿态输入。
- `src/main/fc/rc.c:721`：`updateRcCommands()` 在翻转期间输出闭环翻转油门，其他自主阶段输出悬停或入口油门。
- `src/main/fc/rc_controls.c:148`：`processRcStickPositions()` 在自主流程接管或 USER3 高电平期间禁止普通 ARM 解锁，并保留 failsafe 解锁阻止状态。

### [官方代码修改] Failsafe

- `src/main/flight/failsafe.c:106`：`failsafeReset()` 复位自主链路锁定请求和触发锁存。
- `src/main/flight/failsafe.c:153`：`failsafeIsActive()` 报告真实、BOXFAILSAFE 或 USER2 自主锁定触发的 failsafe 状态。
- `src/main/flight/failsafe.c:158`：`failsafeIsAutonomousLockActive()` 查询自主 synthetic RX 锁。
- `src/main/flight/failsafe.c:164`：`failsafeRequestAutonomousLock()` 设置或解除自主链路锁定；USER2 触发的锁在 failsafe 完成解锁后、飞机已解除 ARM 时自动释放。
- `src/main/flight/failsafe.c:217`：`failsafeIsReceivingRxData()` 在自主锁定期间报告链路不可用。
- `src/main/flight/failsafe.c:228`、`:234`：RX 挂起/恢复回调在自主锁定期间保持链路 down。
- `src/main/flight/failsafe.c:242`：有效 RX 数据在自主锁定期间不立即恢复正常链路。
- `src/main/flight/failsafe.c:344`：`failsafeUpdateState()` 将自主锁定纳入 failsafe 触发、解锁和 RX 恢复流程。
- `src/main/flight/failsafe.c:632`：在 `DEBUG_FAILSAFE` 中输出自主锁定状态和解锁阻止标志。
- `src/main/flight/failsafe.h:110`：声明自主锁定查询和请求接口。

### [官方代码修改] 翻转保护

- `src/main/flight/mixer.c:678`：`mixTable()` 在自主翻转期间强制启用空中模式补偿。
- `src/main/flight/pid.c:651`：`handleCrashRecovery()` 在自主翻转期间跳过失控恢复干预。
- `src/main/flight/pid.c:699`：`detectAndSetCrashRecovery()` 在自主翻转期间禁止碰撞恢复触发。
- `src/main/flight/pid.c:1059`：`pidController()` 接入自主翻转保护。
- `src/main/flight/pid.c:1154-1195`：修正 chirp 轴枚举的循环、类型转换和格式化判断。

### [官方代码修改] 地面站开关映射

- `src/main/msp/msp_box.c:202`：`initActiveBoxIds()` 初始化 MSP 可见开关，并固定暴露 BOXUSER1、BOXUSER2 和 BOXUSER3。
- `src/main/msp/msp_box.c:336-338`：暴露 USER1/USER2/USER3 的 AUX 通道映射入口；USER1 的配置仅用于选择待读取的原始通道，不直接启用高度或位置保持模式。

## SITL 与辅助工具

### [官方代码修改] SITL 固件

- `src/platform/SIMULATOR/sitl.c:276`：`updateState()` 接收仿真状态，并周期输出姿态/加速度诊断信息。
- `src/platform/SIMULATOR/target/SITL/target.h:123-125`：为 SITL 启用 `USE_CHIRP`、`USE_RX_MSP` 和 `USE_RX_MASP_OVERRIDE`。
- `src/platform/SIMULATOR/target/SITL/target.h:238-242`：明确 Gazebo UDP IMU 数据使用 FRD 传感器坐标系，姿态四元数使用桥接包坐标系。

### [自定义新增] `gazebo_pose.py`

Gazebo 模型位姿交互工具，手动运行，不由固件调用。

- 第 31 行：`quaternion_from_euler()`，将欧拉角转换为 Gazebo 使用的四元数。
- 第 52 行：`current_quaternion()`，根据当前姿态生成四元数。
- 第 61 行：`print_status()`，显示位置、姿态、速度和四元数。
- 第 81 行：`apply_pose()`，通过 Gazebo service 设置模型位置和姿态。
- 第 138 行：`apply_velocity()`，通过 Gazebo service 设置线速度。
- 第 180 行：`reset_pose()`，恢复默认位置、水平姿态和零速度。
- 第 190 行：`print_help()`，显示位置、姿态、速度设置、应用和复位命令。
- 第 211 行：`main()`，运行交互命令循环；`p` 支持同时设置位置、姿态和速度，`y` 应用当前值，`re` 复位模型。

### [自定义新增] `sitl_rc.py`

基于 UDP 的 SITL RC 控制工具，手动运行，不由固件调用。

- 第 11-28 行：初始化 16 路 RC 通道，默认设置姿态、油门和开关通道。
- 第 35 行：`print_status()`，显示全部 16 路 RC 通道值。
- 第 45 行：`command_thread()`，处理 `ch1` 至 `ch16`、`ch <number> <value>` 和 `ud<number>` 命令；`ud<number>` 将通道置高 0.5 秒后恢复低位。
- 第 167-181 行：组装 16 路 RC 数据包并以 50 Hz 发送到 SITL UDP 端口。

### [自定义新增] `tx16s_sitl_rc.py`

TX16S USB 摇杆到 SITL RC UDP 的桥接工具。

- 第 54 行：`print_edgetx_setup()`，显示 EdgeTX Extended 模式配置。
- 第 93 行：`axis_to_pwm()`，将摇杆轴值转换为 1000 至 2000 的 PWM。
- 第 101 行：`switch_to_pwm()`，将按钮组转换为二段或三段开关 PWM。
- 第 116 行：`read_channels()`，生成 16 路 RC 通道并报告无效开关编码。
- 第 136 行：`find_joystick()`，查找并初始化 USB 摇杆。
- 第 152 行：`validate_layout()`，检查 8 轴/22 按钮的 Extended HID 布局，拒绝会丢失三段开关中位的 Classic 模式。
- 第 177 行：`format_channels()`，格式化 RC 通道监视输出。
- 第 182 行：`diagnose()`，显示原始 HID 输入和解码结果。
- 第 213 行：`parse_args()`，解析主机、端口、设备、诊断、监视和配置打印参数。
- 第 225 行：`main()`，以 50 Hz 向 SITL 发送 16 路 RC 数据。

### [官方代码修改] Failsafe 单元测试

- `src/test/unit/flight_failsafe_unittest.cc:111`：增加 USER2 开关激活测试辅助函数。
- `src/test/unit/flight_failsafe_unittest.cc:727`：增加 `TestAutonomousLockAutoReleasesAfterDisarmWithUser2High`，验证 USER2 保持高电平时，failsafe 解锁并解除 ARM 后 synthetic RX 锁、RX_FAILSAFE 阻止标志和 failsafe 状态能够恢复。
- `src/test/unit/flight_failsafe_unittest.cc:953`、`:995`：增加有效飞行通道测试桩及 `rxAreFlightChannelsValid()` 实现。
- `src/test/unit/flight_failsafe_unittest.cc:256`：调整 failsafe landing 测试的时间推进，使其覆盖完整 RX 恢复周期。
