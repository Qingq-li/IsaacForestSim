# SAHA Harvester `ros_drive_graph` 说明（中文）

## 范围

本文档记录了当前 `/Graph/ros_drive_graph` 控制设计，包含：

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd)

以及图使用的配套控制脚本：

- [data/robotic_twin/saha-robot/ros_drive_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/ros_drive_controller.py)

并且还记录了未来履带式林业机械控制设计的工程指导。

## 为什么要修改该图

原始 `ros_drive_graph` 存在两个结构性问题：

1. 它使用非常粗糙的运动学近似将 `/cmd_vel` 映射到转向和车轮速度，缺少低速保护、转向饱和和速率限制。
2. 它使用的车轮速度缩放与预期的车轮半径假设不一致。

对于大型铰接采伐机，这会产生明显不现实的行为：

- 当 `vx ~= 0` 时，转向会变得过于激进
- 车轮速度无法干净地跟踪预期车身速度
- 铰接关节可能被命令得比重型底盘实际可响应的更快

## 当前控制路径

该图现在通过 ScriptNode 路由 `/cmd_vel`：

- `/Graph/ros_drive_graph/cmd_vel_filter`

铰接控制器使用 ScriptNode 的输出：

- `J_STEER` 位置命令来自 `outputs:steer_command`
- 车轮速度命令来自：
  - `J_LF_WHEEL`
  - `J_RF_WHEEL`
  - `J_LH_WHEEL`
  - `J_RH_WHEEL`

旧的数学节点仍保留在图中作为回退/参考，但实际运行时控制路径现在是 ScriptNode。

## 当前新增的 Cutting Head 默认控制支路

除了底盘 `/cmd_vel` 支路外，`/Graph/ros_drive_graph` 现在还包含一条独立的 head 默认控制支路：

- `/Graph/ros_drive_graph/cutting_head_targets`
- `/Graph/ros_drive_graph/articulation_controller_cutting_head_position`
- `/Graph/ros_drive_graph/articulation_controller_cutting_head_velocity`

这条支路不消费 `/cmd_vel`，只消费：

- `on_playback_tick.outputs:tick`
- `on_playback_tick.outputs:deltaSeconds`

其目的很明确：

- 在没有完整作业状态机之前
- 先把 `FEED_ARM`、`SAW_SWING`、`MEASURING_WHEEL` 接进控制图
- 并给它们稳定的默认目标值与速率限制

### 关节分配

位置命令数组当前控制：

- `J_CLAMP_ARM_UL`
- `J_CLAMP_ARM_LL`
- `J_CLAMP_ARM_UR`
- `J_CLAMP_ARM_LR`
- `J_FEED_ARM_LEFT`
- `J_FEED_ARM_RIGHT`
- `J_FELLING_SAW_SWING`

速度命令数组当前控制：

- `J_FEED_ROLLER_LEFT_SPIN`
- `J_FEED_ROLLER_RIGHT_SPIN`
- `J_MEASURING_WHEEL`
- `J_SAW_DISC_SPIN`

### 当前模式位

- `0 = IDLE`
- `1 = GRASP`
- `2 = FEED`
- `3 = CUT`

### 当前模式目标

- `IDLE`
  - `clamp_arms = 0 / 0 / 0 / 0 rad`
  - `feed_arm_left/right = -0.16 / 0.16 rad`
  - `rollers = 0 / 0 rad/s`
  - `measuring_wheel = 0 rad/s`
  - `saw_swing = 0 rad`
  - `saw_disc = 0 rad/s`
- `GRASP`
  - `clamp_arms = 0.38 / -0.38 / -0.38 / 0.38 rad`
  - `feed_arm_left/right = -0.28 / 0.28 rad`
  - 其余执行件保持静止
- `FEED`
  - `clamp_arms = 0.30 / -0.30 / -0.30 / 0.30 rad`
  - `feed_arm_left/right = -0.24 / 0.24 rad`
  - `rollers = 5.5 / -5.5 rad/s`
  - `measuring_wheel = 4.0 rad/s`
  - `saw_disc = 0 rad/s`
- `CUT`
  - `clamp_arms = 0.42 / -0.42 / -0.42 / 0.42 rad`
  - `feed_arm_left/right = -0.26 / 0.26 rad`
  - `saw_swing = 1.18 rad`
  - `saw_disc = 95.0 rad/s`
  - `rollers = 0 / 0 rad/s`

### 当前平滑参数

- `feed_arm_rate_limit = 0.45 rad/s`
- `feed_roller_accel_limit = 18.0 rad/s^2`
- `saw_swing_rate_limit = 0.75 rad/s`
- `measuring_wheel_accel_limit = 20.0 rad/s^2`
- `saw_disc_accel_limit = 220.0 rad/s^2`

这表示 head 目前已经有了一版“图上可解释、默认可执行、后续易扩展”的控制基线，但它仍然不是完整的采伐状态机。

## 当前运动学假设

这些参数已写入 ScriptNode 输入：

- `wheel_base = 1.9476267844438553`
- `track_width = 1.5339090526103973`
- `wheel_radius = 0.4`
- `steer_sign = -1.0`

派生量：

- `half_track = 0.7669545263051987`

## 当前控制逻辑

ScriptNode 应用了四层处理。

### 1. 命令限制

输入的 `/cmd_vel` 首先被限幅：

- `max_linear_speed = 2.5 m/s`
- `max_angular_speed = 0.6 rad/s`

### 2. 速度平滑

指令的底盘速度被做速率限制：

- `max_linear_accel = 0.8 m/s^2`
- `max_linear_decel = 1.2 m/s^2`
- `max_angular_accel = 0.5 rad/s^2`

这会产生成滤波后的内部状态：

- `vx_filtered`
- `wz_filtered`

### 3. 转向保护与速率限制

为了避免在极低前向速度时转向奇异，转向计算使用：

- `min_linear_for_steer = 0.3 m/s`

转向目标计算为：

```text
steer_target =
    steer_sign * 2 * atan((0.5 * wheel_base * wz_filtered) / max(|vx_filtered|, min_linear_for_steer))
```

然后它受以下限制约束：

- `max_steer_angle = 1.0 rad`
- `max_steer_rate = 0.35 rad/s`

这会生成最终的 `J_STEER` 命令。

### 4. 车轮速度映射

车轮角速度由滤波后的车身运动计算：

```text
right_speed = (vx_filtered + wz_filtered * half_track) / wheel_radius
left_speed  = (vx_filtered - wz_filtered * half_track) / wheel_radius
```

四个车轮命令被分配为：

```text
[right_speed, left_speed, right_speed, left_speed]
```

并做最终饱和：

- `max_wheel_speed = 8.0 rad/s`

## 修改过的文件

当前实现修改了：

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd)
- [data/robotic_twin/saha-robot/ros_drive_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/ros_drive_controller.py)
- [data/robotic_twin/saha-robot/cutting_head_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/cutting_head_controller.py)
- [tools/update_ros_drive_graph.py](/home/prefor/IsaacForestSim/tools/update_ros_drive_graph.py)

预编辑 USD 的备份：

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd.bak](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd.bak)

## 实际验证检查项

在 Isaac Sim 中验证该控制器时，请检查：

1. 小前进命令且零偏航：
   - 车轮速度应平滑上升
   - `J_STEER` 应保持接近零

2. 小前进命令且小正偏航：
   - `J_STEER` 应缓慢移动，不应突然跳变
   - 左右车轮速度应一致分离

3. 非零 `wz` 时非常小的 `vx`：
   - 转向不应再跳到奇异角度
   - 输出应饱和并缓慢变化

4. `/cmd_vel` 的阶跃输入：
   - 车身响应应看起来被滤波，而不是瞬变
   - 转向应遵守可见的速率限制

5. 中等速度下的大偏航需求：
   - 转向应在不现实铰接发生前被钳位
   - 车轮速度饱和应防止过度车轮打滑

## 面向未来机器的设计指导

对于大型铰接林业机械，避免将平台当成小型通用移动机器人。以下规则很关键。

### 1. 从真实运动拓扑开始

首先确定机器的真实类型：

- 刚性差动驱动
- 前轴转向
- 划桨转向
- 中心铰接底盘
- 铰接底盘加驱动转向架
- 铰接底盘加主动悬挂/腿部机制

`/cmd_vel` 的控制映射必须匹配该拓扑。如果机器主要通过中心铰接关节转向，控制器应围绕铰接几何构建，而不是重用乘用车或简单差动驱动模型。

### 2. 使用真实几何量

始终从已定义的机器人几何或关节坐标系推导：

- 轴距或等效铰接长度
- 轨距
- 车轮半径
- 转向符号约定
- `base_link`、车轴坐标系、铰接关节坐标系的帧定义

不要依赖来自早期原型的近似常数，除非在资产更新后重新验证。

### 3. 保护低速转向

任何包含纵向速度除法的公式必须明确处理：

- `vx -> 0`
- 零点附近的符号翻转
- 倒车运动

否则，大型铰接机器在操作人员需要精细操纵时会显示不稳定的转向命令。

### 4. 对底盘而不仅仅是关节进行速率限制

对于重型作业机器，指令平滑应存在于车身运动层：

- 线性加速限制
- 减速限制
- 角加速限制

同时也要在转向执行器层：

- 转向角度限制
- 转向速率限制

这可以将操作意图与执行器可行性区分开来。

### 5. 尊重工作模式

同一台机器可能需要不同的限值用于：

- 运输驾驶
- 精密机动
- 臂架展开
- 工具接地/作业
- 倒车

单一固定的 `/cmd_vel` 映射通常不足以应对。通常需要基于模式的限值。

### 6. 考虑操纵臂状态

在采伐机上，臂架伸出和工具姿态会改变组合惯性和翻滚风险。良好的控制设计应最终减少允许的：

- 最大线速度
- 最大角速度
- 转向速率
- 曲率

当臂架伸出或承载负载时。

### 7. 优先专用低级控制器层

`/cmd_vel` 应被视为底盘意图接口，而不是直接执行器命令。一个稳健的体系通常是：

1. ROS 导航或遥控发布希望的车身运动
2. 低级机器控制器将车身运动转换为可行的铰接和车轮目标
3. 铰接和车轮执行器跟踪这些可行目标

这个层级是饱和、平滑、安全逻辑和机器特定运动学的所在。

### 8. 保持图可解释性

为便于维护，控制图应让这些量显而易见：

- `vx` 和 `wz` 的入口位置
- 滤波发生的位置
- 转向计算的位置
- 车轮速度计算的位置
- 限制应用的位置

如果图中出现过多难以阅读的小数学节点，请将逻辑移动到有文档的 ScriptNode 或专用扩展中。

### 9. 按预期机动验证

不要只用一个遥控命令进行验证。请测试：

- 直线
- 小半径转弯
- 低速高转向率
- 从停止开始的加速

### 10. 慢慢迭代

大型林业机器控制系统应逐步引入复杂性。先保证基本的平滑与边界保护，再逐步增加模式切换、姿态感知和负载敏感限制。
