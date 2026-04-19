# SAHA 伐木 Cutting Head 设计说明（中文）

## 目标

本文基于当前仓库中的 SAHA 采伐机模型，给出一版面向仿真的 cutting head 设计方案，目标支持后续这些能力：

1. 自动靠近并包络竖直树干
2. 抓持树干并完成根部切断
3. 倒木后持续送料
4. 按定长截断成若干长度一致的木段
5. 为后续自动化控制、感知、状态机和物理仿真预留清晰结构

相关资产：

- [data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd)
- [data/robotic_twin/saha-robot/harveri_isaac_flat/configuration/harveri_isaac_flat_base.usd](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_isaac_flat/configuration/harveri_isaac_flat_base.usd)
- [tools/scaffold_cutting_head.py](/home/prefor/IsaacForestSim/tools/scaffold_cutting_head.py)

## 当前实现状态（2026-04-19）

基于现有 3 个头部关节，已经落地了一版第一阶段 scaffold，用来把目前的头部从“小块占位件”提升到更接近真实伐木头的功能外形。

随后又继续推进了第二阶段的结构化重构：已经在 USD 结构中新增了显式的 body 和 joint，占位层级不再只依赖原始 `CUTTER_ARM` 与左右 gripper。

### 已经实现的内容

实现方式：

- 没有直接往实例代理 prim 上写数据
- 而是把第一版 head scaffold 写入了 base prototype 层
- 具体 authoring 位置在：
  - `/visuals/TOOL_TILT/cutting_head_v1`
  - `/visuals/CUTTER_ARM/cutting_head_v1`
  - `/visuals/GRIPPER_LEFT/cutting_head_v1`
  - `/visuals/GRIPPER_RIGHT/cutting_head_v1`
  - 以及对应的 `/colliders/.../cutting_head_v1`

几何上已经补上的模块：

- 主体框架外壳 `TOOL_TILT` 下的 boxy shell
- 侧置根切锯模块 `CUTTER_ARM` 下的 saw housing + saw disc
- 左右侧双臂包络结构
- 左右送料滚轮占位件

第二阶段已经新增的显式 body：

- `HEAD_FRAME`
- `CLAMP_ARM_UL`
- `CLAMP_ARM_LL`
- `CLAMP_ARM_UR`
- `CLAMP_ARM_LR`
- `FEED_ARM_LEFT`
- `FEED_ARM_RIGHT`
- `FEED_ROLLER_LEFT`
- `FEED_ROLLER_RIGHT`
- `SAW_SWING`
- `SAW_DISC`
- `MEASURING_WHEEL`

第二阶段已经新增的显式 joint：

- `J_HEAD_FRAME_MOUNT`
- `J_CLAMP_ARM_UL`
- `J_CLAMP_ARM_LL`
- `J_CLAMP_ARM_UR`
- `J_CLAMP_ARM_LR`
- `J_FEED_ARM_LEFT`
- `J_FEED_ARM_RIGHT`
- `J_FEED_ROLLER_LEFT_SPIN`
- `J_FEED_ROLLER_RIGHT_SPIN`
- `J_FELLING_SAW_SWING`
- `J_SAW_DISC_SPIN`
- `J_MEASURING_WHEEL`

新增结构写入脚本：

- [tools/upgrade_cutting_head_structure.py](/home/prefor/IsaacForestSim/tools/upgrade_cutting_head_structure.py)

### 当前这版 scaffold 的设计取向

这版不是最终真实 head，但它遵守两个原则：

1. 保持当前模型在视觉上的大致整体形状  
   仍然是一个前端 boxy 头部 + 侧置圆锯 + 左右抓持件的总体轮廓。

2. 向真实 harvester head 的功能拓扑靠近  
   不再把头部看成“两块木块夹具”，而是开始呈现：
   - 主框架
   - 包络臂
   - 送料轮
   - 根切锯

### 当前这版的工程限制

这版 scaffold 仍然受现有关节链约束：

- 只有 `J_CUTTER_ARM`
- 只有 `J_GRIPPER_LEFT`
- 只有 `J_GRIPPER_RIGHT`

因此当前实现是：

- 用 `CUTTER_ARM` 近似承载 saw swing
- 用左右 gripper 各自扩展成上下双臂，从视觉和功能上近似四臂包络

这能显著改善外形和后续控制准备，但还不等于完整的四独立臂机构。

### 当前 compose 验证结果

已经重新检查过 `harveri_sensor_pj_simple.usd` 的最终组合结果：

- `CLAMP_ARM_UL/LL/UR/LR`
- `HEAD_FRAME`
- `FEED_ARM_LEFT/RIGHT`
- `FEED_ROLLER_LEFT/RIGHT`
- `SAW_SWING`
- `SAW_DISC`
- `MEASURING_WHEEL`
- 以及对应的 `visuals` / `collisions` 子 prim

都已经能够在最终 stage 中正常 compose。

目前打开整车场景时最明显的 warning 不是 cutting head 新结构本身，而是外部传感器资产：

- `HESAI_XT32_SD10.usd`

该资产通过远程 URL 引用，当前环境下无法打开，所以会在 stage open 时打印 asset warning。

这意味着：

- cutting head 结构层级已经接通
- 新增 body / joint 的引用链路目前可用
- 后续如果还看到 stage warning，优先区分是 head 结构问题，还是外部传感器资产不可达问题

## 对当前结构的检查结论

从当前 USD 结构看，头部已有的相关关节主要是：

- `J_CUTTER_ARM`
- `J_GRIPPER_LEFT`
- `J_GRIPPER_RIGHT`

对应部件：

- `CUTTER_ARM`
- `GRIPPER_LEFT`
- `GRIPPER_RIGHT`

### 当前模型存在的主要问题

1. 头部包络太小  
   当前 `GRIPPER_LEFT` 和 `GRIPPER_RIGHT` 的包围盒尺寸大约只有 `5 cm x 4.5 cm x 7 cm` 量级，明显更像占位块，不像能包络树干的抓持臂。

2. 抓取结构不完整  
   目前只有左右两个小件，没有形成完整的四杆/四臂包络结构，也没有“上、下、前、后”对树干的约束。

3. 没有 saw 模块  
   现在没有明确的锯片/锯条/锯盒结构，也没有 saw 展开机构。

4. 没有 feed 送料机构  
   没有滚轮、履带轮或压送机构，因此无法稳定沿树干轴向送料。

5. 没有枝条修整/定径结构  
   北欧常见 harvester head 通常不是单纯夹住树，而是要在送料过程中提供一定的束缚、定心和近似去枝/修整功能。当前头部完全缺这层结构。

6. 没有量测链  
   要做定长截断，至少需要长度估计来源。当前头部没有测长轮、送料编码器或替代量测件。

7. 缺少适合控制的结构分层  
   现有结构偏“几个孤立关节”，不利于后续实现“接近树干 -> 抱树 -> 锯切 -> 扶持 -> 送料 -> 定长截断”的明确状态机。

## 当前已落地的结构量级

当前组合场景里，新增部件已经有了可用的占位尺度，量级上比早期“小木块夹具”更接近林业机具：

- `HEAD_FRAME`：约 `0.320 x 0.290 x 0.349 m`
- `CLAMP_ARM_UL` / `CLAMP_ARM_UR`：约 `0.188 x 0.128 x 0.174 m`
- `CLAMP_ARM_LL` / `CLAMP_ARM_LR`：约 `0.207 x 0.128 x 0.140 m`
- `FEED_ARM_LEFT` / `FEED_ARM_RIGHT`：约 `0.177 x 0.065 x 0.131 m`
- `FEED_ROLLER_LEFT` / `FEED_ROLLER_RIGHT`：约 `0.234 x 0.206 x 0.198 m`
- `SAW_SWING`：约 `0.233 x 0.060 x 0.212 m`
- `SAW_DISC`：约 `0.465 x 0.056 x 0.465 m`
- `MEASURING_WHEEL`：约 `0.150 x 0.040 x 0.150 m`

这说明目前已经具备：

- 一个可作为根节点的 head 主框架
- 一个可识别的四臂包络雏形
- 一对可摆入树干通道的送料摆臂
- 一对可独立驱动的送料轮
- 一个明确的锯盒摆出中间层
- 一个明显的根切锯盘占位
- 一个独立的测长轮挂点

但还没有形成完整的工业级 head 运动学。

## 当前已落地的关节参数

目前新增 joint 的驱动参数已经 author 到 USD，可作为下一阶段控制整定的基线：

- `J_CLAMP_ARM_*`
  - 限位大致在 `[-35, 10] deg` 或 `[-10, 35] deg`
  - `stiffness = 3200`
  - `damping = 45`
  - `maxForce = 1200`
- `J_HEAD_FRAME_MOUNT`
  - `TOOL_TILT -> HEAD_FRAME` 固连
- `J_FEED_ARM_LEFT`
  - 限位约 `[-30, 15] deg`
  - `stiffness = 2600`
  - `damping = 40`
  - `maxForce = 1600`
- `J_FEED_ARM_RIGHT`
  - 限位约 `[-15, 30] deg`
  - `stiffness = 2600`
  - `damping = 40`
  - `maxForce = 1600`
- `J_FEED_ROLLER_LEFT_SPIN`
  - 连续转动
  - `stiffness = 0`
  - `damping = 12`
  - `maxForce = 1500`
- `J_FEED_ROLLER_RIGHT_SPIN`
  - 连续转动
  - `stiffness = 0`
  - `damping = 12`
  - `maxForce = 1500`
- `J_FELLING_SAW_SWING`
  - 限位约 `[-8, 78] deg`
  - `stiffness = 1800`
  - `damping = 36`
  - `maxForce = 2600`
- `J_SAW_DISC_SPIN`
  - 连续转动
  - `stiffness = 0`
  - `damping = 8`
  - `maxForce = 3000`
- `J_MEASURING_WHEEL`
  - 连续转动
  - `stiffness = 0`
  - `damping = 4`
  - `maxForce = 400`

这一版参数还不是最终值，但对后续验证已经足够：

- 抓持臂先作为位置驱动件
- 送料摆臂和锯摆臂先作为位置驱动件
- 送料轮、锯盘和测长轮先作为速度驱动件

## 当前已接入的控制图

目前已经把 cutting head 的第一版默认控制接入 `/Graph/ros_drive_graph`，但和底盘 `/cmd_vel` 链路分开：

- 底盘仍由 `cmd_vel_filter -> articulation_controller` 链控制
- head 新增了一条独立链，只负责默认姿态与默认转速

新增节点：

- `/Graph/ros_drive_graph/cutting_head_targets`
- `/Graph/ros_drive_graph/articulation_controller_cutting_head_position`
- `/Graph/ros_drive_graph/articulation_controller_cutting_head_velocity`

对应脚本：

- [data/robotic_twin/saha-robot/cutting_head_controller.py](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/cutting_head_controller.py)
- [tools/update_ros_drive_graph.py](/home/prefor/IsaacForestSim/tools/update_ros_drive_graph.py)

### 当前控制分工

位置控制器负责：

- `J_CLAMP_ARM_UL`
- `J_CLAMP_ARM_LL`
- `J_CLAMP_ARM_UR`
- `J_CLAMP_ARM_LR`
- `J_FEED_ARM_LEFT`
- `J_FEED_ARM_RIGHT`
- `J_FELLING_SAW_SWING`

速度控制器负责：

- `J_FEED_ROLLER_LEFT_SPIN`
- `J_FEED_ROLLER_RIGHT_SPIN`
- `J_MEASURING_WHEEL`
- `J_SAW_DISC_SPIN`

### 当前模式位与默认状态机

当前 ScriptNode 已经从“固定默认值”升级成简化模式机：

- `0 = IDLE`
- `1 = GRASP`
- `2 = FEED`
- `3 = CUT`

当前各模式目标如下：

- `IDLE`
  - `clamp_arms = 0 / 0 / 0 / 0 rad`
  - `feed_arm_left/right = -0.16 / 0.16 rad`
  - `saw_swing = 0.0 rad`
  - `feed_rollers = 0 / 0 rad/s`
  - `measuring_wheel = 0 rad/s`
  - `saw_disc = 0 rad/s`
- `GRASP`
  - `clamp_arms = 0.38 / -0.38 / -0.38 / 0.38 rad`
  - `feed_arm_left/right = -0.28 / 0.28 rad`
  - `saw_swing = 0.0 rad`
  - 其余转动件保持 0
- `FEED`
  - `clamp_arms = 0.30 / -0.30 / -0.30 / 0.30 rad`
  - `feed_arm_left/right = -0.24 / 0.24 rad`
  - `feed_roller_left/right = 5.5 / -5.5 rad/s`
  - `measuring_wheel = 4.0 rad/s`
  - `saw_swing = 0.0 rad`
  - `saw_disc = 0 rad/s`
- `CUT`
  - `clamp_arms = 0.42 / -0.42 / -0.42 / 0.42 rad`
  - `feed_arm_left/right = -0.26 / 0.26 rad`
  - `saw_swing = 1.18 rad`
  - `saw_disc = 95.0 rad/s`
  - `feed_rollers = 0 / 0 rad/s`
  - `measuring_wheel = 0 rad/s`

这意味着当前已经能在 graph 级别区分：

- 待机姿态
- 四臂抓树闭环
- 送料滚轮工作
- 锯盘摆出并自转

### 当前速率限制

ScriptNode 内已经带一层简单平滑：

- `feed_arm_rate_limit = 0.45 rad/s`
- `feed_roller_accel_limit = 18.0 rad/s^2`
- `saw_swing_rate_limit = 0.75 rad/s`
- `measuring_wheel_accel_limit = 20.0 rad/s^2`
- `saw_disc_accel_limit = 220.0 rad/s^2`

这一层的目的不是做完整作业控制，而是：

- 防止 target 突变
- 给后续状态机留一个稳定入口
- 先把 head 默认姿态和默认执行器命令从图里跑通

## 圆柱体切割示例

为了验证当前 cutting head 结构和控制图是否基本连通，现在有两种验证入口：

- 主场景内验证组
  - `/World/CuttingHeadValidation/StandingTrunk`
  - `/World/CuttingHeadValidation/FeedLog`
  - `/World/CuttingHeadValidation/CutTargetMarker`
- 独立 demo stage
  - [data/robotic_twin/saha-robot/harveri_cutting_head_cylinder_demo.usda](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_cutting_head_cylinder_demo.usda)

对应脚本：

- [tools/create_cutting_head_cylinder_demo.py](/home/prefor/IsaacForestSim/tools/create_cutting_head_cylinder_demo.py)
- [tools/add_cutting_head_validation_cylinders.py](/home/prefor/IsaacForestSim/tools/add_cutting_head_validation_cylinders.py)

这个示例包含三个测试件：

- `StandingTrunk`
  - 竖直静态圆柱
  - 用来验证 `GRASP` 和 `CUT` 模式下的包络与锯盘摆出方向
- `FeedLog`
  - 水平动态圆柱
  - 用来验证 `FEED` 模式下送料滚轮和测长轮方向是否合理
- `CutTargetMarker`
  - 薄圆柱标记片
  - 用来快速观察当前锯切平面大致位置

当前 demo 文件默认把：

- `/Graph/ros_drive_graph/cutting_head_targets.inputs:mode = 1`

也就是启动后默认进入 `GRASP` 模式。

## 形态约束

后续继续完善 cutting head 时，必须保持以下方向不变：

1. 总体轮廓要继续接近真实北欧 harvester head  
   即：
   - 相对厚实的主体框架
   - 左右包络臂
   - 一侧明显的锯模块
   - 中央树干通道

2. 不要退回到“简单机械夹爪”造型  
   这类头部不是通用 gripper，而是树干加工头。

3. 外形增强必须服务功能拓扑  
   新增的每个壳体、臂、滚轮、刀具，最好都能映射到：
   - 抓持
   - 送料
   - 根切
   - 定长
   - 导向

4. 尺度上要保持“重型林业机具”的感觉  
   不能再出现几厘米级的小夹块承担主抓持功能。

## 设计原则

对你这个项目，建议不要一开始就追求工业级几何细节，而要优先做“控制友好、物理清晰、可迭代”的头部。建议分三层推进：

1. 功能几何层  
   先把能工作的包络、抓持、切断、送料、测长结构补齐。

2. 物理交互层  
   再把碰撞体、摩擦、驱动力、接触面和关节限位调到可信。

3. 外观细节层  
   最后再补真实外壳、液压缸、软管和造型细节。

## 建议的 Cutting Head 总体结构

建议将 cutting head 拆成 6 个子模块：

1. 主框架 `HEAD_FRAME`
2. 四臂抓持/去枝机构 `CLAMP_ARM_*`
3. 送料滚轮机构 `FEED_ROLLER_*`
4. 根切锯模块 `FELLING_SAW_*`
5. 定长测量模块 `MEASURING_WHEEL`
6. 树干导向通道 `STEM_CHANNEL`

### 1. 主框架 `HEAD_FRAME`

这是整个 cutting head 的刚性主体，挂在现有 `TOOL_TILT` / `CUTTER_ARM` 链下。

职责：

- 承载所有抓持臂、滚轮、锯模块
- 定义“树干通道”中心线
- 作为传感器和控制参考坐标系

建议：

- 不要再把 `CUTTER_ARM` 当成实际 cutting head 主体
- `CUTTER_ARM` 更适合作为一个小副臂或侧刀组件
- 新增一个更大的 `HEAD_FRAME`，作为头部总成根节点

### 2. 四臂抓持/去枝机构

这是你现在最缺的部分，也是最该先补的部分。

建议采用“简化四臂包络结构”，即四个可控臂围绕树干中心闭合：

- `CLAMP_ARM_UL`：左上臂
- `CLAMP_ARM_UR`：右上臂
- `CLAMP_ARM_LL`：左下臂
- `CLAMP_ARM_LR`：右下臂

每个臂末端使用简化的弧形接触面或短圆柱接触面，先不做复杂刀刃形状。

#### 这四个臂的作用

- 抓持并定心树干
- 在根切阶段保持树干稳定
- 在送料阶段持续约束树干，减少跳动
- 为以后扩展成去枝刀提供基础结构

#### 为什么建议四臂

因为你未来要处理的是竖直树木，不是简单抓一个箱体。四臂相比两块木块有几个明显优势：

- 对不同直径树干更容易居中
- 能明显减少偏心抓持
- 更适合后续和送料滚轮配合
- 更像真实 forestry harvester head 的工作包络

#### 建议的第一版几何

对仿真首版，建议每个抓持臂用：

- 一根主梁
- 一个弧形内侧接触块
- 一个简单碰撞体

目标包络建议：

- 最大开口直径：`0.50 m ~ 0.60 m`
- 常用作业树干直径：`0.10 m ~ 0.35 m`
- 闭合后的最小通道：`0.05 m ~ 0.08 m`

这比当前两个小块的量级大得多，也更接近真实使用场景。

### 3. 送料滚轮机构

建议增加两组主动送料滚轮，而不是只靠抓持臂移动树干。

建议最简版本：

- `FEED_ROLLER_LEFT`
- `FEED_ROLLER_RIGHT`

每个滚轮：

- 通过一个摆臂压向树干
- 自身可旋转输出角速度
- 接触面用高摩擦材质

#### 作用

- 沿树干轴线送料
- 为测长提供可靠运动量
- 在“切成相同长度木段”时提供基础执行能力

#### 为什么不建议一开始做四个送料轮

可以做，但第一版仿真更容易收敛的是两主动轮 + 四包络臂。  
如果一开始就做 4 个送料轮、4 个抓持臂、锯模块、测长轮，接触稳定性和调参难度会急剧上升。

建议路线：

1. 先做 4 抓持臂 + 2 送料滚轮
2. 验证能稳住树干并匀速送料
3. 再决定是否升级到 4 送料轮

### 4. 根切锯模块

这里建议用“展开式圆锯盘”或“摆出式锯盒”做仿真近似。

建议结构：

- `FELLING_SAW_HOUSING`
- `FELLING_SAW_DISC`
- `J_FELLING_SAW_SWING`
- `J_FELLING_SAW_SPIN`

#### 仿真建议

第一版不要做复杂链锯。  
用一个薄圆盘近似切割机构更适合仿真和控制。

两个自由度足够：

- `SWING`：锯盘从收纳位摆入树干下方
- `SPIN`：锯盘自转

#### 工作方式

- 抓持树干
- 锯盘摆出进入切割位置
- 自转并完成根切
- 回收到收纳位

### 5. 定长测量模块

为了做“相同长度段木”，必须有长度估计。

建议加：

- `MEASURING_WHEEL`
- `J_MEASURING_WHEEL`

作用：

- 压在树干表面
- 随送料转动
- 通过角位移估算送料长度

如果物理接触版太难，也可以先做控制近似版：

- 直接积分送料轮角速度推算长度

但从结构设计上，仍建议把测长轮预留出来。

### 6. 树干导向通道 `STEM_CHANNEL`

建议在头部中心显式定义“树干通道”。

这是很多仿真模型容易忽略的部分。  
如果没有明确通道，树干在碰撞和送料过程中会漂移，导致：

- 抓持不稳
- 锯切位置不稳定
- 送料抖动
- 测长误差大

导向通道可以由这些部件共同形成：

- 四抓持臂的内侧接触面
- 左右送料滚轮
- 上方或后方导向块

## 建议的关节层级

建议 future-proof 的关节命名与层级如下：

```text
HEAD_FRAME
|- CLAMP_ARM_UL
|  |- J_CLAMP_ARM_UL
|- CLAMP_ARM_UR
|  |- J_CLAMP_ARM_UR
|- CLAMP_ARM_LL
|  |- J_CLAMP_ARM_LL
|- CLAMP_ARM_LR
|  |- J_CLAMP_ARM_LR
|- FEED_ARM_LEFT
|  |- J_FEED_ARM_LEFT
|  |- FEED_ROLLER_LEFT
|     |- J_FEED_ROLLER_LEFT_SPIN
|- FEED_ARM_RIGHT
|  |- J_FEED_ARM_RIGHT
|  |- FEED_ROLLER_RIGHT
|     |- J_FEED_ROLLER_RIGHT_SPIN
|- FELLING_SAW_HOUSING
|  |- J_FELLING_SAW_SWING
|  |- FELLING_SAW_DISC
|     |- J_FELLING_SAW_SPIN
|- MEASURING_ARM
|  |- J_MEASURING_ARM
|  |- MEASURING_WHEEL
|     |- J_MEASURING_WHEEL
```

## 建议的自由度与控制分组

从控制实现角度，建议把 DOF 分成 4 组：

### A. 姿态组

由现有机械臂末端负责：

- `J_TOOL_PITCH`
- `J_TOOL_ROLL`
- `J_TOOL_YAW`
- `J_TOOL_TILT`

职责：

- 让 cutting head 对准树干
- 控制切割角度与扶持姿态

### B. 抓持组

- `J_CLAMP_ARM_UL`
- `J_CLAMP_ARM_UR`
- `J_CLAMP_ARM_LL`
- `J_CLAMP_ARM_LR`

职责：

- 从打开位收拢
- 包络树干
- 维持居中夹持

### C. 送料组

- `J_FEED_ARM_LEFT`
- `J_FEED_ARM_RIGHT`
- `J_FEED_ROLLER_LEFT_SPIN`
- `J_FEED_ROLLER_RIGHT_SPIN`

职责：

- 压紧树干
- 提供轴向送料

### D. 切断组

- `J_FELLING_SAW_SWING`
- `J_FELLING_SAW_SPIN`

职责：

- 完成根切与定长截断

## 建议的作业流程状态机

建议 future controller 按下面状态机设计，而不是把所有动作直接混在一起。

### 1. `APPROACH_TREE`

目标：

- 将 head 中心轴线对准树干
- 保持四抓持臂打开
- 锯模块收回

### 2. `PRE_GRASP`

目标：

- 让树干进入 `STEM_CHANNEL`
- 上下左右抓持臂开始半闭合

### 3. `CLAMP_AND_CENTER`

目标：

- 四臂闭合到接触树干
- 调整姿态让树干进入头部中心

### 4. `FELL_BASE`

目标：

- 保持夹紧
- 锯盘摆出
- 启动锯切
- 完成根切

### 5. `SUPPORT_AND_LAYDOWN`

目标：

- 维持对树干控制
- 配合主臂使树倒向安全方向

### 6. `FEED_AND_MEASURE`

目标：

- 送料滚轮送进树干
- 根据测长轮/滚轮编码累计长度

### 7. `BUCK_TO_LENGTH`

目标：

- 达到目标段长
- 暂停送料
- 夹持稳定
- 锯盘摆出截断

### 8. `REPEAT_OR_RELEASE`

目标：

- 如果剩余长度足够，继续送料截下一段
- 否则释放残余短材

## 北欧作业流程在仿真中的简化建议

你提到“类似北欧标准的伐木流程”。对仿真项目，建议先做工程近似版，而不是立刻追求工业细节全覆盖。

### 第一阶段

先实现：

- 抓持竖直树干
- 根切
- 倒木后送料
- 按固定长度截断

先不做：

- 真实去枝刀刃几何
- 树皮/枝条复杂接触
- 链锯动力学细节
- 原木品质分级

### 第二阶段

再补：

- 上刀/侧刀近似去枝
- 更真实的送料接触面
- 测长误差模型
- 树干直径随高度变化模型

### 第三阶段

再考虑：

- 基于树种/直径的自适应夹持力
- 切段优化
- 多段长度策略
- 更真实的采伐工艺逻辑

## 具体尺寸建议（仿真首版）

这里给一组适合作为第一版的相对保守尺寸，不是工业实测值，而是为了让仿真更稳定、比例更可信。

### 头部主体

- 总高度：`0.9 m ~ 1.2 m`
- 最大外宽：`0.8 m ~ 1.0 m`
- 树干工作中心通道：`0.12 m ~ 0.40 m`

### 四抓持臂

- 单臂长度：`0.20 m ~ 0.35 m`
- 单臂厚度：`0.04 m ~ 0.08 m`
- 内侧接触弧半径覆盖：`0.05 m ~ 0.25 m`

### 送料滚轮

- 滚轮直径：`0.16 m ~ 0.24 m`
- 滚轮宽度：`0.06 m ~ 0.10 m`

### 锯盘

- 圆盘直径：`0.45 m ~ 0.65 m`
- 圆盘厚度：`0.01 m ~ 0.03 m`

这组尺寸已经能明显优于现在头部的“小块占位件”。

## 物理与仿真实现建议

### 抓持臂

先使用：

- `PhysicsRevoluteJoint`
- position drive
- 明确角度限位

不要一开始就上复杂液压缸联动。

### 送料滚轮

先使用：

- 摆臂压紧
- 滚轮自转速度控制
- 较高但合理的接触摩擦

### 锯模块

第一版切割判定建议：

- 当锯盘进入树干截面并持续一段时间
- 或根据接触深度 / 简化切割状态机
- 再把树体做“断裂/分段”处理

也就是说，切割可以先逻辑化，不必把真实切削力学一次做全。

## 你当前最值得优先修改的部分

如果只做最必要的改进，优先级建议如下：

1. 新建 `HEAD_FRAME`
2. 把现有左右 gripper 替换成四抓持臂
3. 增加左右送料滚轮
4. 增加一个可摆出圆锯盘
5. 预留测长轮

## 紧接着的实现建议

基于当前已经落地的 scaffold，下一步建议按下面顺序继续：

1. 新增 `HEAD_FRAME`，把 head 根节点从“视觉上像一个头”变成“结构上真的是一个头”
2. 把四个 `CLAMP_ARM_*` 从“近似独立臂”继续调整成更真实的包络几何
3. 给送料滚轮补独立摆臂，而不只是滚轮本体
4. 给 saw 模块补明确的 `swing` 与 `spin` 双 DOF 拓扑
5. 在 head 中央补测长轮与导向通道
6. 再开始写抓树、根切、送料、定长截断状态机

在这之后，再谈更细的去枝结构和外形优化。

## 一个现实判断

你现在这个 cutting head 不是“差一点细节”，而是还停留在非常早期的占位阶段。  
这不是坏事，反而说明现在重构成本还低。  
最正确的方向不是继续补两个更大的木块，而是直接把头部从“两小块夹具”升级成“主框架 + 四抓持臂 + 两送料轮 + 锯模块”的完整拓扑。

## 下一阶段设计注意事项

后面继续设计这类机器时，建议一直盯住下面几件事：

1. 先定义树干中心线，再布置抓持臂、送料轮和锯盘  
   不要先画壳体，再反推功能件位置。

2. 自由度数量要和控制目标一一对应  
   如果一个 DOF 没有明确工艺职责，后面通常只会增加接触不稳定和调参成本。

3. 抓持与送料要分层  
   抓持臂负责居中和约束，送料轮负责轴向输送，不要让同一个部件同时承担两类主要任务。

4. 切割执行件先做逻辑化，再做高保真力学  
   对自动化流程来说，先把状态机打通，比一开始追求真实切削力更重要。

5. 碰撞体一定要比视觉件更克制  
   真实外形可以复杂，但碰撞体应保持规则、连续、便于接触收敛。

6. 所有关键模块都要给未来传感预留参考坐标  
   尤其是树干中心、测长轮、锯切平面、送料方向。
