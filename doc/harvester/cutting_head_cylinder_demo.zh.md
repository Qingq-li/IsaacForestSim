# Cutting Head 圆柱体切割示例（中文）

## 文件

- 示例场景：
  - [data/robotic_twin/saha-robot/harveri_cutting_head_cylinder_demo.usda](/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_cutting_head_cylinder_demo.usda)
- 生成脚本：
  - [tools/create_cutting_head_cylinder_demo.py](/home/prefor/IsaacForestSim/tools/create_cutting_head_cylinder_demo.py)

## 目的

这个示例不是完整伐木工艺仿真，而是用最小测试件验证三件事：

1. `GRASP` 模式下，送料摆臂是否向树干通道内收
2. `FEED` 模式下，左右送料滚轮和测长轮转向是否合理
3. `CUT` 模式下，锯盘摆出方向和切割平面是否大致正确

## 场景内对象

- `StandingTrunk`
  - 竖直静态圆柱
  - 半径 `0.11 m`
  - 高度 `2.4 m`
- `FeedLog`
  - 水平动态圆柱
  - 半径 `0.08 m`
  - 长度 `1.4 m`
- `CutTargetMarker`
  - 薄圆柱标记片
  - 用来观察当前切割平面位置

## 初始模式

该 demo 默认 override：

- `/Graph/ros_drive_graph/cutting_head_targets.inputs:mode = 1`

也就是默认从 `GRASP` 模式启动。

## 建议验证顺序

1. 先打开 demo stage，观察 `GRASP`
   - `FEED_ARM_LEFT/RIGHT` 应向通道内侧收拢
   - `SAW_SWING` 应保持收纳
   - 所有旋转件应静止

2. 把 mode 改成 `2`，观察 `FEED`
   - 左右送料滚轮应反向转动
   - `MEASURING_WHEEL` 应开始转动
   - `SAW_DISC` 应保持静止
   - `FeedLog` 应是第一优先观察对象

3. 把 mode 改成 `3`，观察 `CUT`
   - `SAW_SWING` 应摆出
   - `SAW_DISC` 应高速旋转
   - `StandingTrunk` 和 `CutTargetMarker` 用来检查切割面位置

4. 把 mode 改回 `0`
   - 所有主动速度应平滑回零
   - 送料摆臂应回到待机位

## 当前判断标准

如果下面这些现象成立，说明当前 cutting head 结构和 graph 基线基本正常：

- 左右送料滚轮不是同向转
- 锯盘不是在 `FEED` 模式误转
- `CUT` 模式下锯盘会先摆出再进入工作位
- `GRASP` 与 `IDLE` 的摆臂开度明显不同
- 圆柱体中心线大致落在 `HEAD_FRAME` 树干通道附近

## 当前局限

这版 demo 仍然没有实现：

- 真实切断后的几何分段
- 圆柱体被切开后的约束重建
- 基于接触的自动模式切换
- clamp arms 的图上联动控制

因此它更适合作为：

- 运动方向检查
- 控制图接线检查
- 关节符号和速率限制检查

而不是最终工艺验收场景。
