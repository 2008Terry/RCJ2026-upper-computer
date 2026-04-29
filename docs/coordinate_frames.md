# Coordinate Frames

本文档是任务 API 中所有坐标、角度、位移转换的统一参考。ROS 自己规定的
map/base_link 坐标系不改；这里额外说明 STM32 yaw、STM32 位移轴、橙球视觉
pose 在本项目里的含义，以及它们和 ROS map 的关系。

## 核心约定

`normalize(angle_deg)` 表示把角度归一化到 `[-180, 180)`。

### ROS Map Frame

用于 `/amcl_pose`、`robot.get_pose()`、`robot.goto(...)`、
`BallDetection.absolute_*`。

- 位置单位：m。
- `+x`：地图右边。
- `+y`：地图上边。
- ROS map yaw `0 deg`：朝地图右边，也就是 `+x`。
- ROS map yaw `90 deg`：朝地图上边，也就是 `+y`。

`robot.goto(x_m=..., y_m=...)` 传入的是这个坐标系下的绝对地图坐标。

### STM32 / Robot Yaw Frame

用于 `robot.turn(angle_deg=...)`、`ball.absolute_angle_deg`、STM32 `cmd_turn`、
STM32 `cmd_request` 返回的 `theta`。

- 角度单位：deg。
- `0 deg`：机器人朝地图上方。
- `90 deg`：机器人朝地图左边。
- `-90 deg`：机器人朝地图右边。
- `180 deg` 或 `-180 deg`：机器人朝地图下方。

比赛任务里说“球的绝对角度”时，默认指这个 STM32 yaw 角度。也就是说，面向
球的绝对角度就是 `ball.absolute_angle_deg`，可以直接传给：

```python
robot.turn(angle_deg=ball.absolute_angle_deg)
```

STM32 yaw 和 ROS map yaw 的转换为：

```text
ros_yaw_deg = normalize(90 + yaw_zero_map_degrees + stm32_yaw_deg)
stm32_yaw_deg = normalize(ros_yaw_deg - 90 - yaw_zero_map_degrees)
```

默认 `yaw_zero_map_degrees = 0.0`。这表示上电后执行 yaw 重置时，机器人朝向
地图上方。

### STM32 Translation Frame

用于 STM32 `cmd_dis x y`，以及 `cmd_request` 返回的 `dx/dy`。

这是场地固定坐标，不是机器人自身的前后左右坐标。

- 位移单位：cm。
- `cmd_dis/request dx > 0`：地图左边，也就是 ROS map `-x`。
- `cmd_dis/request dy > 0`：地图下边，也就是 ROS map `-y`。

因此 ROS map 位移和 STM32 位移的转换为：

```text
cmd_x_cm = -map_dx_m * 100
cmd_y_cm = -map_dy_m * 100

map_dx_m = -cmd_x_cm * 0.01
map_dy_m = -cmd_y_cm * 0.01
```

例子：

- 地图目标在当前点右边 `0.30 m`：`map_dx_m = +0.30`，所以 `cmd_x_cm = -30`。
- 地图目标在当前点上方 `0.30 m`：`map_dy_m = +0.30`，所以 `cmd_y_cm = -30`。

`robot.move(x_cm=..., y_cm=...)` 会直接发送 STM32 `cmd_dis`，所以它的参数也
遵守这一套左/下场地固定轴。普通绝对导航请优先使用 `robot.goto(...)`。

### Robot Base Link Frame

用于任务 API 内部把球的视觉测量转成机器人相对量，并通过
`BallDetection.base_*` 暴露。

- 位置单位：m。
- `+x`：机器人前方。
- `+y`：机器人左方。
- `+z`：上方。

相对球角度：

```text
ball.angle_deg = atan2(base_y_m, base_x_m)
```

含义：

- `0 deg`：球在机器人正前方。
- `90 deg`：球在机器人左边。
- `-90 deg`：球在机器人右边。

### Orange Ball Detector Frame

用于 `/orange_ball_detector/detection.ball_center_m`，并原样暴露为
`BallDetection.local_*`。

- 位置单位：m。
- `local_x_m > 0`：机器人后方，也就是相机图像向下。
- `local_y_m > 0`：机器人左方，也就是相机图像向左。
- `local_z_m`：球心高度估计。

任务 API 转换为 `base_link` 后再计算角度和地图位置：

```text
base_x_m = -ball.local_x_m
base_y_m =  ball.local_y_m
base_z_m =  ball.local_z_m
```

## BallDetection 字段

`robot.find_ball(...)` 返回的球检测对象中：

- `local_x_m/local_y_m/local_z_m`：橙球检测器原始坐标。
- `base_x_m/base_y_m/base_z_m`：机器人 `base_link` 坐标。
- `x_m/y_m/z_m`：从机器人到球的 ROS map 轴向偏移。
- `absolute_x_m/absolute_y_m/absolute_z_m`：球在 ROS map 中的绝对位置。
- `angle_deg`：球相对机器人的方向，前 `0`、左 `90`、右 `-90`。
- `absolute_angle_deg`：STM32 yaw，也就是比赛中面向球的绝对角度，可直接给
  `robot.turn(angle_deg=...)`。

从机器人坐标到 ROS map 偏移，使用最新 `/amcl_pose` 的 ROS yaw：

```text
map_dx = base_x_m * cos(ros_yaw) - base_y_m * sin(ros_yaw)
map_dy = base_x_m * sin(ros_yaw) + base_y_m * cos(ros_yaw)

absolute_x_m = robot_x_m + map_dx
absolute_y_m = robot_y_m + map_dy
```

球的绝对角度计算过程中会先得到一个内部 ROS map yaw，但不暴露给任务 API：

```text
internal_ros_map_angle_deg = normalize(robot_ros_yaw_deg + ball.angle_deg)
absolute_angle_deg = normalize(
    internal_ros_map_angle_deg - 90 - yaw_zero_map_degrees
)
```

推荐用法：

- 要地图位置：用 `ball.absolute_x_m` / `ball.absolute_y_m`。
- 要球相对机器人方向：用 `ball.angle_deg`。
- 要让机器人转向球：用 `ball.absolute_angle_deg`。

## STM32 Motion Commands

`robot.turn(angle_deg=...)` 发送 STM32 `cmd_turn`，参数是 STM32 yaw：

```python
robot.turn(angle_deg=0)    # 朝地图上方
robot.turn(angle_deg=90)   # 朝地图左边
robot.turn(angle_deg=-90)  # 朝地图右边
```

`robot.drive(speed_percent=..., move_angle_deg=...)` 的移动角度是机器人自身方向：

- `0 deg`：机器人前方。
- `90 deg`：机器人左方。

`robot.move(x_cm=..., y_cm=...)` 直接发送 STM32 `cmd_dis`：

- `x_cm > 0`：地图左边。
- `y_cm > 0`：地图下边。

## Sanity Checks

下面这些例子用于快速检查坐标理解是否正确，假设 `yaw_zero_map_degrees = 0`。

- 机器人朝地图上方，球在机器人正前方：`ball.angle_deg = 0`，
  `ball.absolute_angle_deg = 0`。
- 机器人朝地图上方，球在机器人左边：`ball.angle_deg = 90`，
  `ball.absolute_angle_deg = 90`。
- 地图目标在当前点右边：`cmd_dis` 的 `x` 应为负。
- 地图目标在当前点上方：`cmd_dis` 的 `y` 应为负。

## Existing Pitfalls

- ROS map yaw 和 STM32 yaw 的零方向不同。`robot.get_pose().yaw_deg` 是 ROS
  map yaw，不是 STM32 yaw。
- 任务层只保留 `absolute_angle_deg` 作为球的绝对角度；它就是可直接
  `turn` 的 STM32 yaw，不再暴露 ROS map yaw 版本。
- `robot.move()` / `cmd_dis` 的 `x/y` 不是机器人 body 坐标，也不是 ROS map
  `x/y`，而是场地固定的左/下轴。
- `cmd_request` 返回的 `dx/dy` 和 `cmd_dis` 同轴：`dx` 正方向为地图左，
  `dy` 正方向为地图下。把它们和 ROS map `x/y` 直接相加会得到反号。
- `yaw_zero_map_degrees` 必须在 `amcl_fusion` 和 `CompetitionRobot` 中保持一致，
  否则 `ball.absolute_angle_deg` 会和 STM32 实际 yaw 不一致。
