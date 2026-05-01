# Task Runner 速查

只改 `scripts/task_logic.py` 的 `run_task(robot)`。

## 规则

- 尽量用关键字参数：`robot.goto(x_m=0.3, y_m=-0.4)`。
- `wait_sec`: 函数结束后额外等待。
- `speed_profile`: `0` 快，`1` 默认，`2` 稳。
- `goto`: 地图坐标，单位 m。
- `move`: STM32 世界系 cm；`x_cm` 地图上，`y_cm` 地图左。
- `drive`: 机器人自身方向；`0` 前，`90` 左。
- `turn`: STM32 yaw；`0` 地图上，`90` 地图左，`-90` 地图右。

## 常用函数

| 函数 | 用途 |
| --- | --- |
| `robot.motion_enable()` | 使能底盘 |
| `robot.motion_disable()` | 禁用底盘 |
| `robot.goto(x_m=..., y_m=...)` | 走到地图点，会用定位修正 |
| `robot.move(x_cm=..., y_cm=...)` | 直接按 STM32 位移走 |
| `robot.turn(angle_deg=...)` | 转到指定 yaw |
| `robot.turn_to_point(x_m=..., y_m=...)` | 转向地图点 |
| `robot.drive(speed_percent=..., move_angle_deg=...)` | 持续速度控制 |
| `robot.stop()` | 停止持续运动 |
| `robot.timer(duration_sec=...)` | 等待，不停止当前动作 |
| `robot.sleep(duration_sec=...)` | 先 `stop()` 再等待 |
| `robot.suck_on(speed_percent=...)` | 开吸球 |
| `robot.suck_off()` | 关吸球 |
| `robot.wait_for_ball_sucked(...)` | 等吸到球 |
| `robot.wait_for_ball_released(...)` | 等球放掉 |
| `robot.find_ball(...)` | 找球，失败返回 `None` |
| `robot.spin_find_ball(...)` | 原地旋转找球 |
| `robot.goto_ball_standoff(ball, stand_off_m=...)` | 走到球前指定距离 |
| `robot.get_pose()` | 读当前定位 |
| `robot.reset_yaw()` | STM32 yaw 归零 |
| `robot.relay_on()` / `robot.relay_off()` | 继电器开/关 |
| `robot.infrared_channel()` | 读红外通道 `1-7` |

## ball 字段

- `ball.angle_deg`: 给 `drive(move_angle_deg=...)`。
- `ball.absolute_angle_deg`: 给 `turn(angle_deg=...)`。
- `ball.absolute_x_m`, `ball.absolute_y_m`: 球的地图坐标。

## 例子

```python
def run_task(robot):
    robot.motion_enable()
    robot.goto(x_m=0.3, y_m=-0.4)
    robot.turn_to_point(x_m=0.0, y_m=0.0)
```

```python
def run_task(robot):
    robot.motion_enable()
    robot.suck_on(speed_percent=15)

    ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
    if ball is None:
        ball = robot.spin_find_ball(min_confidence=0.5)

    if ball is not None:
        robot.turn(angle_deg=ball.absolute_angle_deg)
        robot.goto_ball_standoff(ball, stand_off_m=0.30)
```
