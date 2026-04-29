# Coordinate Frames

This file is the reference for coordinate-frame math in the task API. When
adding new pose, angle, or coordinate conversions, use the conventions here.

## Map Frame

Used by `/amcl_pose`, `robot.get_pose()`, `robot.goto(...)`, and
`BallDetection.absolute_*`.

- Basis: world/map frame.
- Unit: meters for position, degrees or radians for yaw.
- `+x`: map +x.
- `+y`: map +y.
- `yaw_deg = 0`: facing map `+x`.
- `yaw_deg = 90`: facing map `+y`.

`robot.goto(x_m=..., y_m=...)` expects an absolute target in this frame.

## Robot Base Link Frame

Used internally by the task API before rotating local ball measurements into the
map frame. `BallDetection.base_*` exposes this converted robot-frame value.

- Basis: robot body frame.
- Origin: robot body reference point used by localization.
- Unit: meters.
- `+x`: robot front.
- `+y`: robot left.
- `+z`: up.

Relative ball bearing in the robot frame:

```text
angle_deg = atan2(base_y_m, base_x_m)
```

So `0 deg` is front, `90 deg` is left, and `-90 deg` is right.

## Orange Ball Detector Frame

Used by `/orange_ball_detector/detection.ball_center_m` and exposed unchanged as
`BallDetection.local_*`.

- Basis: detector ground coordinate generated from the camera image/LUT.
- Unit: meters.
- `+x`: down in the camera image.
- `+y`: left in the camera image.
- `+z`: ball center height estimate.

The task API converts detector coordinates to `base_link` before computing
angles and map positions:

```text
base_x_m = -detector_x_m
base_y_m =  detector_y_m
base_z_m =  detector_z_m
```

This means image-up is robot-front, and image-left is robot-left.

## BallDetection Fields

Returned by `robot.find_ball(...)`.

- `local_x_m`, `local_y_m`, `local_z_m`: raw orange-ball detector frame.
- `base_x_m`, `base_y_m`, `base_z_m`: converted robot `base_link` frame.
- `x_m`, `y_m`, `z_m`: offset from robot to ball, rotated into map axes.
- `absolute_x_m`, `absolute_y_m`, `absolute_z_m`: absolute ball position in
  map frame.
- `angle_deg`: relative robot-frame bearing to the ball.
- `absolute_map_angle_deg`: ROS map-frame yaw toward the ball. This is for map
  math and debugging, not for `robot.turn(...)`.
- `turn_angle_deg`: STM32 firmware yaw target that can be passed directly to
  `robot.turn(angle_deg=...)`.

The map conversion uses the latest `/amcl_pose` yaw:

```text
map_dx = base_x_m * cos(yaw) - base_y_m * sin(yaw)
map_dy = base_x_m * sin(yaw) + base_y_m * cos(yaw)

absolute_x_m = robot_x_m + map_dx
absolute_y_m = robot_y_m + map_dy

absolute_map_angle_deg = normalize(robot_yaw_deg + angle_deg)
```

`normalize(...)` returns an angle in `[-180, 180)`.

`yaw_zero_map_degrees` must match the value used by `amcl_fusion`. The default
is `0.0`. If AMCL is launched with a different value, start `task_runner` /
`CompetitionRobot` with the same parameter.

## STM32 Motion Conventions

`robot.turn(angle_deg=...)` sends an absolute firmware yaw target.

Do not pass `absolute_map_angle_deg` to `robot.turn(...)`. ROS map yaw and STM32
yaw have different zero directions. `amcl_fusion` uses this conversion:

```text
ros_map_yaw_deg = normalize(90 + yaw_zero_map_degrees + stm32_yaw_deg)
```

The task API therefore computes the direct turn target as:

```text
turn_angle_deg = normalize(absolute_map_angle_deg - 90 - yaw_zero_map_degrees)
```

Use:

```python
robot.turn(angle_deg=ball.turn_angle_deg)
```

`robot.drive(speed_percent=..., move_angle_deg=...)` uses the firmware movement
angle convention:

- `0 deg`: robot front.
- `90 deg`: robot left.

`robot.move(x_cm=..., y_cm=...)` sends firmware `cmd_dis` directly in the
STM32 odometry-frame centimeter convention. Use `robot.goto(...)` for normal
map-frame navigation.
