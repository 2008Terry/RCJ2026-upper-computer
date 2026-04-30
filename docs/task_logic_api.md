# Task Logic Robot API

This document describes the high-level `robot.*` functions available inside
`scripts/task_logic.py`. These functions are intended for writing competition
task sequences in `run_task(robot)` without dealing directly with ROS actions,
ROS services, serial packets, CRCs, or STM32 reply parsing.

## Where The API Comes From

`task_runner.py` creates a `CompetitionRobot` object and passes it into:

```python
def run_task(robot) -> None:
    ...
```

The public methods on `robot` are implemented by:

- `scripts/goto_point.py`: map-frame navigation and STM32 motion action support.
- `scripts/competition_robot.py`: competition-level wrappers for STM32 commands,
  pose reads, ball detection reads, and sleeps that keep ROS callbacks alive.

Most functions block until the requested operation finishes or succeeds. STM32
commands are retried until they receive a successful gateway response, unless
ROS shuts down or a validation error is raised before sending the command.

All public `robot.*` functions accept `wait_sec=0.0`. This is an optional extra
wait after the function finishes normally. The wait keeps ROS callbacks spinning,
like `robot.sleep(...)`, and `0.0` means no extra wait.

## Coordinate And Unit Conventions

For the full coordinate-frame contract and conversion formulas, see
[`docs/coordinate_frames.md`](coordinate_frames.md). New coordinate math should
refer to that document.

`CompetitionRobot` also accepts `yaw_zero_map_degrees` as a ROS parameter. Keep
it equal to the `amcl_fusion` value so `ball.absolute_angle_deg` matches STM32
yaw.
With the default `0.0`, STM32 yaw `0 deg` means the robot faces map-up,
`90 deg` means map-left, and `-90 deg` means map-right.

- `robot.goto(...)` uses the ROS map frame in meters.
- `robot.get_pose()` returns the current ROS map-frame pose in meters and yaw in
  ROS map convention: `0 deg` is map `+x`/right, `90 deg` is map `+y`/up. This
  is not STM32 yaw.
- `robot.move(...)` sends STM32 `cmd_dis` directly, so its `x_cm` and `y_cm`
  values are field-fixed STM32 centimeters: `x_cm > 0` moves map-left and
  `y_cm > 0` moves map-down. The wrapper always sends a `speed_profile`
  argument; the default is `1`.
- `robot.drive(...)` sends STM32 `cmd_dkmotor` directly. Its motion angle uses
  the firmware convention: `0 deg` is robot front, `90 deg` is robot left.
- STM32 yaw angles used by `robot.turn(...)` are firmware yaw angles in degrees:
  `0 deg` is map-up, `90 deg` is map-left, and `-90 deg` is map-right.

## Navigation And Motion

### `robot.goto(...)`

```python
robot.goto(
    x_m=0.40,
    y_m=-0.60,
    goal_tolerance_m=None,
    max_step_m=None,
    settle_sec=None,
    max_iterations=None,
    goto_timeout_sec=None,
    pose_wait_timeout_sec=None,
    action_server_wait_sec=None,
    wait_sec=0.0,
)
```

Drives to an absolute target in the ROS map frame. `x_m` and `y_m` are meters
from `/amcl_pose`.

The function repeatedly reads the current pose, computes the remaining map-frame
delta, limits each movement to `max_step_m`, converts that step into STM32
`cmd_dis` centimeters, sends the motion goal through `/stm32/motion`, waits for
the STM32 motion result, then replans from the latest pose after `settle_sec`.

Defaults, when optional parameters are omitted:

- `goal_tolerance_m=0.02`
- `max_step_m=0.80`
- `settle_sec=0.7`
- `max_iterations=25`
- `goto_timeout_sec=40.0`
- `pose_wait_timeout_sec=5.0`
- `action_server_wait_sec=10.0`
- `wait_sec=0.0`

Returns `True` when the target is reached. Raises `GotoError` if the target
cannot be reached before the timeout or iteration limit, or if the motion action
server cannot be used safely.

### `robot.move(...)`

```python
robot.move(
    x_cm=10,
    y_cm=0,
    speed_profile=1,
    retry_delay_sec=None,
    timeout_sec=None,
    wait_sec=0.0,
)
```

Sends STM32 `cmd_dis <x_cm> <y_cm> <speed_profile>` directly through the
`/stm32/motion` action. This is a field-fixed relative movement in centimeters:
`x_cm > 0` is map-left and `y_cm > 0` is map-down. It is not robot-front/left
body motion. During the movement, the STM32 firmware holds the current yaw.

`speed_profile` must be `0`, `1`, or `2`: `0` is faster acceleration, `1` is the
normal/default profile, and `2` is smoother acceleration.

Use this when you want the exact firmware-level relative movement command. Use
`robot.goto(...)` when you want absolute map-frame navigation using AMCL pose
feedback.

Returns `True` after the gateway receives the STM32 motion `done` reply. Retries
after failures using `retry_delay_sec`; each motion attempt uses `timeout_sec`.
If omitted, those use the robot's configured defaults.

### `robot.turn(...)`

```python
robot.turn(angle_deg=90, retry_delay_sec=None, timeout_sec=None, wait_sec=0.0)
```

Sends STM32 `cmd_turn <angle_deg>` through the `/stm32/motion` action. The target
is an absolute STM32 yaw angle in degrees: `0 deg` is map-up, `90 deg` is
map-left, and `-90 deg` is map-right. The firmware normalizes the target angle.

Returns `True` after the STM32 reports the turn is done. Retries until success
or ROS shutdown.

### `robot.drive(...)`

```python
robot.drive(speed_percent=50, move_angle_deg=90)
robot.drive(speed_percent=50, move_angle_deg=90, head_lock=True)
robot.drive(speed_percent=0, move_angle_deg=0, wait_sec=0.0)
```

Sends STM32 `cmd_dkmotor <speed_percent> <move_angle_deg> [head_lock]` through
the `/stm32/send_command` service. This enters the firmware continuous velocity
mode.

Parameters:

- `speed_percent`: integer `0-100`. Firmware maps this to chassis speed.
- `move_angle_deg`: movement direction in degrees. `0` is robot front, `90` is
  robot left.
- `head_lock`: optional. `True` or `1` keeps the current heading while
  translating. `False` or `0` lets the firmware turn toward the movement angle
  and drive forward. If omitted, the firmware default is used.

This command only waits for the STM32 ACK that the continuous motion command was
accepted. It does not wait for a `done` reply because `cmd_dkmotor` is continuous
mode. Stop continuous movement with `robot.stop()` or by sending
`robot.drive(speed_percent=0, move_angle_deg=0)`.

### `robot.stop()`

```python
robot.stop(wait_sec=0.0)
```

Sends STM32 `cmd_juststop`. This stops current continuous chassis motion while
keeping the STM32 chassis motion feature enabled and leaving yaw hold active.

Returns `True` after a successful ACK.

## STM32 Motion Enable And Reset

### `robot.motion_enable()` / `robot.motion_disable()`

```python
robot.motion_enable(wait_sec=0.0)
robot.motion_disable(wait_sec=0.0)
```

Send STM32 `cmd_conmotion 1` or `cmd_conmotion 0`.

`motion_enable()` allows STM32 motion commands and idle yaw-hold output.
`motion_disable()` immediately stops chassis motor output and disables the
default yaw-hold output. Both return `True` after a successful ACK.

### `robot.reset_yaw()`

```python
robot.reset_yaw(wait_sec=0.0)
```

Sends STM32 `cmd_anglecal`. This performs the same yaw zeroing operation as the
firmware's BNO key yaw reset.

Returns `True` after the command succeeds. The firmware may reject this command
as busy if it does not yet have valid BNO085 yaw data; the wrapper retries until
success.

### `robot.reset_mcu()`

```python
robot.reset_mcu(wait_sec=0.0)
```

Sends STM32 `cmd_mcureset`, which asks the firmware to perform a software reset.
The function returns `True` once the gateway receives the success reply before
the MCU resets.

Use this carefully: after reset, the STM32 serial connection and firmware state
will need time to come back.

## Suction Motor

### `robot.suck(...)`

```python
robot.suck(speed_percent=50, wait_sec=0.0)
```

Sends STM32 `cmd_suck <speed_percent>`. `speed_percent` must be in `0-100`.

Returns `True` after a successful ACK.

### `robot.suck_on(...)`

```python
robot.suck_on()
robot.suck_on(speed_percent=15, wait_sec=0.0)
```

Convenience wrapper for `robot.suck(...)`. The default is `100%` if no speed is
specified.

### `robot.suck_off()`

```python
robot.suck_off(wait_sec=0.0)
```

Convenience wrapper for `robot.suck(speed_percent=0)`.

### `robot.is_ball_detected()`

```python
detected = robot.is_ball_detected(wait_sec=0.0)
```

Sends STM32 `cmd_xqcx` and returns `True` when the PB15/xqwd suction
microswitch reports that a ball is detected.

### `robot.set_relay()` / `robot.relay_on()` / `robot.relay_off()`

```python
robot.set_relay(enabled=True, wait_sec=0.0)
robot.relay_on(wait_sec=0.0)
robot.relay_off(wait_sec=0.0)
```

Sends STM32 `cmd_dct 1` or `cmd_dct 0` to control the PD0/JD1 relay output.

## STM32 State And Infrared Sensor

### `robot.request_state()`

```python
state = robot.request_state(wait_sec=0.0)
print(state.dx, state.dy, state.dtheta, state.theta)
```

Sends STM32 `cmd_request` and returns a `Stm32State` object:

- `state.dx`: STM32 odometry x delta since the previous `cmd_request`, in cm.
- `state.dy`: STM32 odometry y delta since the previous `cmd_request`, in cm.
- `state.dtheta`: yaw delta since the previous `cmd_request`, in degrees.
- `state.theta`: current STM32 yaw, in degrees.
- `state.attempts`: gateway send attempts used for this command.
- `state.message`: gateway summary text.

`state.dx/state.dy` use the same field-fixed axes as `cmd_dis`: positive `dx`
is map-left and positive `dy` is map-down. `state.theta` is STM32 yaw, not ROS
map yaw.

The firmware defines the first request as the reference point, so its deltas are
normally zero.

### `robot.read_infrared()`

```python
ir = robot.read_infrared(wait_sec=0.0)
print(ir.channel)
```

Sends STM32 `cmd_infred` and returns a `Stm32Infrared` object:

- `ir.channel`: strongest BE-1732 infrared channel, from `1` to `7`.
- `ir.attempts`: gateway send attempts used for this command.
- `ir.message`: gateway summary text.

The wrapper validates that the returned channel is in range.

### `robot.infrared_channel()`

```python
channel = robot.infrared_channel(wait_sec=0.0)
```

Convenience wrapper for `robot.read_infrared().channel`. Use this when only the
channel number matters.

### `robot.set_infrared_mode(...)`

```python
robot.set_infrared_mode(mode="pt", wait_sec=0.0)
robot.set_infrared_mode(mode="tz", wait_sec=0.0)
```

Sends STM32 `cmd_infred_mode <mode>`.

Modes:

- `"pt"`: plain detection mode.
- `"tz"`: modulated detection mode. This is the firmware default.

Returns `True` after a successful ACK.

### `robot.infrared_plain_mode()` / `robot.infrared_modulated_mode()`

```python
robot.infrared_plain_mode(wait_sec=0.0)
robot.infrared_modulated_mode(wait_sec=0.0)
```

Convenience wrappers for:

- `robot.set_infrared_mode(mode="pt")`
- `robot.set_infrared_mode(mode="tz")`

## Pose And Vision

### `robot.get_pose(...)`

```python
pose = robot.get_pose()
pose = robot.get_pose(timeout_sec=1.0, wait_sec=0.0)
print(pose.x_m, pose.y_m, pose.yaw_deg)
```

Reads the latest `/amcl_pose` pose and returns a `RobotPose` object:

- `pose.x_m`: map-frame x position in meters.
- `pose.y_m`: map-frame y position in meters.
- `pose.yaw_rad`: map-frame yaw in radians.
- `pose.yaw_deg`: ROS map-frame yaw in degrees. `0 deg` is map-right/+x and
  `90 deg` is map-up/+y; this is not STM32 yaw.

If no pose is available before `timeout_sec`, the function raises `GotoError`.
If `timeout_sec` is omitted, it waits until a pose is available.

### `robot.find_ball(...)`

```python
ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5, wait_sec=0.0)
if ball is not None:
    robot.goto(x_m=ball.absolute_x_m, y_m=ball.absolute_y_m)
```

Reads `/orange_ball_detector/detection` and returns the latest valid ball
detection, or `None` if no valid detection arrives before `timeout_sec`.

The detection must satisfy:

- The detector says `detected=True`.
- `confidence >= min_confidence`.
- A robot yaw from `/amcl_pose` is available, because the wrapper converts the
  local ball offset into map-axis coordinates.
- For nonzero timeouts, the detection must be received after the function call
  starts, so old detections are not reused accidentally.

Returns a `BallDetection` object:

- `ball.x_m`, `ball.y_m`, `ball.z_m`: ball offset from the robot, rotated into
  the map axes, in meters.
- `ball.absolute_x_m`, `ball.absolute_y_m`, `ball.absolute_z_m`: ball position
  in the map frame, in meters.
- `ball.local_x_m`, `ball.local_y_m`, `ball.local_z_m`: raw orange-ball detector
  coordinates, in meters. Detector `+x` is camera-image down / robot rear, and
  detector `+y` is camera-image left / robot left.
- `ball.base_x_m`, `ball.base_y_m`, `ball.base_z_m`: ball position converted to
  robot `base_link`, in meters. `+x` is robot front, and `+y` is robot left.
- `ball.angle_deg`: relative ball bearing in the robot/base-link frame, in degrees.
  `0 deg` is robot front, `90 deg` is robot left, and `-90 deg` is robot right.
- `ball.absolute_angle_deg`: STM32 firmware yaw target toward the ball, in degrees.
  This is the competition "absolute angle"; use it with
  `robot.turn(angle_deg=ball.absolute_angle_deg)`.
- `ball.confidence`: detector confidence.
- `ball.stamp_sec`: detection message timestamp in seconds.

To move toward the ball's map position, use `ball.absolute_x_m` and
`ball.absolute_y_m`, as shown above.

## Waiting

### `robot.sleep(...)`

```python
robot.sleep(duration_sec=1.0, wait_sec=0.0)
```

First calls `robot.stop()` and waits for the STM32 `cmd_juststop` ACK, then waits
for `duration_sec` while still spinning ROS callbacks. This means pose and ball
detection subscribers continue updating during the wait.

Use this instead of `time.sleep(...)` inside `run_task(robot)` when you want ROS
callbacks to keep running.

## Error Handling Notes

- Invalid arguments, such as out-of-range speed percentages or non-finite
  coordinates, raise `ValueError` before sending any command.
- Navigation failures raise `GotoError`.
- STM32 service commands retry until a success response, so a persistent firmware
  `busy` reply will keep the task waiting and retrying.
- `cmd_dkmotor` continuous drive commands only wait for acceptance, not for
  movement completion.
- `robot.sleep(...)` intentionally sends `cmd_juststop` before waiting.
