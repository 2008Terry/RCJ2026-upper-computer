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
  values are field-fixed world-frame centimeters: `x_cm > 0` moves map-up and
  `y_cm > 0` moves map-left. The wrapper always sends a `speed_profile`
  argument; the default is `1`.
- `robot.drive(...)` sends STM32 `cmd_dkmotor` directly. Its motion angle uses
  the firmware convention: `0 deg` is robot front, `90 deg` is robot left.
- STM32 yaw angles used by `robot.turn(...)` are firmware yaw angles in degrees:
  `0 deg` is map-up, `90 deg` is map-left, and `-90 deg` is map-right.

## Quick Usage Recipes

Use these as starting templates inside `run_task(robot)`.

### Enable Motion Before A Task

```python
def run_task(robot) -> None:
    robot.motion_enable()
    robot.suck_off()

    robot.goto(x_m=0.30, y_m=0.20)
```

### Go Through Several Map Points

```python
robot.motion_enable()

robot.goto(x_m=-0.40, y_m=-0.60)
robot.turn(angle_deg=90)
robot.goto(x_m=0.40, y_m=-0.60)
robot.turn_to_point(x_m=0.0, y_m=0.0)
```

### Find A Ball, Face It, Approach It, Then Chase And Suck

```python
ball = robot.find_ball(timeout_sec=0.5, min_confidence=0.5)
if ball is None:
    ball = robot.spin_find_ball(min_confidence=0.5)

if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
    robot.goto_ball_standoff(ball, stand_off_m=0.30)

    robot.suck_on(speed_percent=15)
    robot.reset_ball_sucked_detector(
        required_detected_count=3,
        sample_interval_sec=0.05,
    )

    while True:
        ball = robot.find_ball(timeout_sec=0.05, min_confidence=0.5)
        if ball is not None:
            robot.drive(
                speed_percent=10,
                move_angle_deg=ball.angle_deg,
                head_lock=False,
            )

        if robot.poll_ball_sucked(stop_on_success=True):
            break
else:
    print("Ball not found")
```

In the chase loop, use `ball.angle_deg` for `robot.drive(...)`, because
`drive()` expects a robot-relative direction. Use `ball.absolute_angle_deg` only
with `robot.turn(...)`.

### Drive Continuously For A Fixed Time

```python
robot.drive(speed_percent=20, move_angle_deg=0, head_lock=True)
robot.timer(duration_sec=1.0)
robot.stop()
```

Use `robot.timer(...)` here, not `robot.sleep(...)`, because `sleep()` sends
`robot.stop()` before waiting.

### Release A Ball And Confirm It Is Gone

```python
robot.stop()
robot.suck_off()

released = robot.wait_for_ball_released(
    timeout_sec=2.0,
    required_empty_count=3,
    sample_interval_sec=0.05,
)
if released:
    print("Released")
```

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
    speed_profile=1,
    wait_sec=0.0,
)
```

Drives to an absolute target in the ROS map frame. `x_m` and `y_m` are meters
from `/amcl_pose`.

The function repeatedly reads the current pose, computes the remaining map-frame
delta, limits each movement to `max_step_m`, converts that step into STM32
`cmd_dis` centimeters, sends the motion goal through `/stm32/motion`, waits for
the STM32 motion result, then replans from the latest pose after `settle_sec`.

`speed_profile` controls the STM32 `cmd_dis` speed profile for every segment:
`0` is faster acceleration, `1` is the normal/default profile, and `2` is
smoother acceleration.

Defaults, when optional parameters are omitted:

- `goal_tolerance_m=0.02`
- `max_step_m=0.80`
- `settle_sec=0.7`
- `max_iterations=25`
- `goto_timeout_sec=40.0`
- `pose_wait_timeout_sec=5.0`
- `action_server_wait_sec=10.0`
- `speed_profile=1`
- `wait_sec=0.0`

Returns `True` when the target is reached. Raises `GotoError` if the target
cannot be reached before the timeout or iteration limit, or if the motion action
server cannot be used safely.

Use this for normal map navigation, especially when you trust `/amcl_pose` and
want the robot to replan from pose feedback.

Example:

```python
robot.goto(x_m=0.20, y_m=0.40)
robot.goto(
    x_m=0.60,
    y_m=0.10,
    goal_tolerance_m=0.04,
    max_step_m=0.30,
    speed_profile=2,
)
```

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
`x_cm > 0` is map-up and `y_cm > 0` is map-left. It is not robot-front/left
body motion. During the movement, the STM32 firmware holds the current yaw.

`speed_profile` must be `0`, `1`, or `2`: `0` is faster acceleration, `1` is the
normal/default profile, and `2` is smoother acceleration.

Use this when you want the exact firmware-level relative movement command. Use
`robot.goto(...)` when you want absolute map-frame navigation using AMCL pose
feedback.

Returns `True` after the gateway receives the STM32 motion `done` reply. Retries
after failures using `retry_delay_sec`; each motion attempt uses `timeout_sec`.
If omitted, those use the robot's configured defaults.

Example:

```python
# Move 20 cm toward map-up while holding the current yaw.
robot.move(x_cm=20, y_cm=0, speed_profile=1)

# Move 10 cm toward map-left.
robot.move(x_cm=0, y_cm=10, speed_profile=2)
```

### `robot.turn(...)`

```python
robot.turn(angle_deg=90, retry_delay_sec=None, timeout_sec=None, wait_sec=0.0)
```

Sends STM32 `cmd_turn <angle_deg>` through the `/stm32/motion` action. The target
is an absolute STM32 yaw angle in degrees: `0 deg` is map-up, `90 deg` is
map-left, and `-90 deg` is map-right. The firmware normalizes the target angle.

Returns `True` after the STM32 reports the turn is done. Retries until success
or ROS shutdown.

Use this when you already know the desired STM32 yaw, or when using
`ball.absolute_angle_deg`.

Example:

```python
robot.turn(angle_deg=0)    # face map-up
robot.turn(angle_deg=90)   # face map-left
robot.turn(angle_deg=-90)  # face map-right

ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
```

### `robot.turn_to_point(...)`

```python
robot.turn_to_point(x_m=0.40, y_m=-0.60)
robot.turn_to_point(x_m=0.40, y_m=-0.60, min_distance_m=0.02)
```

Turns in place to face an absolute ROS map-frame point. `x_m` and `y_m` are map
coordinates in meters, using the same frame as `robot.goto(...)`.

The wrapper reads the current robot pose, computes the map direction from the
robot to the target point, converts that ROS map angle into the STM32 yaw
convention, then calls `robot.turn(...)`.

The calculation is:

```text
dx = target_x_m - current_x_m
dy = target_y_m - current_y_m

ros_map_angle_deg = atan2(dy, dx)
stm32_yaw_deg = normalize(
    ros_map_angle_deg - 90 - yaw_zero_map_degrees
)
```

Returns `True` after the turn succeeds. Returns `False` without turning if the
target is closer than `min_distance_m`, because the facing direction is then not
stable enough to define.

Use this when you know a point on the map and want the robot to face it without
manually calculating yaw.

Example:

```python
# Face the map origin.
robot.turn_to_point(x_m=0.0, y_m=0.0)

# Drive somewhere, then face a scoring area or another known map point.
robot.goto(x_m=0.30, y_m=-0.20)
robot.turn_to_point(x_m=-0.50, y_m=0.60)
```

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

Use this for short continuous behaviors such as final ball chasing. If you want
the chassis to keep facing its current heading while sliding, use
`head_lock=True`. If you want the firmware to turn toward the movement direction,
use `head_lock=False`.

Example:

```python
# Drive forward for 1 second, then stop.
robot.drive(speed_percent=20, move_angle_deg=0, head_lock=True)
robot.timer(duration_sec=1.0)
robot.stop()

# Chase a detected ball using robot-relative angle.
ball = robot.find_ball(timeout_sec=0.1, min_confidence=0.5)
if ball is not None:
    robot.drive(
        speed_percent=10,
        move_angle_deg=ball.angle_deg,
        head_lock=False,
    )
```

### `robot.stop()`

```python
robot.stop(wait_sec=0.0)
```

Sends STM32 `cmd_juststop`. This stops current continuous chassis motion while
keeping the STM32 chassis motion feature enabled and leaving yaw hold active.

Returns `True` after a successful ACK.

Example:

```python
robot.drive(speed_percent=15, move_angle_deg=0, head_lock=True)
robot.timer(duration_sec=0.5)
robot.stop()
```

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

Typical use:

```python
def run_task(robot) -> None:
    robot.motion_enable()
    robot.goto(x_m=0.20, y_m=0.20)
```

### `robot.reset_yaw()`

```python
robot.reset_yaw(wait_sec=0.0)
```

Sends STM32 `cmd_anglecal`. This performs the same yaw zeroing operation as the
firmware's BNO key yaw reset.

Returns `True` after the command succeeds. The firmware may reject this command
as busy if it does not yet have valid BNO085 yaw data; the wrapper retries until
success.

Use this only when you intentionally want the current robot heading to become
the STM32 yaw zero reference.

Example:

```python
robot.reset_yaw()
robot.turn(angle_deg=0)
```

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

Example:

```python
robot.suck(speed_percent=40)
robot.timer(duration_sec=0.5)
robot.suck(speed_percent=0)
```

### `robot.suck_on(...)`

```python
robot.suck_on()
robot.suck_on(speed_percent=15, wait_sec=0.0)
```

Convenience wrapper for `robot.suck(...)`. The default is `100%` if no speed is
specified.

Example:

```python
robot.suck_on(speed_percent=15)
```

### `robot.suck_off()`

```python
robot.suck_off(wait_sec=0.0)
```

Convenience wrapper for `robot.suck(speed_percent=0)`.

Example:

```python
robot.suck_off()
```

### `robot.is_ball_detected()`

```python
detected = robot.is_ball_detected(wait_sec=0.0)
```

Sends STM32 `cmd_xqcx` and returns `True` when the PB15/xqwd suction
microswitch reports that a ball is detected.

Use this for a one-shot raw read. For task decisions, prefer
`robot.wait_for_ball_sucked(...)` or `robot.poll_ball_sucked(...)` because they
debounce the switch.

Example:

```python
if robot.is_ball_detected():
    print("Switch says ball is present")
```

### `robot.wait_for_ball_sucked(...)`

```python
caught = robot.wait_for_ball_sucked(
    timeout_sec=3.0,
    required_detected_count=3,
    sample_interval_sec=0.05,
    stop_on_success=True,
)
```

Robust suction-state detector for the binary suction microswitch. It repeatedly
calls `robot.is_ball_detected()` and uses a small state machine:

- `empty`: the latest stable state is no ball.
- `candidate`: one or more detected samples have appeared, but not enough to
  confirm the ball.
- `confirmed`: `required_detected_count` consecutive detected samples have been
  observed.

The function returns `True` only after the `confirmed` state is reached. If the
signal drops back to not detected before enough consecutive samples are seen,
the streak resets to `empty`. It returns `False` when `timeout_sec` expires.

Set `stop_on_success=True` when this function is used during a slow approach;
the robot will send `robot.stop()` as soon as the suction state is confirmed.

Use this when the robot does not need to keep updating `drive()` inside your own
loop.

Example:

```python
robot.suck_on(speed_percent=30)
robot.drive(speed_percent=12, move_angle_deg=0, head_lock=True)

caught = robot.wait_for_ball_sucked(
    timeout_sec=2.0,
    required_detected_count=3,
    stop_on_success=True,
)
if not caught:
    robot.stop()
```

### Non-Blocking Suction Poll

```python
robot.reset_ball_sucked_detector(
    required_detected_count=3,
    sample_interval_sec=0.05,
)

while True:
    ball = robot.find_ball(timeout_sec=0.05, min_confidence=0.5)
    if ball is not None:
        robot.drive(
            speed_percent=10,
            move_angle_deg=ball.angle_deg,
            head_lock=False,
        )

    if robot.poll_ball_sucked(stop_on_success=True):
        break
```

Use this pattern when your task loop must keep doing other work, such as
continuously steering with `robot.drive(...)`. Call
`robot.reset_ball_sucked_detector(...)` once before the loop. Then call
`robot.poll_ball_sucked(...)` once per loop iteration.

`poll_ball_sucked()` does not own the loop and does not sleep until timeout. It
only samples the microswitch when `sample_interval_sec` has elapsed; otherwise
it returns the current confirmed/not-confirmed state immediately. Once
`required_detected_count` consecutive detected samples are seen, it returns
`True`. With `stop_on_success=True`, it also sends `robot.stop()` at that moment.

Use this with vision-guided chasing, where each loop iteration may send a new
`robot.drive(...)` command based on the current ball angle.

### `robot.wait_for_ball_released(...)`

```python
released = robot.wait_for_ball_released(
    timeout_sec=3.0,
    required_empty_count=3,
    sample_interval_sec=0.05,
)
```

Robust release/drop detector for the same binary suction microswitch. It is the
reverse of `robot.wait_for_ball_sucked(...)`: the function returns `True` only
after `required_empty_count` consecutive not-detected samples.

The state machine is:

- `sucked`: the latest stable state is ball present.
- `release_candidate`: one or more empty samples have appeared, but not enough
  to confirm release/drop.
- `released`: `required_empty_count` consecutive empty samples have been
  observed.

Use this after intentionally turning suction off, or while carrying a ball if
you want to detect a real drop instead of reacting to a single noisy sample.

Example:

```python
robot.suck_off()
if robot.wait_for_ball_released(timeout_sec=2.0, required_empty_count=3):
    print("Release confirmed")
```

### `robot.set_relay()` / `robot.relay_on()` / `robot.relay_off()`

```python
robot.set_relay(enabled=True, wait_sec=0.0)
robot.relay_on(wait_sec=0.0)
robot.relay_off(wait_sec=0.0)
```

Sends STM32 `cmd_dct 1` or `cmd_dct 0` to control the PD0/JD1 relay output.

Example:

```python
robot.relay_on()
robot.timer(duration_sec=0.5)
robot.relay_off()
```

## STM32 State And Infrared Sensor

### `robot.request_state()`

```python
state = robot.request_state(wait_sec=0.0)
print(state.dx, state.dy, state.dtheta, state.theta)
```

Sends STM32 `cmd_request` and returns a `Stm32State` object:

- `state.dx`: world-frame x delta since the previous `cmd_request`, in cm.
- `state.dy`: world-frame y delta since the previous `cmd_request`, in cm.
- `state.dtheta`: yaw delta since the previous `cmd_request`, in degrees.
- `state.theta`: current STM32 yaw, in degrees.
- `state.attempts`: gateway send attempts used for this command.
- `state.message`: gateway summary text.

`state.dx/state.dy` use the same field-fixed world axes as `cmd_dis`: positive
`dx` is map-up and positive `dy` is map-left. `state.theta` is STM32 yaw, not
ROS map yaw.

The firmware defines the first request as the reference point, so its deltas are
normally zero.

Example:

```python
state = robot.request_state()
print(f"STM32 yaw={state.theta:.1f}, dx={state.dx:.1f}, dy={state.dy:.1f}")
```

### `robot.read_infrared()`

```python
ir = robot.read_infrared(wait_sec=0.0)
print(ir.channel)
```

Sends STM32 `cmd_infred` and returns a `Stm32Infrared` object:

- `ir.channel`: strongest BE-1732 infrared channel, from `1` to `7`; `-1`
  means the STM32 judged the ball to be behind the robot.
- `ir.is_behind`: `True` when `ir.channel == -1`.
- `ir.has_direction`: `True` when `ir.channel` is one of `1-7`.
- `ir.attempts`: gateway send attempts used for this command.
- `ir.message`: gateway summary text.

The wrapper validates that the returned channel is either `-1` or in `1-7`.
The offence task treats `-1` as a rear-ball cue when vision has no ball, and
backs up to search.

Example:

```python
ir = robot.read_infrared()
if ir.channel in (3, 4, 5):
    print("IR source is near the center channels")
```

### `robot.infrared_channel()`

```python
channel = robot.infrared_channel(wait_sec=0.0)
```

Convenience wrapper for `robot.read_infrared().channel`. Use this when only the
channel number matters. It may return `-1` for a rear-ball cue, or `1-7` for a
directional infrared channel.

Example:

```python
channel = robot.infrared_channel()
print(channel)
```

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

Example:

```python
robot.infrared_modulated_mode()
channel = robot.infrared_channel()

robot.infrared_plain_mode()
```

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

Use this when you need to make a decision from the robot's current map position.
Use `robot.goto(...)` for navigation instead of manually calculating `move(...)`
commands from pose.

Example:

```python
pose = robot.get_pose(timeout_sec=1.0)
print(f"x={pose.x_m:.2f}, y={pose.y_m:.2f}, yaw={pose.yaw_deg:.1f}")

if pose.y_m < 0.0:
    robot.goto(x_m=pose.x_m, y_m=0.0)
```

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

Use this for a single vision read. If it returns `None`, you can call
`robot.spin_find_ball(...)` to scan in place.

Examples:

```python
ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
if ball is not None:
    print(f"relative angle={ball.angle_deg:.1f}")
    print(f"turn target={ball.absolute_angle_deg:.1f}")
```

```python
# Turn to face the ball.
ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
```

```python
# Chase the ball with continuous drive.
ball = robot.find_ball(timeout_sec=0.05, min_confidence=0.5)
if ball is not None:
    robot.drive(
        speed_percent=10,
        move_angle_deg=ball.angle_deg,
        head_lock=False,
    )
```

### `robot.goto_ball_standoff(...)`

```python
ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
    robot.goto_ball_standoff(ball, stand_off_m=0.12)

robot.goto_ball_standoff(stand_off_m=0.15, speed_profile=2)
```

Moves to a map-frame target that stops before the detected ball by
`stand_off_m` meters. If `ball` is omitted, the function calls
`robot.find_ball(timeout_sec=detection_timeout_sec,
min_confidence=min_confidence)` first. It returns `False` when no usable ball is
available, otherwise it calls `robot.goto(...)` and returns `True`.

The target point is calculated on the line from the robot to the ball:

```text
distance = hypot(ball.x_m, ball.y_m)
direction_x = ball.x_m / distance
direction_y = ball.y_m / distance

target_x_m = ball.absolute_x_m - direction_x * stand_off_m
target_y_m = ball.absolute_y_m - direction_y * stand_off_m
```

`ball.x_m/y_m` are the robot-to-ball offset in map axes, so this backs up from
the absolute ball position along the same map-frame direction. For example,
`stand_off_m=0.12` means the goto target is about 12 cm before the ball, from
the robot's current side.

`speed_profile` is passed through to `robot.goto(...)` and uses the same values:
`0` fast, `1` normal/default, or `2` smooth.

Use this before the final slow suction approach. It should not be the last
catching step; after reaching the standoff point, use `robot.drive(...)` and the
suction switch detector for the final few centimeters.

Example:

```python
ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
    if robot.goto_ball_standoff(ball, stand_off_m=0.25):
        robot.suck_on(speed_percent=15)
```

### `robot.spin_find_ball(...)`

```python
ball = robot.find_ball(timeout_sec=0.5, min_confidence=0.5)
if ball is None:
    ball = robot.spin_find_ball(
        step_deg=20.0,
        direction=1,
        max_turn_deg=360.0,
        min_confidence=0.5,
        confirm_settle_sec=0.5,
    )

if ball is not None:
    print(ball.base_x_m, ball.angle_deg, ball.absolute_angle_deg)
```

Searches for a ball by turning in place in small absolute-yaw steps. This uses
`robot.turn(...)`, so it does not translate the chassis. `direction > 0` scans
toward positive STM32 yaw, and `direction < 0` scans toward negative STM32 yaw.
Positive STM32 yaw is map-left with the default yaw convention.

After each step, the function waits `settle_sec`, then calls
`robot.find_ball(timeout_sec=detection_timeout_sec,
min_confidence=min_confidence)`. When a ball is found, it sends `robot.stop()`,
waits `confirm_settle_sec` so rotation and camera blur settle, reads the ball
again for up to `confirm_timeout_sec`, and returns that confirmed
`BallDetection`. If the confirm read fails, it returns the first detection as a
fallback. If no ball is found before `max_turn_deg` is swept, it returns `None`.

Use this when a direct `robot.find_ball(...)` call fails and you want the robot
to search without translating.

Example:

```python
ball = robot.find_ball(timeout_sec=0.5, min_confidence=0.5)
if ball is None:
    ball = robot.spin_find_ball(
        step_deg=15.0,
        direction=-1,
        max_turn_deg=270.0,
        min_confidence=0.5,
    )

if ball is not None:
    robot.turn(angle_deg=ball.absolute_angle_deg)
```

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

Use `robot.sleep(...)` only when you intentionally want the chassis stopped
during the wait.

Example:

```python
robot.stop()
robot.sleep(duration_sec=0.5)
```

### `robot.timer(...)`

```python
robot.drive(speed_percent=20, move_angle_deg=0, head_lock=True)
robot.timer(duration_sec=1.5)
robot.stop()
```

Waits for `duration_sec` while still spinning ROS callbacks, but does not send
`robot.stop()`. Use this when a continuous command such as `robot.drive(...)` or
suction should keep running for a fixed amount of time.

`robot.sleep(...)` is the stopping wait. `robot.timer(...)` is the non-stopping
wait.

Example:

```python
robot.suck_on(speed_percent=30)
robot.timer(duration_sec=0.2)

robot.drive(speed_percent=10, move_angle_deg=0, head_lock=False)
robot.timer(duration_sec=0.5)
robot.stop()
```

## Error Handling Notes

- Invalid arguments, such as out-of-range speed percentages or non-finite
  coordinates, raise `ValueError` before sending any command.
- Navigation failures raise `GotoError`.
- STM32 service commands retry until a success response, so a persistent firmware
  `busy` reply will keep the task waiting and retrying.
- `cmd_dkmotor` continuous drive commands only wait for acceptance, not for
  movement completion.
- `robot.sleep(...)` intentionally sends `cmd_juststop` before waiting.
- `robot.timer(...)` does not stop active continuous motion.
