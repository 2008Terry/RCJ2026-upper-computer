from __future__ import annotations
# from turtle import goto


# Edit only run_task(robot) for normal competition programming.
# Full API documentation: docs/task_logic_api.md
# Prefer keyword arguments such as x_m=-0.40 and angle_deg=90, so each number
# keeps its meaning at the call site.
#
# Available functions:
# - robot.goto(x_m=..., y_m=..., goal_tolerance_m=None, max_step_m=None,
#              settle_sec=None, max_iterations=None, goto_timeout_sec=None,
#              pose_wait_timeout_sec=None, action_server_wait_sec=None)
#     Go to an absolute map-frame target. x_m/y_m are meters from /amcl_pose.
#     If the optional parameters are omitted, defaults match the old
#     task_sequence.py settings:
#     goal_tolerance_m=0.02, max_step_m=0.8, settle_sec=0.7,
#     max_iterations=25, goto_timeout_sec=40.0,
#     pose_wait_timeout_sec=5.0, action_server_wait_sec=10.0.
#     Override per call when needed, for example:
#     robot.goto(x_m=0.40, y_m=-0.60, goal_tolerance_m=0.03, max_step_m=0.5)
# - robot.turn(angle_deg=..., retry_delay_sec=None, timeout_sec=None)
#     Turn to an STM32 yaw angle in degrees. Retries until the gateway reports
#     motion done.
# - robot.suck(speed_percent=..., retry_delay_sec=None, timeout_sec=None)
#     Set suction speed. speed_percent is 0-100. Returns after a successful ACK.
# - robot.suck_on(speed_percent=100)
#     Turn suction on. Default is 100%.
# - robot.suck_off()
#     Turn suction off. Equivalent to robot.suck(speed_percent=0).
# - robot.reset_yaw()
#     Reset STM32 yaw zero. Sends cmd_anglecal.
# - robot.reset_mcu()
#     Reset the STM32. Sends cmd_mcureset.
# - robot.motion_enable() / robot.motion_disable()
#     Enable or disable STM32 chassis motion. Sends cmd_conmotion 1/0.
# - robot.stop()
#     Stop continuous chassis motion while keeping the yaw-hold loop active.
#     Sends cmd_juststop.
# - robot.request_state()
#     Read the latest STM32 motion error/state. Returns
#     state.dx/state.dy/state.dtheta/state.theta.
# - robot.get_pose(timeout_sec=None)
#     Read the latest robot pose in the map/world frame. Returns
#     pose.x_m/pose.y_m in meters and pose.yaw_deg/pose.yaw_rad. yaw_deg uses
#     ROS map convention: 0 deg is +x, 90 deg is +y.
# - robot.find_ball(timeout_sec=1.0, min_confidence=0.0)
#     Reads /orange_ball_detector/detection, where detected/confidence/position
#     are published together from the same camera frame. Returns the latest valid
#     detection, or None if no ball is detected before timeout_sec. The returned
#     object has ball.x_m/ball.y_m/ball.z_m as map-axis relative offsets from
#     the robot. Add them to the current /amcl_pose x/y to get the ball's
#     absolute map position. ball.local_x_m/local_y_m/local_z_m keep the raw
#     base_link detection, and ball.confidence is the detector confidence.
#     Example: ball = robot.find_ball(timeout_sec=1.0)
#     if ball is not None:
#         pose = robot.get_pose()
#         robot.goto(x_m=pose.x_m + ball.x_m, y_m=pose.y_m + ball.y_m)
# - robot.move(x_cm=..., y_cm=..., retry_delay_sec=None, timeout_sec=None)
#     Send STM32 cmd_dis directly. x_cm/y_cm are relative odometry-frame
#     centimeters, matching the firmware interface document.
# - robot.sleep(duration_sec=...)
#     First calls robot.stop(), waits for the cmd_juststop ACK, then waits for
#     duration_sec while still spinning ROS callbacks. run_task does not execute
#     the next command during this wait. The chassis should stay in STM32 IDLE:
#     yaw hold remains active, but no translation command is active.
# - robot.drive(speed_percent=..., move_angle_deg=..., head_lock=None,
#               retry_delay_sec=None, timeout_sec=None)
#     Send STM32 cmd_dkmotor for continuous velocity control. speed_percent is
#     0-100, move_angle_deg is 0 front / 90 left, head_lock can be True/False.
#     Stop it with robot.stop() or robot.drive(speed_percent=0, move_angle_deg=0).
# - robot.read_infrared()
#     Send cmd_infred and return an object with channel/attempts/message.
# - robot.infrared_channel()
#     Send cmd_infred and return only the strongest infrared channel, 1-7.
# - robot.set_infrared_mode(mode="pt" or "tz")
#     Send cmd_infred_mode. "pt" is plain mode, "tz" is modulated mode.
# - robot.infrared_plain_mode() / robot.infrared_modulated_mode()
#     Convenience wrappers for cmd_infred_mode pt/tz.
#


def run_task(robot) -> None:
    """Edit this function to write the competition task sequence."""

    # robot.reset_yaw()
    # robot.motion_enable()

    # Example task sequence. Coordinates are absolute map-frame meters.
    # ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
    # if ball is not None:
    #     print(ball.x_m, ball.y_m, ball.z_m, ball.confidence)

    # pose = robot.get_pose()
    # print(pose.x_m, pose.y_m, pose.yaw_deg)
    
    robot.suck_on(speed_percent=0)
    robot.motion_disable()
    
    
    

    # while()
    #     random()

    #     goto()

    #     take picture

    # robot.sleep(duration_sec=1.0)
    # robot.suck_on(speed_percent=0)
    
    # robot.goto(x_m=-0.40, y_m=-0.60)
    # robot.goto(x_m=0.40, y_m=-0.60)
    # robot.goto(x_m=-0.40, y_m=0.60)
    # robot.goto(x_m=0.40, y_m=0.60)
    
    # robot.turn(angle_deg=-90)
    # robot.suck_off()
