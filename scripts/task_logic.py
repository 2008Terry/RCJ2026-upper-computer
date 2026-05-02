from __future__ import annotations
# from turtle import goto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from competition_robot import CompetitionRobot

# Edit only run_task(robot) for normal competition programming.
# Quick API documentation: docs/task_runner_quick.md
# Full API documentation: docs/task_logic_api.md
# Coordinate-frame reference: docs/coordinate_frames.md
# Keep CompetitionRobot yaw_zero_map_degrees equal to amcl_fusion's value when
# using ball.absolute_angle_deg.
# Prefer keyword arguments such as x_m=-0.40 and angle_deg=90, so each number
# keeps its meaning at the call site.
#
# Available functions:
# - robot.goto(x_m=..., y_m=..., goal_tolerance_m=None, max_step_m=None,
#              settle_sec=None, max_iterations=None, goto_timeout_sec=None,
#              pose_wait_timeout_sec=None, action_server_wait_sec=None,
#              speed_profile=1, wait_sec=0.0)
#     Go to an absolute map-frame target. x_m/y_m are meters from /amcl_pose.
#     speed_profile can be 0 fast, 1 normal, or 2 smooth.
#     If the optional parameters are omitted, defaults match the old
#     task_sequence.py settings:
#     goal_tolerance_m=0.02, max_step_m=0.8, settle_sec=0.7,
#     max_iterations=25, goto_timeout_sec=40.0,
#     pose_wait_timeout_sec=5.0, action_server_wait_sec=10.0,
#     speed_profile=1, wait_sec=0.0.
#     Override per call when needed, for example:
#     robot.goto(x_m=0.40, y_m=-0.60, goal_tolerance_m=0.03, max_step_m=0.5,
#                speed_profile=2)
#     wait_sec is an optional extra wait after the function finishes.
# - robot.turn(angle_deg=..., retry_delay_sec=None, timeout_sec=None, wait_sec=0.0)
#     Turn to an STM32 yaw angle in degrees. Retries until the gateway reports
#     motion done.
# - robot.turn_to_point(x_m=..., y_m=..., min_distance_m=0.01,
#                       pose_wait_timeout_sec=None, retry_delay_sec=None,
#                       timeout_sec=None, wait_sec=0.0)
#     Turn in place to face an absolute map-frame point. Returns False if the
#     target point is too close to the current robot position to define a stable
#     direction.
# - robot.suck(speed_percent=..., retry_delay_sec=None, timeout_sec=None,
#              wait_sec=0.0)
#     Set suction speed. speed_percent is 0-100. Returns after a successful ACK.
# - robot.suck_on(speed_percent=100, wait_sec=0.0)
#     Turn suction on. Default is 100%.
# - robot.suck_off(wait_sec=0.0)
#     Turn suction off. Equivalent to robot.suck(speed_percent=0).
# - robot.is_ball_detected(wait_sec=0.0)
#     Send cmd_xqcx and return True when the suction microswitch reports a ball.
# - robot.wait_for_ball_sucked(timeout_sec=3.0, required_detected_count=3,
#                              sample_interval_sec=0.05,
#                              stop_on_success=False, wait_sec=0.0)
#     Robust suction state detector. It repeatedly reads the suction
#     microswitch and returns True only after required_detected_count
#     consecutive detected samples. Returns False on timeout.
# - robot.reset_ball_sucked_detector(required_detected_count=3,
#                                    sample_interval_sec=0.05)
#     Reset the non-blocking suction debounce state machine before a catch loop.
# - robot.poll_ball_sucked(stop_on_success=False)
#     Non-blocking suction debounce poll for use inside your own while loop.
#     Returns True once the configured consecutive detected count is reached.
# - robot.wait_for_ball_released(timeout_sec=3.0, required_empty_count=3,
#                                sample_interval_sec=0.05,
#                                stop_on_success=False, wait_sec=0.0)
#     Robust release/drop detector. It repeatedly reads the suction microswitch
#     and returns True only after required_empty_count consecutive not-detected
#     samples. Returns False on timeout.
# - robot.set_relay(enabled=True/False, wait_sec=0.0)
#     Send cmd_dct 1/0 to control the PD0/JD1 relay.
# - robot.relay_on(wait_sec=0.0) / robot.relay_off(wait_sec=0.0)
#     Convenience wrappers for cmd_dct.
# - robot.reset_yaw(wait_sec=0.0)
#     Reset STM32 yaw zero. Sends cmd_anglecal.
# - robot.reset_mcu(wait_sec=0.0)
#     Reset the STM32. Sends cmd_mcureset.
# - robot.motion_enable(wait_sec=0.0) / robot.motion_disable(wait_sec=0.0)
#     Enable or disable STM32 chassis motion. Sends cmd_conmotion 1/0.
# - robot.stop(wait_sec=0.0)
#     Stop continuous chassis motion while keeping the yaw-hold loop active.
#     Sends cmd_juststop.
# - robot.request_state(wait_sec=0.0)
#     Read the latest STM32 motion error/state. Returns
#     state.dx/state.dy/state.dtheta/state.theta.
# - robot.get_pose(timeout_sec=None, wait_sec=0.0)
#     Read the latest robot pose in the map/world frame. Returns
#     pose.x_m/pose.y_m in meters and pose.yaw_deg/pose.yaw_rad. yaw_deg uses
#     ROS map convention: 0 deg is map-right/+x, 90 deg is map-up/+y. This is
#     not STM32 yaw.
# - robot.find_ball(timeout_sec=1.0, min_confidence=0.0, wait_sec=0.0)
#     Reads /orange_ball_detector/detection, where detected/confidence/position
#     are published together from the same camera frame. Returns the latest valid
#     detection, or None if no ball is detected before timeout_sec. The returned
#     object has ball.x_m/ball.y_m/ball.z_m as map-axis relative offsets from
#     the robot. ball.absolute_x_m/absolute_y_m/absolute_z_m are the ball's
#     absolute map-frame position. ball.local_x_m/local_y_m/local_z_m keep the
#     raw detector frame: +x is camera-image down, +y is camera-image left.
#     ball.base_x_m/base_y_m/base_z_m are converted to robot base_link:
#     +x front, +y left. ball.angle_deg is the relative ball bearing in the
#     robot frame: 0 is front, 90 is left, -90 is right.
#     ball.absolute_angle_deg is the STM32 yaw target to pass to
#     robot.turn(angle_deg=...). ball.confidence is the detector confidence.
#     Example: ball = robot.find_ball(timeout_sec=1.0)
#     if ball is not None:
#         print(ball.angle_deg, ball.absolute_angle_deg)
#         robot.turn(angle_deg=ball.absolute_angle_deg)
#         robot.goto(x_m=ball.absolute_x_m, y_m=ball.absolute_y_m)
# - robot.goto_ball_standoff(ball=None, stand_off_m=0.12,
#                            min_confidence=0.5, detection_timeout_sec=1.0,
#                            goal_tolerance_m=0.04, max_step_m=0.35,
#                            speed_profile=1, wait_sec=0.0)
#     Go to a map target that stops stand_off_m meters before the ball. If ball
#     is omitted, this function calls robot.find_ball(...) first. Returns False
#     if no ball is available.
# - robot.spin_find_ball(step_deg=20.0, direction=1, max_turn_deg=360.0,
#                        min_confidence=0.5, detection_timeout_sec=0.25,
#                        settle_sec=0.2, confirm_settle_sec=0.5,
#                        confirm_timeout_sec=1.0, wait_sec=0.0)
#     Search for a ball by turning in place in small absolute-yaw steps. When a
#     ball is found, stop, wait for confirm_settle_sec, read the ball again, and
#     return the confirmed BallDetection. Returns None if no ball is found.
# - robot.move(x_cm=..., y_cm=..., speed_profile=1, retry_delay_sec=None,
#              timeout_sec=None, wait_sec=0.0)
#     Send STM32 cmd_dis directly. x_cm/y_cm are field-fixed centimeters:
#     x_cm > 0 is map-up, y_cm > 0 is map-left. speed_profile can be
#     0 fast, 1 normal, or 2 smooth.
# - robot.sleep(duration_sec=..., wait_sec=0.0)
#     First calls robot.stop(), waits for the cmd_juststop ACK, then waits for
#     duration_sec while still spinning ROS callbacks. run_task does not execute
#     the next command during this wait. The chassis should stay in STM32 IDLE:
#     yaw hold remains active, but no translation command is active.
# - robot.timer(duration_sec=..., wait_sec=0.0)
#     Wait for duration_sec while still spinning ROS callbacks, without sending
#     robot.stop(). Use this to let continuous drive/suction keep running for a
#     fixed amount of time.
# - robot.drive(speed_percent=..., move_angle_deg=..., head_lock=None,
#               retry_delay_sec=None, timeout_sec=None, wait_sec=0.0)
#     Send STM32 cmd_dkmotor for continuous velocity control. speed_percent is
#     0-100, move_angle_deg is 0 front / 90 left, head_lock can be True/False.
#     Stop it with robot.stop() or robot.drive(speed_percent=0, move_angle_deg=0).
# - robot.read_infrared(wait_sec=0.0)
#     Send cmd_infred and return an object with channel/attempts/message.
# - robot.infrared_channel(wait_sec=0.0)
#     Send cmd_infred and return only the strongest infrared channel, 1-7.
# - robot.set_infrared_mode(mode="pt" or "tz", wait_sec=0.0)
#     Send cmd_infred_mode. "pt" is plain mode, "tz" is modulated mode.
# - robot.infrared_plain_mode(wait_sec=0.0) /
#   robot.infrared_modulated_mode(wait_sec=0.0)
#     Convenience wrappers for cmd_infred_mode pt/tz.
#


def run_task(robot: CompetitionRobot) -> None:
    """Edit this function to write the competition task sequence."""
    # robot.reset_yaw()
    # robot.motion_enable()
    # robot.suck_on(speed_percent=10)
    # robot.timer(duration_sec=5)
    # robot.suck_off()
    # robot.set_relay(enabled=True)
    # robot.relay_on()
    # robot.timer(duration_sec=3)
    # robot.relay_off()
    # robot.timer(duration_sec=100)
    

    # Example task sequence. Coordinates are absolute map-frame meters.
    # ball = robot.find_ball(timeout_sec=1.0, min_confidence=0.5)
    # if ball is not None:
    #     print(ball.x_m, ball.y_m, ball.z_m, ball.confidence)

    # pose = robot.get_pose()
    # print(pose.x_m, pose.y_m, pose.yaw_deg)
    
    # robot.suck_on(speed_percent=15)
    # # robot.motion_disable()
    # robot.sleep(duration_sec=10.0)
    # robot.suck_on(speed_percent=0)
    # robot.suck_off()
    
    # robot.reset_ball_sucked_detector(required_detected_count=5, sample_interval_sec=0.2)
    # while(1):
    #     if robot.poll_ball_sucked(stop_on_success=True):
    #         break
        
    # print("Ball sucked!")
    
    
    ### test dk move
    # robot.turn(angle_deg=0)
    # robot.drive(speed_percent=10, move_angle_deg=0, head_lock=True)
    # robot.timer(duration_sec=3)
    # robot.drive(speed_percent=10, move_angle_deg=90, head_lock=True)
    # robot.timer(duration_sec=3)
    # robot.drive(speed_percent=10, move_angle_deg=180, head_lock=True)
    # robot.timer(duration_sec=3)
    # robot.drive(speed_percent=10, move_angle_deg=270, head_lock=True)
    # robot.timer(duration_sec=3)
    # robot.stop()
    
    
    
    ### successfully catched the ball and shoot
    ball = robot.find_ball(timeout_sec=5.0, min_confidence=0.5)
    if ball is None:
        ball = robot.spin_find_ball(
            step_deg=20.0,
            direction=1,
            max_turn_deg=360.0,
            min_confidence=0.5,
            settle_sec=0.2,
            confirm_settle_sec=0.5,
            confirm_timeout_sec=1.0,
        )
    if ball is not None:
        robot.turn(angle_deg=ball.absolute_angle_deg)
        robot.goto_ball_standoff(ball, stand_off_m=0.30,goal_tolerance_m=0.08)
        
        robot.suck_on(speed_percent=15)
        robot.reset_ball_sucked_detector(required_detected_count=5, sample_interval_sec=0.2)
        while(1):
            ball = robot.find_ball(timeout_sec=0.1)
            if ball is not None:
                robot.drive(speed_percent=10, move_angle_deg=ball.absolute_angle_deg, head_lock=False)
            else:
                print("Ball not found")
            if robot.poll_ball_sucked(stop_on_success=True):
                break
            
            
        print("donedonedonedonedonedonedonedonedonedonedonedonedonedonedonedonedone")
        robot.turn_to_point(x_m=-0.4, y_m=-0.60)
        robot.goto(x_m=-0.4, y_m=-0.6,speed_profile=2,goal_tolerance_m=0.08)
        # robot.turn_to_point(x_m=-0.4, y_m=-0.60)
        robot.turn(angle_deg=0)
        robot.goto(x_m=-0.5, y_m=-0.0,speed_profile=2,goal_tolerance_m=0.08)
        # robot.turn_to_point(x_m=-0.4, y_m=-0.60)
        robot.goto(x_m=-0.4, y_m=0.4,speed_profile=2,goal_tolerance_m=0.08)
        # robot.suck_off()
        robot.turn(angle_deg=-45)
        robot.suck_on(speed_percent=10)
        robot.timer(duration_sec=3.0)
        robot.relay_on()
        robot.timer(duration_sec=3.0)
        robot.relay_off()
        robot.suck_off()
            
    else:
        print("Ball not found")
    
    
    
    
    ### standard spinning and find ball
    # ball = robot.find_ball(timeout_sec=10, min_confidence=0.5)
    # if ball is None:
    #     ball = robot.spin_find_ball(
    #         step_deg=20.0,
    #         direction=1,
    #         max_turn_deg=360.0,
    #         min_confidence=0.5,
    #         settle_sec=0.2,
    #         confirm_settle_sec=0.5,
    #         confirm_timeout_sec=1.0,
    #     )
    # if ball is not None:
    #     robot.turn(angle_deg=ball.absolute_angle_deg)
    #     print(ball.base_x_m, ball.angle_deg, ball.absolute_angle_deg)
    # else:
    #     print("Ball not found")
        
        
        
    ### test move
    # robot.turn(angle_deg=0)
    # robot.move(x_cm=0, y_cm=10, speed_profile=1)
    # robot.turn(angle_deg=90)
    # robot.move(x_cm=10, y_cm=0, speed_profile=1)
    # robot.turn(angle_deg=-90)
    # robot.move(x_cm=10, y_cm=0, speed_profile=1)
    # robot.turn(angle_deg=180)
    # robot.move(x_cm=10, y_cm=0, speed_profile=1)
    
    
    
    ### walk points
    # robot.goto(x_m=-0.40, y_m=-0.60)
    # robot.turn(angle_deg=90)
    # robot.goto(x_m=0.40, y_m=-0.60)
    # robot.turn(angle_deg=0)
    # robot.goto(x_m=-0.40, y_m=0.60)
    # # robot.turn(angle_deg=-90)
    # robot.goto(x_m=0.40, y_m=0.60)
    
    
    
    # robot.goto(x_m=0.0, y_m=-0.315)
    # robot.sleep(duration_sec=2)
    # robot.goto(x_m=-0.30, y_m=0.0)
    # robot.sleep(duration_sec=2)
    # robot.goto(x_m=0.0, y_m=0.315)
    # robot.sleep(duration_sec=2)
    # robot.goto(x_m=0.20, y_m=0.0)
    # robot.sleep(duration_sec=2)
    # robot.goto(x_m=0.0, y_m=-0.315)
    # robot.sleep(duration_sec=2)
    # robot.goto(x_m=-0.40, y_m=-0.60)
    