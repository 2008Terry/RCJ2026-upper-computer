from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import (
    camera_control_launch_arguments,
    camera_control_parameters,
    declare_camera_control_arguments,
    declare_hsv_green_white_black_arguments,
    hsv_green_white_black_launch_arguments,
    hsv_green_white_black_parameters,
)


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration("camera_index")
    role = LaunchConfiguration("role")
    image_format = LaunchConfiguration("format")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    orientation = LaunchConfiguration("orientation")
    sensor_mode = LaunchConfiguration("sensor_mode")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_node_time = LaunchConfiguration("use_node_time")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    white_mask_topic = LaunchConfiguration("white_mask_topic")
    robot_mask_topic = LaunchConfiguration("robot_mask_topic")

    exposure_time = LaunchConfiguration("exposure_time")
    exposure_time_mode = LaunchConfiguration("exposure_time_mode")
    ae_enable = LaunchConfiguration("ae_enable")
    analogue_gain = LaunchConfiguration("analogue_gain")
    awb_enable = LaunchConfiguration("awb_enable")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera index
            DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros role
            DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
            DeclareLaunchArgument("width", default_value="800"),  # Capture width
            DeclareLaunchArgument("height", default_value="600"),  # Capture height
            DeclareLaunchArgument("orientation", default_value="0"),  # Camera rotation angle
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Camera sensor mode
            DeclareLaunchArgument("frame_id", default_value="camera"),  # Image frame id
            DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration URL
            DeclareLaunchArgument("use_node_time", default_value="false"),  # Whether to use node time
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            *declare_camera_control_arguments(),
            DeclareLaunchArgument(
                "white_mask_topic",
                default_value="/white_line_hsv_white_node/white_mask",
            ),  # White mask topic
            DeclareLaunchArgument(
                "robot_mask_topic", default_value=""
            ),  # Optional robot mask topic
            *declare_hsv_green_white_black_arguments(),
            DeclareLaunchArgument(
                "hsv_enable_timing_log", default_value="true"
            ),  # Whether to log HSV timing
            DeclareLaunchArgument(
                "hsv_timing_log_interval", default_value="15"
            ),  # HSV timing log frame interval
            DeclareLaunchArgument(
                "hsv_enable_image_view", default_value="true"
            ),  # Master switch for HSV debug windows
            DeclareLaunchArgument(
                "hsv_enable_controls_window", default_value="true"
            ),  # Whether to show HSV slider controls window
            DeclareLaunchArgument(
                "hsv_show_input_image", default_value="true"
            ),  # Show the HSV input image window
            DeclareLaunchArgument(
                "hsv_show_white_mask", default_value="true"
            ),  # Show the white-priority mask window
            DeclareLaunchArgument(
                "hsv_show_green_mask", default_value="true"
            ),  # Show the green-priority mask window
            DeclareLaunchArgument(
                "hsv_show_black_mask", default_value="true"
            ),  # Show the black-priority mask window
            DeclareLaunchArgument(
                "hsv_show_noise_mask", default_value="true"
            ),  # Show the remaining noise mask window
            DeclareLaunchArgument(
                "hsv_show_overlay_image", default_value="true"
            ),  # Show the HSV overlay window
            DeclareLaunchArgument(
                "hsv_publish_debug_images", default_value="false"
            ),  # Master switch for HSV debug image topics
            DeclareLaunchArgument(
                "hsv_publish_input_image", default_value="true"
            ),  # Publish HSV input debug topic when subscribed
            DeclareLaunchArgument(
                "hsv_publish_white_mask", default_value="true"
            ),  # Publish HSV white debug mask when subscribed
            DeclareLaunchArgument(
                "hsv_publish_green_mask", default_value="true"
            ),  # Publish HSV green debug mask when subscribed
            DeclareLaunchArgument(
                "hsv_publish_black_mask", default_value="true"
            ),  # Publish HSV black debug mask when subscribed
            DeclareLaunchArgument(
                "hsv_publish_noise_mask", default_value="true"
            ),  # Publish HSV noise debug mask when subscribed
            DeclareLaunchArgument(
                "hsv_publish_overlay_image", default_value="true"
            ),  # Publish HSV overlay debug topic when subscribed
            DeclareLaunchArgument(
                "hsv_display_max_width", default_value="960"
            ),  # HSV window max width
            DeclareLaunchArgument(
                "hsv_display_max_height", default_value="720"
            ),  # HSV window max height
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    ("~/image_raw", input_topic),
                    ("~/camera_info", camera_info_topic),
                ],
                parameters=[
                    {
                        "camera": ParameterValue(camera_index, value_type=int),
                        "role": role,
                        "format": image_format,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "orientation": ParameterValue(orientation, value_type=int),
                        "sensor_mode": sensor_mode,
                        "frame_id": frame_id,
                        "camera_info_url": camera_info_url,
                        "use_node_time": ParameterValue(use_node_time, value_type=bool),
                        **camera_control_parameters(),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="white_line_hsv_white_node",
                name="white_line_hsv_white_node",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[("~/white_mask", white_mask_topic)],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "robot_mask_topic": robot_mask_topic,
                        **hsv_green_white_black_parameters(),
                        "enable_timing_log": ParameterValue(
                            LaunchConfiguration("hsv_enable_timing_log"),
                            value_type=bool,
                        ),
                        "timing_log_interval": ParameterValue(
                            LaunchConfiguration("hsv_timing_log_interval"),
                            value_type=int,
                        ),
                        "enable_image_view": ParameterValue(
                            LaunchConfiguration("hsv_enable_image_view"),
                            value_type=bool,
                        ),
                        "enable_controls_window": ParameterValue(
                            LaunchConfiguration("hsv_enable_controls_window"),
                            value_type=bool,
                        ),
                        "show_input_image": ParameterValue(
                            LaunchConfiguration("hsv_show_input_image"),
                            value_type=bool,
                        ),
                        "show_white_mask": ParameterValue(
                            LaunchConfiguration("hsv_show_white_mask"),
                            value_type=bool,
                        ),
                        "show_green_mask": ParameterValue(
                            LaunchConfiguration("hsv_show_green_mask"),
                            value_type=bool,
                        ),
                        "show_black_mask": ParameterValue(
                            LaunchConfiguration("hsv_show_black_mask"),
                            value_type=bool,
                        ),
                        "show_noise_mask": ParameterValue(
                            LaunchConfiguration("hsv_show_noise_mask"),
                            value_type=bool,
                        ),
                        "show_overlay_image": ParameterValue(
                            LaunchConfiguration("hsv_show_overlay_image"),
                            value_type=bool,
                        ),
                        "publish_debug_images": ParameterValue(
                            LaunchConfiguration("hsv_publish_debug_images"),
                            value_type=bool,
                        ),
                        "publish_input_image": ParameterValue(
                            LaunchConfiguration("hsv_publish_input_image"),
                            value_type=bool,
                        ),
                        "publish_white_mask": ParameterValue(
                            LaunchConfiguration("hsv_publish_white_mask"),
                            value_type=bool,
                        ),
                        "publish_green_mask": ParameterValue(
                            LaunchConfiguration("hsv_publish_green_mask"),
                            value_type=bool,
                        ),
                        "publish_black_mask": ParameterValue(
                            LaunchConfiguration("hsv_publish_black_mask"),
                            value_type=bool,
                        ),
                        "publish_noise_mask": ParameterValue(
                            LaunchConfiguration("hsv_publish_noise_mask"),
                            value_type=bool,
                        ),
                        "publish_overlay_image": ParameterValue(
                            LaunchConfiguration("hsv_publish_overlay_image"),
                            value_type=bool,
                        ),
                        "display_max_width": ParameterValue(
                            LaunchConfiguration("hsv_display_max_width"),
                            value_type=int,
                        ),
                        "display_max_height": ParameterValue(
                            LaunchConfiguration("hsv_display_max_height"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
