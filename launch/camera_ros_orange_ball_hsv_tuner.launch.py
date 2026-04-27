from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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
    exposure_time = LaunchConfiguration("exposure_time")
    exposure_time_mode = LaunchConfiguration("exposure_time_mode")
    ae_enable = LaunchConfiguration("ae_enable")
    analogue_gain = LaunchConfiguration("analogue_gain")
    awb_enable = LaunchConfiguration("awb_enable")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("role", default_value="viewfinder"),
            DeclareLaunchArgument("format", default_value="RGB888"),
            DeclareLaunchArgument("width", default_value="800"),
            DeclareLaunchArgument("height", default_value="600"),
            DeclareLaunchArgument("orientation", default_value="0"),
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            DeclareLaunchArgument("use_node_time", default_value="false"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),
            *declare_camera_control_arguments(),
            DeclareLaunchArgument(
                "robot_mask_path",
                default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "mask.png"),
            ),
            DeclareLaunchArgument("orange_h_min", default_value="5"),
            DeclareLaunchArgument("orange_h_max", default_value="30"),
            DeclareLaunchArgument("orange_s_min", default_value="100"),
            DeclareLaunchArgument("orange_v_min", default_value="60"),
            DeclareLaunchArgument("enable_morph_open", default_value="true"),
            DeclareLaunchArgument("morph_kernel_size", default_value="3"),
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
                executable="orange_ball_hsv_tuner_node",
                name="orange_ball_hsv_tuner",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "robot_mask_path": robot_mask_path,
                        "orange_h_min": ParameterValue(
                            LaunchConfiguration("orange_h_min"), value_type=int
                        ),
                        "orange_h_max": ParameterValue(
                            LaunchConfiguration("orange_h_max"), value_type=int
                        ),
                        "orange_s_min": ParameterValue(
                            LaunchConfiguration("orange_s_min"), value_type=int
                        ),
                        "orange_v_min": ParameterValue(
                            LaunchConfiguration("orange_v_min"), value_type=int
                        ),
                        "enable_morph_open": ParameterValue(
                            LaunchConfiguration("enable_morph_open"), value_type=bool
                        ),
                        "morph_kernel_size": ParameterValue(
                            LaunchConfiguration("morph_kernel_size"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
