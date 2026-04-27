from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    # Fixed-exposure controls for camera_ros/libcamera.
    exposure_time = LaunchConfiguration("exposure_time")
    exposure_time_mode = LaunchConfiguration("exposure_time_mode")
    ae_enable = LaunchConfiguration("ae_enable")
    analogue_gain = LaunchConfiguration("analogue_gain")
    awb_enable = LaunchConfiguration("awb_enable")
    enable_camera_debug_view = LaunchConfiguration("enable_camera_debug_view")
    camera_node_name = LaunchConfiguration("camera_node_name")
    debug_window_name = LaunchConfiguration("debug_window_name")
    exposure_time_min = LaunchConfiguration("exposure_time_min")
    exposure_time_max = LaunchConfiguration("exposure_time_max")
    exposure_time_step = LaunchConfiguration("exposure_time_step")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera device index.
            DeclareLaunchArgument("role", default_value="viewfinder"),  # Libcamera stream role.
            DeclareLaunchArgument("format", default_value="RGB888"),  # Pixel format for published frames.
            DeclareLaunchArgument("width", default_value="800"),  # Output image width in pixels.
            DeclareLaunchArgument("height", default_value="600"),  # Output image height in pixels.
            DeclareLaunchArgument("orientation", default_value="0"),  # Image rotation/orientation setting.
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Sensor capture mode/resolution preset.
            DeclareLaunchArgument("frame_id", default_value="camera"),  # TF frame attached to camera messages.
            DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration file URL.
            DeclareLaunchArgument("use_node_time", default_value="false"),  # Use node clock for timestamps.
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Remapped camera info topic.
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Remapped image output topic.
            *declare_camera_control_arguments(),
            DeclareLaunchArgument("enable_camera_debug_view", default_value="true"),  # Start the debug viewer node.
            DeclareLaunchArgument("camera_node_name", default_value="/camera"),  # Camera node name for parameter updates.
            DeclareLaunchArgument(
                "debug_window_name", default_value="Camera Fixed Exposure Debug"
            ),  # Debug viewer window title.
            DeclareLaunchArgument("exposure_time_min", default_value="100"),  # Minimum selectable exposure time.
            DeclareLaunchArgument("exposure_time_max", default_value="40000"),  # Maximum selectable exposure time.
            DeclareLaunchArgument("exposure_time_step", default_value="100"),  # Exposure adjustment step size.
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
                        # On newer libcamera builds, manual exposure is typically selected
                        # via ExposureTimeMode=1. Keeping AeEnable=false also helps when
                        # testing on setups that still expose the older auto-exposure flag.
                        **camera_control_parameters(),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="camera_fixed_exposure_debug.py",
                name="camera_fixed_exposure_debug",
                output="screen",
                condition=IfCondition(enable_camera_debug_view),
                parameters=[
                    {
                        "input_topic": input_topic,
                        "camera_node_name": camera_node_name,
                        "window_name": debug_window_name,
                        "exposure_time": ParameterValue(exposure_time, value_type=int),
                        "exposure_time_min": ParameterValue(
                            exposure_time_min, value_type=int
                        ),
                        "exposure_time_max": ParameterValue(
                            exposure_time_max, value_type=int
                        ),
                        "exposure_time_step": ParameterValue(
                            exposure_time_step, value_type=int
                        ),
                        "exposure_time_mode": ParameterValue(
                            exposure_time_mode, value_type=int
                        ),
                        "ae_enable": ParameterValue(ae_enable, value_type=bool),
                    }
                ],
            ),
        ]
    )
