import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    gui_default = (
        "true"
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        else "false"
    )

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

    orange_h_min = LaunchConfiguration("orange_h_min")
    orange_h_max = LaunchConfiguration("orange_h_max")
    orange_s_min = LaunchConfiguration("orange_s_min")
    orange_v_min = LaunchConfiguration("orange_v_min")
    enable_morph_open = LaunchConfiguration("enable_morph_open")
    morph_kernel_size = LaunchConfiguration("morph_kernel_size")

    exposure_time = LaunchConfiguration("exposure_time")
    exposure_time_mode = LaunchConfiguration("exposure_time_mode")
    ae_enable = LaunchConfiguration("ae_enable")
    analogue_gain_mode = LaunchConfiguration("analogue_gain_mode")
    analogue_gain = LaunchConfiguration("analogue_gain")
    digital_gain = LaunchConfiguration("digital_gain")
    awb_enable = LaunchConfiguration("awb_enable")
    enable_hsv_tuner = LaunchConfiguration("enable_hsv_tuner")
    enable_exposure_slider = LaunchConfiguration("enable_exposure_slider")
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
            DeclareLaunchArgument("orange_h_min", default_value="5"),  # HSV lower hue threshold for orange.
            DeclareLaunchArgument("orange_h_max", default_value="30"),  # HSV upper hue threshold for orange.
            DeclareLaunchArgument("orange_s_min", default_value="100"),  # HSV lower saturation threshold for orange.
            DeclareLaunchArgument("orange_v_min", default_value="60"),  # HSV lower value threshold for orange.
            DeclareLaunchArgument("enable_morph_open", default_value="true"),  # Enable morphological open in the tuner.
            DeclareLaunchArgument("morph_kernel_size", default_value="3"),  # Morphological kernel size for the tuner.
            DeclareLaunchArgument("exposure_time", default_value="10000"),  # Manual exposure time in microseconds.
            DeclareLaunchArgument("exposure_time_mode", default_value="1"),  # Exposure control mode.
            DeclareLaunchArgument("ae_enable", default_value="false"),  # Disable auto exposure by default.
            DeclareLaunchArgument("analogue_gain_mode", default_value="1"),  # Analogue gain control mode.
            DeclareLaunchArgument("analogue_gain", default_value="1.0"),  # Fixed analogue gain multiplier.
            DeclareLaunchArgument("digital_gain", default_value="1.0"),  # Fixed digital gain multiplier.
            DeclareLaunchArgument("awb_enable", default_value="false"),  # Disable auto white balance by default.
            DeclareLaunchArgument("enable_hsv_tuner", default_value=gui_default),  # Start the HSV tuner node when a GUI is available.
            DeclareLaunchArgument("enable_exposure_slider", default_value=gui_default),  # Start the exposure slider debug node when a GUI is available.
            DeclareLaunchArgument("camera_node_name", default_value="/camera"),  # Camera node name for parameter updates.
            DeclareLaunchArgument(
                "debug_window_name", default_value="Orange Ball Fixed Exposure Debug"
            ),  # Exposure slider window title.
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
                        "ExposureTime": ParameterValue(exposure_time, value_type=int),
                        "ExposureTimeMode": ParameterValue(
                            exposure_time_mode, value_type=int
                        ),
                        "AeEnable": ParameterValue(ae_enable, value_type=bool),
                        "AnalogueGainMode": ParameterValue(
                            analogue_gain_mode, value_type=int
                        ),
                        "AnalogueGain": ParameterValue(
                            analogue_gain, value_type=float
                        ),
                        "DigitalGain": ParameterValue(digital_gain, value_type=float),
                        "AwbEnable": ParameterValue(awb_enable, value_type=bool),
                        "ColourGains": [1.0, 1.0],
                        "FrameDurationLimits": [33333, 33333],
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="orange_ball_hsv_tuner_node",
                name="orange_ball_hsv_tuner",
                output="screen",
                condition=IfCondition(enable_hsv_tuner),
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "orange_h_min": ParameterValue(orange_h_min, value_type=int),
                        "orange_h_max": ParameterValue(orange_h_max, value_type=int),
                        "orange_s_min": ParameterValue(orange_s_min, value_type=int),
                        "orange_v_min": ParameterValue(orange_v_min, value_type=int),
                        "enable_morph_open": ParameterValue(
                            enable_morph_open, value_type=bool
                        ),
                        "morph_kernel_size": ParameterValue(
                            morph_kernel_size, value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="camera_fixed_exposure_debug.py",
                name="camera_fixed_exposure_debug",
                output="screen",
                condition=IfCondition(enable_exposure_slider),
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
