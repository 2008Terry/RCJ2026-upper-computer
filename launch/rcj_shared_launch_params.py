from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


CAMERA_NODE_DEFAULTS = {
    "camera_index": "0",
    "role": "viewfinder",
    "format": "RGB888",
    "width": "800",
    "height": "600",
    "orientation": "0",
    "sensor_mode": "1332:990",
    "frame_id": "camera",
    "camera_info_url": "",
    "use_node_time": "false",
    "exposure_time": "10000",
    "exposure_time_mode": "1",
    "ae_enable": "false",
}


HSV_GREEN_WHITE_BLACK_DEFAULTS = {
    "white_h_min": "0",
    "white_h_max": "179",
    "white_s_max": "196",
    "white_v_min": "158",
    "black_v_max": "45",
    "green_h_min": "35",
    "green_h_max": "100",
    "green_s_min": "210",
    "green_v_min": "80",
}


REMAP_DEFAULTS = {
    "interpolation": "linear",
}


def declare_camera_ros_arguments():
    return [
        DeclareLaunchArgument(
            "camera_index",
            default_value=CAMERA_NODE_DEFAULTS["camera_index"],
        ),  # Camera index
        DeclareLaunchArgument(
            "role",
            default_value=CAMERA_NODE_DEFAULTS["role"],
        ),  # camera_ros role
        DeclareLaunchArgument(
            "format",
            default_value=CAMERA_NODE_DEFAULTS["format"],
        ),  # Camera pixel format
        DeclareLaunchArgument(
            "width",
            default_value=CAMERA_NODE_DEFAULTS["width"],
        ),  # Capture width
        DeclareLaunchArgument(
            "height",
            default_value=CAMERA_NODE_DEFAULTS["height"],
        ),  # Capture height
        DeclareLaunchArgument(
            "orientation",
            default_value=CAMERA_NODE_DEFAULTS["orientation"],
        ),  # Camera rotation angle
        DeclareLaunchArgument(
            "sensor_mode",
            default_value=CAMERA_NODE_DEFAULTS["sensor_mode"],
        ),  # Camera sensor mode
        DeclareLaunchArgument(
            "frame_id",
            default_value=CAMERA_NODE_DEFAULTS["frame_id"],
        ),  # Image frame id
        DeclareLaunchArgument(
            "camera_info_url",
            default_value=CAMERA_NODE_DEFAULTS["camera_info_url"],
        ),  # Camera calibration URL
        DeclareLaunchArgument(
            "use_node_time",
            default_value=CAMERA_NODE_DEFAULTS["use_node_time"],
        ),  # Whether to use node time
        *declare_camera_control_arguments(),
    ]


def declare_camera_control_arguments():
    return [
        DeclareLaunchArgument(
            "exposure_time",
            default_value=CAMERA_NODE_DEFAULTS["exposure_time"],
        ),  # Manual exposure time in microseconds
        DeclareLaunchArgument(
            "exposure_time_mode",
            default_value=CAMERA_NODE_DEFAULTS["exposure_time_mode"],
        ),  # Exposure control mode
        DeclareLaunchArgument(
            "ae_enable",
            default_value=CAMERA_NODE_DEFAULTS["ae_enable"],
        ),  # Enable auto exposure
    ]


def camera_ros_parameters(width_value=None, height_value=None):
    if width_value is None:
        width_value = LaunchConfiguration("width")
    if height_value is None:
        height_value = LaunchConfiguration("height")

    return {
        "camera": ParameterValue(
            LaunchConfiguration("camera_index"), value_type=int
        ),
        "role": LaunchConfiguration("role"),
        "format": LaunchConfiguration("format"),
        "width": ParameterValue(width_value, value_type=int),
        "height": ParameterValue(height_value, value_type=int),
        "orientation": ParameterValue(
            LaunchConfiguration("orientation"), value_type=int
        ),
        "sensor_mode": LaunchConfiguration("sensor_mode"),
        "frame_id": LaunchConfiguration("frame_id"),
        "camera_info_url": LaunchConfiguration("camera_info_url"),
        "use_node_time": ParameterValue(
            LaunchConfiguration("use_node_time"), value_type=bool
        ),
        **camera_control_parameters(),
    }


def camera_control_parameters():
    return {
        "ExposureTime": ParameterValue(
            LaunchConfiguration("exposure_time"), value_type=int
        ),
        "ExposureTimeMode": ParameterValue(
            LaunchConfiguration("exposure_time_mode"), value_type=int
        ),
        "AeEnable": ParameterValue(LaunchConfiguration("ae_enable"), value_type=bool),
    }


def camera_control_launch_arguments():
    return {
        "exposure_time": LaunchConfiguration("exposure_time"),
        "exposure_time_mode": LaunchConfiguration("exposure_time_mode"),
        "ae_enable": LaunchConfiguration("ae_enable"),
    }


def declare_remap_interpolation_argument():
    return DeclareLaunchArgument(
        "interpolation",
        default_value=REMAP_DEFAULTS["interpolation"],
    )  # Remap interpolation mode


def declare_hsv_green_white_black_arguments():
    return [
        DeclareLaunchArgument(
            "white_h_min",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["white_h_min"],
        ),  # White HSV minimum H
        DeclareLaunchArgument(
            "white_h_max",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["white_h_max"],
        ),  # White HSV maximum H
        DeclareLaunchArgument(
            "white_s_max",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["white_s_max"],
        ),  # White HSV maximum S
        DeclareLaunchArgument(
            "white_v_min",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["white_v_min"],
        ),  # White HSV minimum V
        DeclareLaunchArgument(
            "black_v_max",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["black_v_max"],
        ),  # Black HSV maximum V
        DeclareLaunchArgument(
            "green_h_min",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["green_h_min"],
        ),  # Green HSV minimum H
        DeclareLaunchArgument(
            "green_h_max",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["green_h_max"],
        ),  # Green HSV maximum H
        DeclareLaunchArgument(
            "green_s_min",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["green_s_min"],
        ),  # Green HSV minimum S
        DeclareLaunchArgument(
            "green_v_min",
            default_value=HSV_GREEN_WHITE_BLACK_DEFAULTS["green_v_min"],
        ),  # Green HSV minimum V
    ]


def hsv_green_white_black_parameters():
    return {
        "white_h_min": ParameterValue(
            LaunchConfiguration("white_h_min"), value_type=int
        ),
        "white_h_max": ParameterValue(
            LaunchConfiguration("white_h_max"), value_type=int
        ),
        "white_s_max": ParameterValue(
            LaunchConfiguration("white_s_max"), value_type=int
        ),
        "white_v_min": ParameterValue(
            LaunchConfiguration("white_v_min"), value_type=int
        ),
        "black_v_max": ParameterValue(
            LaunchConfiguration("black_v_max"), value_type=int
        ),
        "green_h_min": ParameterValue(
            LaunchConfiguration("green_h_min"), value_type=int
        ),
        "green_h_max": ParameterValue(
            LaunchConfiguration("green_h_max"), value_type=int
        ),
        "green_s_min": ParameterValue(
            LaunchConfiguration("green_s_min"), value_type=int
        ),
        "green_v_min": ParameterValue(
            LaunchConfiguration("green_v_min"), value_type=int
        ),
    }


def hsv_green_white_black_launch_arguments():
    return {
        "white_h_min": LaunchConfiguration("white_h_min"),
        "white_h_max": LaunchConfiguration("white_h_max"),
        "white_s_max": LaunchConfiguration("white_s_max"),
        "white_v_min": LaunchConfiguration("white_v_min"),
        "black_v_max": LaunchConfiguration("black_v_max"),
        "green_h_min": LaunchConfiguration("green_h_min"),
        "green_h_max": LaunchConfiguration("green_h_max"),
        "green_s_min": LaunchConfiguration("green_s_min"),
        "green_v_min": LaunchConfiguration("green_v_min"),
    }
