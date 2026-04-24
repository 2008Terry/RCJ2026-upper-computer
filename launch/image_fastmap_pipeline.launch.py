from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def find_latest_fastmap_file():
    config_dir = Path(get_package_share_directory("rcj_localization")) / "config"
    candidates = sorted(config_dir.glob("undistort_map_*_fast.xml"))
    if not candidates:
        candidates = sorted(config_dir.glob("*.xml"))
    if not candidates:
        raise FileNotFoundError(f"No fastmap XML file found in {config_dir}")
    return candidates[-1]


def generate_launch_description():
    fastmap_default = str(find_latest_fastmap_file())
    selected_fastmap_file = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_latest_fastmap"),
            "' == 'true' and '",
            fastmap_default,
            "' or '",
            LaunchConfiguration("fastmap_file"),
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),  # Serial port for the camera publisher
            DeclareLaunchArgument("baudrate", default_value="115200"),  # Serial baudrate
            DeclareLaunchArgument("timeout_sec", default_value="1.0"),  # Serial read timeout in seconds
            DeclareLaunchArgument("frame_id", default_value="camera"),  # Frame id for published images
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw input image topic
            DeclareLaunchArgument("output_topic", default_value="/camera/image_remapped"),  # Remapped output image topic
            DeclareLaunchArgument("use_latest_fastmap", default_value="true"),  # Auto-select the newest fastmap file
            DeclareLaunchArgument("fastmap_file", default_value=""),  # Fastmap XML path when auto-select is off
            DeclareLaunchArgument("robot_mask_path", default_value="/home/rcj/Documents/calibration_images/remapped_mask.png"),  # Optional remapped-space robot mask image path
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Input transport for the remap node
            DeclareLaunchArgument("interpolation", default_value="linear"),  # Remap interpolation mode
            DeclareLaunchArgument("remap_enable_image_view", default_value="true"),  # Show remap debug windows
            DeclareLaunchArgument("remap_show_input_image", default_value="true"),  # Show the remap input image
            DeclareLaunchArgument("remap_show_output_image", default_value="true"),  # Show the remap output image
            Node(
                package="rcj_localization",
                executable="image_publisher.py",
                name="openmv_camera",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "timeout_sec": LaunchConfiguration("timeout_sec"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "image_topic": LaunchConfiguration("input_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="fastmap_remap_node",
                name="fastmap_remap_node",
                output="screen",
                parameters=[
                    {
                        "fastmap_file": selected_fastmap_file,
                        "robot_mask_path": LaunchConfiguration("robot_mask_path"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "input_transport": LaunchConfiguration("input_transport"),
                        "interpolation": LaunchConfiguration("interpolation"),
                        "enable_image_view": LaunchConfiguration("remap_enable_image_view"),
                        "show_input_image": LaunchConfiguration("remap_show_input_image"),
                        "show_output_image": LaunchConfiguration("remap_show_output_image"),
                    }
                ],
            ),
        ]
    )
