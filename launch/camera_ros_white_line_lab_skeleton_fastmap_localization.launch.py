from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import (
    camera_ros_parameters,
    declare_camera_ros_arguments,
)


def find_latest_fastmap_file():
    config_dir = Path(get_package_share_directory("rcj_localization")) / "config"
    candidates = sorted(config_dir.glob("undistort_map_*_fast.xml"))
    if not candidates:
        candidates = sorted(config_dir.glob("*.xml"))
    if not candidates:
        raise FileNotFoundError(f"No fastmap XML file found in {config_dir}")
    return candidates[-1]


def read_fastmap_source_size(fastmap_file):
    root = ET.parse(fastmap_file).getroot()
    source_width = root.findtext("source_width")
    source_height = root.findtext("source_height")
    if source_width is None or source_height is None:
        raise RuntimeError(
            f"Fastmap XML '{fastmap_file}' is missing source_width/source_height"
        )
    return int(source_width), int(source_height)


def resolve_fastmap_file(context):
    use_latest_fastmap = (
        LaunchConfiguration("use_latest_fastmap").perform(context).strip().lower() == "true"
    )
    fastmap_file_value = LaunchConfiguration("fastmap_file").perform(context).strip()

    if use_latest_fastmap or not fastmap_file_value:
        return find_latest_fastmap_file()

    fastmap_path = Path(fastmap_file_value).expanduser()
    if not fastmap_path.is_absolute():
        fastmap_path = fastmap_path.resolve()
    if not fastmap_path.exists():
        raise FileNotFoundError(f"Fastmap XML file does not exist: {fastmap_path}")
    return fastmap_path


def build_nodes(context):
    package_share = Path(get_package_share_directory("rcj_localization"))
    pipeline_launch = (
        package_share / "launch" / "white_line_lab_skeleton_fastmap_localization.launch.py"
    )
    selected_fastmap_file = resolve_fastmap_file(context)
    default_width, default_height = read_fastmap_source_size(selected_fastmap_file)
    width_value = int(
        LaunchConfiguration("width").perform(context).strip() or str(default_width)
    )
    height_value = int(
        LaunchConfiguration("height").perform(context).strip() or str(default_height)
    )

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    output_topic = LaunchConfiguration("output_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")

    return [
        Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                remappings=[
                    ("~/image_raw", input_topic),
                    ("~/camera_info", camera_info_topic),
                ],
                parameters=[
                    {
                        **camera_ros_parameters(width_value, height_value),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pipeline_launch),
                launch_arguments={
                    "enable_openmv_camera": "false",
                    "camera_info_topic": camera_info_topic,
                    "input_topic": input_topic,
                    "output_topic": output_topic,
                    "robot_mask_path": robot_mask_path,
                    "use_latest_fastmap": LaunchConfiguration("use_latest_fastmap"),
                    "fastmap_file": str(selected_fastmap_file),
                }.items(),
            ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *declare_camera_ros_arguments(),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            DeclareLaunchArgument("output_topic", default_value="/camera/image_remapped"),  # Remapped image topic
            DeclareLaunchArgument("robot_mask_path", default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "remapped_mask.png")),  # Optional remapped-space robot mask image path
            DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Auto-select the latest fastmap XML
            DeclareLaunchArgument("fastmap_file", default_value=""),  # Specific fastmap XML path
            OpaqueFunction(function=build_nodes),
        ]
    )
