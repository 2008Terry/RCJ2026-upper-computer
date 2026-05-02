from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "web_host",
                default_value="0.0.0.0",
                description=(
                    "Host interface for the debug web server. Use 0.0.0.0 for direct "
                    "competition LAN access; 127.0.0.1 also works for SSH-only forwarding."
                ),
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8080",
                description="HTTP port for the debug web server.",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG quality for raw-image fallback streams.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="5.0",
                description="Maximum FPS for raw-image fallback stream encoding.",
            ),
            DeclareLaunchArgument(
                "stream_overrides_json",
                default_value="{}",
                description=(
                    "Optional JSON object keyed by stream id. Each value may override "
                    "compressed_topic, raw_topic, or label."
                ),
            ),
            DeclareLaunchArgument(
                "map_topic",
                default_value="/map",
                description="OccupancyGrid topic for the localization map view.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/amcl_pose",
                description="PoseWithCovarianceStamped topic for robot pose.",
            ),
            DeclareLaunchArgument(
                "particle_topic",
                default_value="/particlecloud",
                description="PoseArray topic for particle cloud visualization.",
            ),
            DeclareLaunchArgument(
                "debug_pointcloud_topic",
                default_value="/field_line_observations_debug",
                description="PointCloud2 topic for localization observation debug status.",
            ),
            DeclareLaunchArgument(
                "processing_time_topic",
                default_value="/amcl_fusion/processing_time_ms",
                description="Float32 topic for localization processing time.",
            ),
            DeclareLaunchArgument(
                "state_file",
                default_value="~/.ros/rcj_localization_web_debug_state.json",
                description="Persistent dashboard UI state file on the robot.",
            ),
            Node(
                package="rcj_localization",
                executable="localization_web_debug_server.py",
                name="localization_web_debug_server",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "web_host": LaunchConfiguration("web_host"),
                        "web_port": ParameterValue(
                            LaunchConfiguration("web_port"),
                            value_type=int,
                        ),
                        "jpeg_quality": ParameterValue(
                            LaunchConfiguration("jpeg_quality"),
                            value_type=int,
                        ),
                        "max_fps": ParameterValue(
                            LaunchConfiguration("max_fps"),
                            value_type=float,
                        ),
                        "stream_overrides_json": ParameterValue(
                            LaunchConfiguration("stream_overrides_json"),
                            value_type=str,
                        ),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "particle_topic": LaunchConfiguration("particle_topic"),
                        "debug_pointcloud_topic": LaunchConfiguration(
                            "debug_pointcloud_topic"
                        ),
                        "processing_time_topic": LaunchConfiguration(
                            "processing_time_topic"
                        ),
                        "state_file": LaunchConfiguration("state_file"),
                    }
                ],
            ),
        ]
    )
