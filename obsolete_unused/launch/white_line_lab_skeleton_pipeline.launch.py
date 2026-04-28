from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")

    morph_node_name = "white_line_lab_morph_node"

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/image_raw",
            description="Input image topic for the LAB morph stage.",
        ),
        DeclareLaunchArgument("morph_enable_timing_debug", default_value="false"),  # Enable morph timing logs
        DeclareLaunchArgument("morph_timing_summary_interval", default_value="10"),  # Morph timing summary frame interval
        DeclareLaunchArgument("skeleton_enable_timing_debug", default_value="false"),  # Enable skeleton timing logs
        DeclareLaunchArgument("skeleton_timing_summary_interval", default_value="10"),  # Skeleton timing summary frame interval
        Node(
            package="rcj_localization",
            executable="white_line_lab_morph_node",
            name=morph_node_name,
            output="screen",
            parameters=[
                {
                    "input_topic": input_topic,
                    "green_a_max": 117,
                    "enable_timing_debug": LaunchConfiguration("morph_enable_timing_debug"),
                    "timing_summary_interval": LaunchConfiguration("morph_timing_summary_interval"),
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="white_line_skeleton_filter_node",
            name="white_line_skeleton_filter_node",
            output="screen",
            parameters=[
                {
                    "morph_mask_topic": f"/{morph_node_name}/white_morph_mask",
                    "green_mask_topic": f"/{morph_node_name}/green_mask",
                    "black_mask_topic": f"/{morph_node_name}/black_mask",
                    "noise_mask_topic": f"/{morph_node_name}/noise_mask",
                    "orientation_window_radius_px": 5,
                    "min_orientation_neighbors": 6,
                    "enable_image_view": True,
                    "enable_timing_debug": LaunchConfiguration("skeleton_enable_timing_debug"),
                    "timing_summary_interval": LaunchConfiguration(
                        "skeleton_timing_summary_interval"
                    ),
                }
            ],
        ),
    ])
