from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_x", default_value="0.0"),
        DeclareLaunchArgument("map_y", default_value="0.0"),
        DeclareLaunchArgument("map_z", default_value="0.0"),
        Node(
            package="fractal_tactile_exploration",
            executable="omega_explorer_node",
            name="omega_explorer_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("fractal_tactile_exploration"), "config", "omega_explorer.yaml"]),
                {"map_translation_xyz": [LaunchConfiguration("map_x"), LaunchConfiguration("map_y"), LaunchConfiguration("map_z")]},
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="panda_to_tactile_map",
            arguments=[LaunchConfiguration("map_x"), LaunchConfiguration("map_y"), LaunchConfiguration("map_z"), "0", "0", "0", "panda_link0", "tactile_map"],
        ),
    ])