from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnStateTransition
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    explorer = Node(
        package="fractal_tactile_exploration",
        executable="omega_explorer_node",
        name="omega_explorer_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("fractal_tactile_exploration"), "config", "omega_explorer.yaml"]),
            {"map_translation_xyz": [LaunchConfiguration("map_x"), LaunchConfiguration("map_y"), LaunchConfiguration("map_z")]},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_x", default_value="0.0"),
        DeclareLaunchArgument("map_y", default_value="0.0"),
        DeclareLaunchArgument("map_z", default_value="0.0"),
        DeclareLaunchArgument("map_roll", default_value="0.0"),
        DeclareLaunchArgument("map_pitch", default_value="0.0"),
        DeclareLaunchArgument("map_yaw", default_value="0.0"),
        explorer,
        RegisterEventHandler(
            OnProcessStart(
                target_action=explorer,
                on_start=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(explorer),
                    transition_id=Transition.TRANSITION_CONFIGURE))],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=explorer,
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(explorer),
                    transition_id=Transition.TRANSITION_ACTIVATE))],
            )
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="panda_to_tactile_map",
            arguments=[
                LaunchConfiguration("map_x"), LaunchConfiguration("map_y"),
                LaunchConfiguration("map_z"), LaunchConfiguration("map_roll"),
                LaunchConfiguration("map_pitch"), LaunchConfiguration("map_yaw"),
                "panda_link0", "tactile_map",
            ],
        ),
    ])