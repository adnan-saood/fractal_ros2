from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    moveit_config = get_package_share_directory('fractal_panda_moveit_config')
    exploration_config = get_package_share_directory('fractal_tactile_exploration')

    explorer = Node(
        package='fractal_tactile_exploration',
        executable='omega_explorer_node',
        name='omega_explorer_node',
        output='screen',
        parameters=[
            f'{exploration_config}/config/omega_explorer.yaml',
            {'motion_test.enabled': True},
        ],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{moveit_config}/launch/mock_planning_rviz.launch.py')),
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
    ])