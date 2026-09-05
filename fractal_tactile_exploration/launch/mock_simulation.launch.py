from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    moveit_share = get_package_share_directory('fractal_panda_moveit_config')
    exploration_share = get_package_share_directory('fractal_tactile_exploration')
    explorer = LifecycleNode(
        package='fractal_tactile_exploration', executable='omega_explorer_node',
        namespace='', name='omega_explorer_node', output='screen', parameters=[
            f'{exploration_share}/config/omega_explorer.yaml',
            {'motion_test.frame_id': 'world',
             'exploration.max_probes': 3},
        ])
    sensor = Node(
        package='fractal_tactile_exploration', executable='sim_sensor_node', output='screen',
        parameters=[
            {'surface.mode': 'bump', 'surface.origin_x': -0.20,
             'surface.origin_y': -0.20, 'surface.size_x': 0.40,
             'surface.size_y': 0.40, 'surface.base_z': 0.0},
        ])
    configure = RegisterEventHandler(OnProcessStart(
        target_action=explorer,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(explorer),
            transition_id=Transition.TRANSITION_CONFIGURE))]))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=explorer, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(explorer),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{moveit_share}/launch/mock_planning_rviz.launch.py'),
            launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                       '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'tactile_map']),
        sensor, explorer, configure, activate,
    ])
