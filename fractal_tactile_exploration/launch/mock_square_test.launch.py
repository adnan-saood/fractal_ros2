from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = get_package_share_directory('fractal_panda_moveit_config')
    exploration_config = get_package_share_directory('fractal_tactile_exploration')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{moveit_config}/launch/mock_planning_rviz.launch.py')),
        Node(
            package='fractal_tactile_exploration',
            executable='omega_explorer_node',
            name='omega_explorer_node',
            output='screen',
            parameters=[
                f'{exploration_config}/config/omega_explorer.yaml',
                {'motion_test.enabled': True},
            ],
        ),
    ])