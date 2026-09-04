import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    description_package = get_package_share_directory('fractal_panda_description')
    config_package = get_package_share_directory('fractal_panda_moveit_config')

    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description_package, 'urdf', 'fractal_panda.urdf.xacro'),
        ' robot_ip:=mock',
        ' use_fake_hardware:=true',
        ' fake_sensor_commands:=true',
        ' planning_mode:=true',
    ])
    robot_description = {'robot_description': robot_description_config}

    semantic_description_config = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(config_package, 'srdf', 'panda_arm.srdf.xacro'),
        ' hand:=false',
    ])
    robot_description_semantic = {
        'robot_description_semantic': semantic_description_config,
    }

    kinematics_yaml = load_yaml(
        'fractal_panda_moveit_config', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml(
        'fractal_panda_moveit_config', 'config/ompl_planning.yaml')
    controller_yaml = load_yaml(
        'fractal_panda_moveit_config', 'config/panda_controllers.yaml')

    planning_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'default_planner_config': 'RRTConnectkConfigDefault',
            'start_state_max_bounds_error': 0.1,
        }
    }
    planning_pipeline['move_group'].update(ompl_planning_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': controller_yaml,
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        ),
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    ros2_controllers = os.path.join(
        config_package, 'config', 'panda_mock_ros_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ros2_controllers],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'joint_state_broadcaster'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'panda_arm_controller'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                planning_pipeline,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor,
                {'publish_robot_description_semantic': True},
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(config_package, 'rviz', 'moveit.rviz')],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                planning_pipeline,
            ],
            output='log',
            condition=IfCondition(use_rviz),
        ),
    ])