import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                             IncludeLaunchDescription, RegisterEventHandler,
                             SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
import yaml


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    exploration_share = get_package_share_directory('fractal_tactile_exploration')
    moveit_share = get_package_share_directory('fractal_panda_moveit_config')
    description_share = get_package_share_directory('fractal_panda_description')
    franka_share = get_package_share_directory('franka_description')
    controller_file = os.path.join(exploration_share, 'config', 'simulation_controllers.yaml')
    xacro_file = os.path.join(description_share, 'urdf', 'fractal_panda.urdf.xacro')
    srdf_file = os.path.join(moveit_share, 'srdf', 'panda_arm.srdf.xacro')
    rviz_file = os.path.join(moveit_share, 'rviz', 'moveit.rviz')

    surface_arguments = [
        DeclareLaunchArgument('surface_mode', default_value='bump'),
        DeclareLaunchArgument('surface_origin_x', default_value='-0.20'),
        DeclareLaunchArgument('surface_origin_y', default_value='-0.20'),
        DeclareLaunchArgument('surface_size_x', default_value='0.40'),
        DeclareLaunchArgument('surface_size_y', default_value='0.40'),
        DeclareLaunchArgument('surface_base_z', default_value='0.0'),
        DeclareLaunchArgument('bump_x', default_value='0.05'),
        DeclareLaunchArgument('bump_y', default_value='0.05'),
        DeclareLaunchArgument('bump_height', default_value='0.008'),
        DeclareLaunchArgument('bump_sigma', default_value='0.04'),
        DeclareLaunchArgument('max_probes', default_value='3'),
        DeclareLaunchArgument('run_smoke_test', default_value='false'),
        DeclareLaunchArgument('use_fractal_pad', default_value='true'),
    ]

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' hand:=false robot_ip:=0.0.0.0 use_fake_hardware:=false',
            ' fake_sensor_commands:=false sim_gazebo:=true',
            ' simulation_controllers:=', controller_file,
            ' planning_mode:=true',
            ' use_fractal_pad:=', LaunchConfiguration('use_fractal_pad'),
        ])
    }
    simulation_time = {'use_sim_time': True}
    robot_description_semantic = {
        'robot_description_semantic': Command([
            FindExecutable(name='xacro'), ' ', srdf_file, ' arm_id:=panda',
        ])
    }
    kinematics = load_yaml('fractal_panda_moveit_config', 'config/kinematics.yaml')
    ompl = load_yaml('fractal_panda_moveit_config', 'config/ompl_planning.yaml')
    controllers = load_yaml('fractal_panda_moveit_config', 'config/panda_controllers.yaml')
    move_group_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision',
            'start_state_max_bounds_error': 0.1,
        }
    }
    move_group_pipeline['move_group'].update(ompl)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_version': '6',
            'gz_args': [PathJoinSubstitution([
                FindPackageShare('fractal_tactile_exploration'),
                'worlds', 'tactile_probe.sdf'])],
        }.items())
    gazebo_resource_paths = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(franka_share),
            os.path.dirname(description_share),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]))
    ignition_resource_paths = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(franka_share),
            os.path.dirname(description_share),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]))
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='both', parameters=[robot_description, simulation_time])
    spawn_robot = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'fractal_panda'])])
    spawn_joint_state_broadcaster = TimerAction(period=6.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster',
             '--controller-manager', '/controller_manager'], output='screen')])
    spawn_arm_controller = TimerAction(period=7.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'panda_arm_controller',
             '--controller-manager', '/controller_manager'], output='screen')])

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics,
                    move_group_pipeline,
                    {'moveit_simple_controller_manager': controllers,
                     'moveit_controller_manager':
                         'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                     'use_sim_time': True,
                     'moveit_manage_controllers': False,
                     'trajectory_execution.allowed_execution_duration_scaling': 1.5,
                     'trajectory_execution.allowed_goal_duration_margin': 1.0,
                     'trajectory_execution.allowed_start_tolerance': 0.05,
                     'publish_planning_scene': True,
                     'publish_geometry_updates': True,
                     'publish_state_updates': True,
                     'publish_transforms_updates': True}])
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_file],
        parameters=[robot_description, robot_description_semantic, kinematics,
                    move_group_pipeline, simulation_time])

    map_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                   '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'tactile_map'])
    sensors = Node(
        package='fractal_tactile_exploration', executable='sim_sensor_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'tactile_map', 'tip_frame': 'omega_contact_tip',
            'surface.mode': LaunchConfiguration('surface_mode'),
            'surface.origin_x': LaunchConfiguration('surface_origin_x'),
            'surface.origin_y': LaunchConfiguration('surface_origin_y'),
            'surface.size_x': LaunchConfiguration('surface_size_x'),
            'surface.size_y': LaunchConfiguration('surface_size_y'),
            'surface.base_z': LaunchConfiguration('surface_base_z'),
            'surface.bump_x': LaunchConfiguration('bump_x'),
            'surface.bump_y': LaunchConfiguration('bump_y'),
            'surface.bump_height': LaunchConfiguration('bump_height'),
            'surface.bump_sigma': LaunchConfiguration('bump_sigma'),
        }])
    explorer = LifecycleNode(
        package='fractal_tactile_exploration', executable='omega_explorer_node',
        namespace='', name='omega_explorer_node', output='screen', parameters=[
            os.path.join(exploration_share, 'config', 'omega_explorer.yaml'),
            {'use_sim_time': True,
             'motion_test.frame_id': 'world',
             'exploration.max_probes': LaunchConfiguration('max_probes')},
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
    smoke_test = Node(
        package='fractal_tactile_exploration', executable='simulation_topic_test',
        output='screen', parameters=[{'expected_samples': LaunchConfiguration('max_probes')}],
        condition=IfCondition(LaunchConfiguration('run_smoke_test')))

    return LaunchDescription(surface_arguments + [
        gazebo_resource_paths, ignition_resource_paths, gz,
        robot_state_publisher, spawn_robot, spawn_joint_state_broadcaster,
        spawn_arm_controller, move_group, rviz, map_tf, sensors, explorer,
        configure, activate, smoke_test,
    ])
