# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file is https://github.com/StoglRobotics/ros_team_workspace repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="fractal_pad_description",
            description="Description package of the fractal_pad. Usually the argument is not set, \
        it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup. If changed then also joint names in the controllers' configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "input_angle_offset",
            default_value="3.141592653589793",
            description="Planar L2 angle when the GUI slider is at zero, in radians.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motor_joint_direction",
            default_value="1.0",
            description="Use -1.0 to reverse the L2 slider direction.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "l3_joint_direction",
            default_value="1.0",
            description="L3 joint direction for the current URDF joint-axis convention.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "l4_joint_direction",
            default_value="1.0",
            description="L4 joint direction for the current URDF joint-axis convention.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "solution_sign",
            default_value="-1",
            description="Selects the four-bar assembly branch; use -1 for the alternate configuration.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")
    input_angle_offset = LaunchConfiguration("input_angle_offset")
    motor_joint_direction = LaunchConfiguration("motor_joint_direction")
    l3_joint_direction = LaunchConfiguration("l3_joint_direction")
    l4_joint_direction = LaunchConfiguration("l4_joint_direction")
    solution_sign = LaunchConfiguration("solution_sign")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "fractal_pad.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "fractal_pad.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("/joint_states", "/four_bar/input_joint_states")],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    four_bar_state_node = Node(
        package="fractal_pad_controller",
        executable="four_bar_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "input_joint_states_topic": "/four_bar/input_joint_states",
                "output_joint_states_topic": "/joint_states",
                "motor_joint_name": ParameterValue([prefix, "L1_L2_joint"], value_type=str),
                "l3_joint_name": ParameterValue([prefix, "L1_L3_joint"], value_type=str),
                "l4_joint_name": ParameterValue([prefix, "L2_L4_joint"], value_type=str),
                "r1": 0.022,
                "r2": 0.0095,
                "r3": 0.0201,
                "r4": 0.02236,
                "input_angle_offset": input_angle_offset,
                "motor_joint_direction": motor_joint_direction,
                "l3_joint_direction": l3_joint_direction,
                "l4_joint_direction": l4_joint_direction,
                "solution_sign": solution_sign,
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
            four_bar_state_node,
        ]
    )
