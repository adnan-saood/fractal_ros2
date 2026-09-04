# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    pad_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fractal_pad_description"), "launch", "view_fractal_pad.launch.py"]
            )
        )
    )

    raw_hardware = Node(
        package="paxini_raw_hardware",
        executable="paxini_raw_hardware_node",
        name="paxini_raw_hardware_node",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "publish_rate_hz": publish_rate_hz,
            }
        ],
    )
    raw_controller = Node(
        package="paxini_raw_controller",
        executable="paxini_raw_controller_node",
        name="paxini_raw_controller_node",
        output="screen",
    )

    l5325_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="l5325_paxini_frame",
        arguments=["0", "0", "0", "0", "0", "0", "fractal_L5325", "L5325_omega_link"],
    )
    s1813_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="s1813_paxini_frame",
        arguments=["0", "0", "0", "0", "0", "0", "fractal_S1813", "S1813_elite_link"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud_rate", default_value="921600"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            pad_view,
            raw_hardware,
            raw_controller,
            l5325_frame,
            s1813_frame,
        ]
    )