#!/usr/bin/env python3
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration
    robot_ns = LaunchConfiguration("robot_ns")
    # left_motor_port = LaunchConfiguration("left_motor_port")
    # right_motor_port = LaunchConfiguration("right_motor_port")
    # wheel_radius = LaunchConfiguration("wheel_radius")
    # base_distance = LaunchConfiguration("base_distance")

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="rp5",
    )
    left_motor_port_launch_arg = DeclareLaunchArgument(
        "left_motor_port",
        default_value="A",
    )
    right_motor_port_launch_arg = DeclareLaunchArgument(
        "right_motor_port",
        default_value="D",
    )
    wheel_radius_launch_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.0275",
    )
    base_distance_launch_arg = DeclareLaunchArgument(
        "base_distance",
        default_value="0.12",
        # default_value="0.124",
        # default_value="0.075",
    )

    # Launch motors
    motors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("sandbox"), "launch", "motors_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_ns": LaunchConfiguration("robot_ns"),
            "left_motor_port": LaunchConfiguration("left_motor_port"),
            "right_motor_port": LaunchConfiguration("right_motor_port"),
        }.items(),
    )

    # Differential drive node
    drive = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="drive",
        name="differential_drive_controller",
        parameters=[
            {
                "wheel_radius": LaunchConfiguration("wheel_radius"),
                "base_distance": LaunchConfiguration("base_distance"),
            }
        ],
    )

    cord_node_config = os.path.join(
        get_package_share_directory("sandbox"), "config", "cord_node.yaml"
    )

    cord_node = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="cord_node",
        name="cord_node",
        parameters=[cord_node_config],
    )

    # gyro = Node(
    #     package="sandbox",
    #     namespace=robot_ns,
    #     executable="gyro",
    #     parameters=[
    #         {
    #             "port": 3,
    #         }
    #     ],
    # )

    planner_service = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="planner_service",
    )

    color_right = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="color",
        name="color_right",
        parameters=[
            {
                "port": 1,
            }
        ],
        remappings=[
            ("color", "right/color"),
        ],
    )

    color_left = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="color",
        name="color_left",
        parameters=[
            {
                "port": 2,
            }
        ],
        remappings=[
            ("color", "left/color"),
        ],
    )

    return LaunchDescription(
        [
            robot_ns_launch_arg,
            left_motor_port_launch_arg,
            right_motor_port_launch_arg,
            wheel_radius_launch_arg,
            base_distance_launch_arg,
            motors,
            drive,
            cord_node,
            color_right,
            color_left,
            planner_service,
        ]
    )
