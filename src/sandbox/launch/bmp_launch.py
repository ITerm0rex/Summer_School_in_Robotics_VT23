#!/usr/bin/env python3
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Launch configuration
    robot_ns = LaunchConfiguration("robot_ns")

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="rp5",
    )
    left_motor_port_launch_arg = DeclareLaunchArgument(
        "left_motor_port",
        default_value="D",
    )
    right_motor_port_launch_arg = DeclareLaunchArgument(
        "right_motor_port",
        default_value="A",
    )
    # motor_limit_launch_arg = DeclareLaunchArgument(
    #     "limit",
    #     default_value="1000",
    # )

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
            # "limit": LaunchConfiguration("limit"),
        }.items(),
    )

    # shell_input = Node(
    #     package="sandbox",
    #     namespace=robot_ns,
    #     executable="shell_input",
    #     name="shell_input",
    # )

    bmp = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="bmp",
    )

    color = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="color",
        parameters=[
            {
                "port": 1,
            }
        ],
    )
    
    # original build
    # touch = Node(
    #     package="sandbox",
    #     namespace=robot_ns,
    #     executable="touch",
    #     parameters=[
    #         {
    #             "port": 2,
    #         }
    #     ],
    # )

    gyro = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="gyro",
        parameters=[
            {
                "port": 3,
            }
        ],
    )

    ultrasonic = Node(
        package="sandbox",
        namespace=robot_ns,
        executable="ultrasonic",
        parameters=[
            {
                "port": 4,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_ns_launch_arg,
            left_motor_port_launch_arg,
            right_motor_port_launch_arg,
            bmp,
            motors,
            color,
            gyro,
            # touch,
            ultrasonic,
            # shell_input,
        ]
    )
