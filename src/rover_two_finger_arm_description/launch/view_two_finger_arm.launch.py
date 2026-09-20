# Copyright 2023 ros2_control Development Team
# Licensed under Apache 2.0 - adapted for two-finger variant

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically with this launch file.",
        )
    )
    gui = LaunchConfiguration("gui")

    # Fix CONDA_PREFIX for joint_state_publisher_gui on macOS when launched without micromamba activation
    # python_qt_binding hard-requires CONDA_PREFIX env var
    conda_prefix = os.environ.get("CONDA_PREFIX", "/Users/giuliomastromartino/micromamba")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_two_finger_arm_description"),
                    "urdf",
                    "two_finger_arm.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_two_finger_arm_description"), "rviz", "urdf_config.rviz"]
    )

    # Joint state publishers: GUI when requested, fallback non-GUI otherwise
    # GUI requires CONDA_PREFIX (python_qt_binding), so we ensure env var is set
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        additional_env={"CONDA_PREFIX": conda_prefix},
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(gui),
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
        condition=IfCondition(gui),
    )

    # For Gazebo/mock controller cases, mimic bridge is used separately
    # Simple RViz view does NOT need it - JSP GUI directly publishes finger joints via mimic sliders
    nodes_to_start = [
        SetEnvironmentVariable("CONDA_PREFIX", conda_prefix),
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
