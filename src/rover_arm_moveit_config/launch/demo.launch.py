import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for robot links and joints.",
        )
    )

    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_arm_description"),
                    "urdf",
                    "arm.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover_arm_description"),
            "bringup",
            "config",
            "arm_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager",
        ],
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_arm_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "prefix": prefix,
        }.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("rover_arm_moveit_config"),
            "rviz",
            "moveit_config.rviz",
        ]
    )

    moveit_common_params = PathJoinSubstitution(
        [FindPackageShare("rover_arm_moveit_config"), "config", "moveit_common_params.yaml"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_common_params],
        condition=IfCondition(gui),
    )

    demo_motion_node = Node(
        package="rover_arm_moveit_config",
        executable="demo_motion",
        name="demo_motion",
        parameters=[moveit_common_params],
        output="both",
    )

    delay_demo_motion = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[demo_motion_node],
        )
    )

    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    shutdown_on_demo_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=demo_motion_node,
            on_exit=[Shutdown(reason="Demo finished")],
        )
    )

    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        move_group_launch,
        delay_demo_motion,
        delay_rviz_after_joint_state_broadcaster,
        shutdown_on_demo_exit,
    ]

    return LaunchDescription(declared_arguments + nodes)
