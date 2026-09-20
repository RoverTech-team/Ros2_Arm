import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start the Tkinter panel GUI.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2.",
        )
    )

    gui = LaunchConfiguration("gui")
    start_rviz = LaunchConfiguration("rviz")

    panel_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("rover_arm_panel_description"), "urdf", "panel.urdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )
    panel_description = {"robot_description": panel_description_content}

    panel_controllers = PathJoinSubstitution(
        [FindPackageShare("rover_arm_panel_description"), "config", "panel_controller.yaml"]
    )

    panel_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="panel_ros2_control_node",
        parameters=[panel_description, panel_controllers],
        remappings=[("~/robot_description", "/panel/robot_description")],
        output="both",
    )

    panel_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="panel_robot_state_publisher",
        parameters=[panel_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/panel/controller_manager"],
    )

    panel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panel_controller",
            "--param-file", panel_controllers,
            "--controller-manager", "/panel/controller_manager",
        ],
    )

    delay_panel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[panel_controller_spawner],
        )
    )

    panel_gui_node = Node(
        package="rover_arm_panel_description",
        executable="panel_gui.py",
        name="panel_gui",
        output="both",
        condition=IfCondition(gui),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rover_arm_panel_description"), "rviz", "maintenance_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        panel_control_node,
        panel_rsp,
        joint_state_broadcaster_spawner,
        delay_panel_controller,
        panel_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
