import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rover_arm_description",
            description="Package containing the robot URDF xacro.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="arm.urdf.xacro",
            description="URDF xacro file for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="rover_arm_moveit_config",
            description="MoveIt2 config package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for robot links and joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_file",
            default_value="rover_arm.srdf",
            description="SRDF file name for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="joint_states",
            description="Topic to subscribe for joint states.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    prefix = LaunchConfiguration("prefix")
    srdf_file = LaunchConfiguration("srdf_file")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "config",
                    srdf_file,
                ]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    moveit_params_file = os.path.join(
        get_package_share_directory("rover_arm_moveit_config"),
        "config",
        "merged_moveit_params.yaml",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("joint_states", joint_states_topic)],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_params_file,
        ],
        remappings=[("joint_states", joint_states_topic)],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        move_group_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
