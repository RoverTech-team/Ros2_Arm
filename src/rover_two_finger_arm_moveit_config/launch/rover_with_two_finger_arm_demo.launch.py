import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("gui", default_value="true",
                               description="Start RViz2 automatically.")
    )
    gui = LaunchConfiguration("gui")

    description_package = "my_robot_description"
    description_file = "rover_with_two_finger_arm.urdf.xacro"
    srdf_file = "rover_with_two_finger_arm.srdf"

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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover_two_finger_arm_description"),
            "bringup",
            "config",
            "arm_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--param-file", robot_controllers,
                   "--controller-manager", "/controller_manager"],
    )
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_two_finger_arm_moveit_config"),
                "launch", "move_group.launch.py",
            )
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "srdf_file": srdf_file,
            "moveit_config_package": "rover_two_finger_arm_moveit_config",
        }.items(),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rover_two_finger_arm_moveit_config"), "rviz", "moveit_config.rviz"]
    )
    moveit_common = PathJoinSubstitution(
        [FindPackageShare("rover_two_finger_arm_moveit_config"), "config", "moveit_common_params.yaml"]
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config], parameters=[moveit_common],
        condition=IfCondition(gui),
    )

    demo_motion_node = Node(
        package="rover_two_finger_arm_moveit_config", executable="demo_motion",
        name="demo_motion", parameters=[moveit_common], output="both",
    )
    delay_demo = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=arm_spawner, on_exit=[demo_motion_node])
    )
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[rviz_node])
    )

    nodes = [
        control_node, jsb_spawner, delay_arm, move_group_launch,
        delay_demo, delay_rviz,
    ]
    return LaunchDescription(declared_arguments + nodes)
