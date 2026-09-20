import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "prefix", default_value="",
            description="Prefix for robot links and joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "run_demo", default_value="true",
            description="Run autonomous panel interaction motion demo with MoveIt2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "force_panel_facing", default_value="true",
            description="Force planner to ensure manipulator end-effector always faces the maintenance panel.",
        )
    )

    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    run_demo = LaunchConfiguration("run_demo")
    force_panel_facing = LaunchConfiguration("force_panel_facing")

    # ── Panel ──────────────────────────────────────
    panel_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_arm_panel_description"), "urdf", "panel.urdf.xacro"]
            ),
        ]
    )
    panel_description = {"robot_description": ParameterValue(panel_description_content, value_type=str)}

    panel_controllers = PathJoinSubstitution(
        [FindPackageShare("rover_arm_panel_description"), "config", "panel_controller.yaml"]
    )

    panel_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="panel",
        parameters=[panel_description, panel_controllers],
        remappings=[
            ("~/robot_description", "/panel/robot_description"),
        ],
        output="both",
    )

    panel_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="panel_robot_state_publisher",
        parameters=[panel_description],
        remappings=[
            ("robot_description", "/panel/robot_description"),
            ("joint_states", "/panel/joint_states"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/panel/controller_manager",
            "--param-file", panel_controllers,
        ],
    )

    panel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panel_controller", "--controller-manager", "/panel/controller_manager",
            "--param-file", panel_controllers,
        ],
    )

    delay_panel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[panel_controller_spawner],
        )
    )

    # ── Arm + MoveIt2 ──────────────────────────────
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
            "joint_states_topic": "/arm/joint_states",
        }.items(),
    )

    # ── Arm ros2_control ───────────────────────────
    arm_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_arm_description"), "urdf", "arm.urdf.xacro"]
            ),
        ]
    )
    arm_robot_description = {"robot_description": ParameterValue(arm_robot_description_content, value_type=str)}

    arm_controllers = PathJoinSubstitution(
        [FindPackageShare("rover_arm_description"), "bringup", "config", "arm_controller.yaml"]
    )

    arm_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm",
        parameters=[arm_robot_description, arm_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    arm_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/arm/controller_manager",
            "--param-file", arm_controllers,
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller", "--controller-manager", "/arm/controller_manager",
            "--param-file", arm_controllers,
        ],
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # ── Planning scene publisher ───────────────────
    scene_publisher_node = Node(
        package="rover_arm_panel_description",
        executable="panel_scene_publisher.py",
        name="panel_scene_publisher",
        output="both",
    )

    # ── Panel GUI ──────────────────────────────────
    panel_gui_node = Node(
        package="rover_arm_panel_description",
        executable="panel_gui.py",
        name="panel_gui",
        output="both",
        condition=IfCondition(gui),
    )

    # ── RViz ───────────────────────────────────────
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rover_arm_panel_description"), "rviz", "maintenance_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    # ── World → panel_base_frame static bridge ──
    # Connects the panel TF tree (rooted at panel_base_frame) into the
    # world frame so the panel is positioned next to the arm. Identity
    # transform: the panel URDF itself places panel_base at its world pose.
    world_to_panel_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panel_base_frame",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panel_base_frame"],
        output="log",
    )

    # ── Interaction demo node (arm-only, panel-facing) ────────────
    moveit_common_params = PathJoinSubstitution(
        [FindPackageShare("rover_arm_moveit_config"), "config", "moveit_common_params.yaml"]
    )
    interaction_demo_node = Node(
        package="rover_arm_moveit_config",
        executable="panel_interaction_demo",
        name="panel_interaction_demo",
        parameters=[
            moveit_common_params,
            {
                "arm_only": True,
                "force_panel_facing": force_panel_facing,
            },
        ],
        output="both",
        condition=IfCondition(run_demo),
    )

    delay_interaction_demo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[interaction_demo_node],
        )
    )

    nodes = [
        # Panel stack
        panel_control_node,
        panel_rsp,
        joint_state_broadcaster_spawner,
        delay_panel_controller,
        # Arm stack
        arm_control_node,
        arm_joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_interaction_demo,
        move_group_launch,
        # Integration
        world_to_panel_tf,
        scene_publisher_node,
        panel_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
