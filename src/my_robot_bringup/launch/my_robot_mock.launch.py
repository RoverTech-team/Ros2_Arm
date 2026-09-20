from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("use_rviz", default_value="true")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_teleop", default_value="false")
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution(["xacro"]), " ",
            PathJoinSubstitution([
                FindPackageShare("my_robot_description"),
                "urdf", "my_robot.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    controllers_config = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config", "my_robot_controllers.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "rviz", "mock_config.rviz",
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    world_to_odom = Node(
        package="my_robot_bringup",
        executable="world_to_odom.py",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_config],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", controllers_config],
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        parameters=[{"stamped": True}],
        condition=IfCondition(use_teleop),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        world_to_odom,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delay_arm_controller,
        rviz_node,
        teleop_node,
    ])
