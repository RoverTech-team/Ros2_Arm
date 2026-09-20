from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path, get_package_share_directory
import os
from datetime import datetime


def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────

    controllers_config_path = os.path.join(
        get_package_share_directory("my_robot_bringup"), "config", "my_robot_controllers.yaml"
    )
    urdf_path = os.path.join(
        get_package_share_path("my_robot_description"), "urdf", "my_robot.urdf.xacro"
    )
    rviz_config_path = os.path.join(
        get_package_share_path("my_robot_description"), "rviz", "urdf_config.rviz"
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    # ── Bag output folder (timestamped so you never overwrite old bags) ───────

    bag_output = os.path.join(
        os.path.expanduser("~"),
        "bags",
        "rover_" + datetime.now().strftime("%Y%m%d_%H%M%S"),
    )

    # ── Arguments ─────────────────────────────────────────────────────────────

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    # ── Robot state publisher ─────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # ── ros2_control ──────────────────────────────────────────────────────────

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_config_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # ── RViz ──────────────────────────────────────────────────────────────────

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ── ZED camera ────────────────────────────────────────────────────────────

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
            )
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_tf": "false",
            "publish_map_tf": "false",
        }.items(),
    )

    # ── Teleop keyboard ───────────────────────────────────────────────────────
    # Publishes to /diff_drive_controller/cmd_vel_unstamped by default.
    # Adjust the remap if your controller expects a different topic.

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",          # opens in its own terminal window
        remappings=[
            ("/cmd_vel", "/diff_drive_controller/cmd_vel")
        ],
        parameters=[
            {"stamped": True}
        ],
    )

    # ── Bag recording ─────────────────────────────────────────────────────────
    # Records all topics needed for offline SLAM + Nav2 replay.
    # Delayed by 3 s to let all nodes finish starting up first.

    bag_record = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "/tf",
                    "/tf_static",
                    "/odom",
                    "/joint_states",
                    "/diff_drive_controller/cmd_vel_unstamped",
                    "/zed/zed_node/rgb/image_rect_color",
                    "/zed/zed_node/depth/depth_registered",
                    "/zed/zed_node/point_cloud/cloud_registered",
                    "/zed/zed_node/imu/data",
                    "/zed/zed_node/pose",
                    "/cmd_vel",
                    "/diff_drive_controller/cmd_vel",
                    "--output", bag_output,
                ],
                output="screen",
            )
        ],
    )

    # ── Assemble ──────────────────────────────────────────────────────────────

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
	joint_state_publisher_gui_node,
        diff_drive_controller_spawner,
        rviz2_node,
        zed_launch,
        teleop_node,
        bag_record,
    ])
