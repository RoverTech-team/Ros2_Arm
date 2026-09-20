import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("my_robot_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    robot_launch = os.path.join(bringup_share, "launch", "my_robot.launch.py")
    pointcloud_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan_nav2.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params_rover.yaml")
    default_map = os.path.join(bringup_share, "maps", "rover_real_map.yaml")
    rviz_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/zed/zed_node/point_cloud/cloud_registered",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/zed/scan"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={"use_rviz": "false"}.items(),
                condition=IfCondition(LaunchConfiguration("start_robot_bringup")),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[pointcloud_params],
                remappings=[
                    ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            Node(
                package="my_robot_bringup",
                executable="nav2_cmd_vel_relay.py",
                name="nav2_cmd_vel_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/diff_drive_controller/cmd_vel",
                        "output_stamped": True,
                        "frame_id": "base_footprint",
                        "max_linear_x": 0.25,
                        "max_angular_z": 0.50,
                        "command_timeout": 0.5,
                        "publish_zero_on_timeout": True,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": "false",
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
