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

    robot_launch = os.path.join(bringup_share, "launch", "my_robot.launch.py")
    pointcloud_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan_nav2.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_nav2.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "nav2_mapping.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot_bringup", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
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
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params,
                    {
                        "use_sim_time": False,
                        "scan_topic": LaunchConfiguration("scan_topic"),
                    },
                ],
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
