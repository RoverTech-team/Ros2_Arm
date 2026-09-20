import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _generate_urdf(pkg_name: str, xacro_path: str, use_gazebo: bool,
                   extra_env: dict = {}) -> str:
    env = os.environ.copy()
    env['use_gazebo'] = str(use_gazebo).lower()
    env.update(extra_env)
    cmd = [
        'xacro',
        xacro_path,
    ]
    urdf = subprocess.check_output(cmd, env=env).decode('utf-8')
    pkg_dir = get_package_share_directory(pkg_name)
    urdf = urdf.replace(f'package://{pkg_name}/', pkg_dir + '/')
    return urdf


def _resolved(pkg: str, *subpaths: str) -> str:
    base = get_package_share_directory(pkg)
    return os.path.join(base, *subpaths)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'gazebo',
            default_value='false',
            description='Launch with Gazebo simulation instead of mock hardware.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Tkinter panel GUI.',
        ),
    ]

    gazebo_mode = LaunchConfiguration('gazebo')
    gui = LaunchConfiguration('gui')

    arm_xacro = _resolved('rover_arm_description', 'urdf', 'arm.urdf.xacro')
    panel_xacro = _resolved('rover_arm_panel_description', 'urdf', 'panel.urdf.xacro')
    arm_ctrl_yaml = _resolved(
        'rover_arm_description', 'bringup', 'config', 'arm_controller.yaml'
    )
    panel_ctrl_yaml = _resolved(
        'rover_arm_panel_description', 'config', 'panel_controller.yaml'
    )
    rviz_config = _resolved(
        'rover_arm_panel_description', 'rviz', 'maintenance_config.rviz'
    )

    # Eager URDF generation
    arm_urdf_false = _generate_urdf('rover_arm_description', arm_xacro, False)
    arm_urdf_true = _generate_urdf(
        'rover_arm_description', arm_xacro, True,
        extra_env={'arm_ctrl_yaml': arm_ctrl_yaml})
    panel_urdf_false = _generate_urdf('rover_arm_panel_description', panel_xacro, False)
    panel_urdf_true = _generate_urdf('rover_arm_panel_description', panel_xacro, True)

    # ================================================================
    #  MODE A — Gazebo simulation (Harmonic via ros_gz_sim)
    # ================================================================
    gz_ros2_control_lib = _resolved('gz_ros2_control', 'lib')
    ros_gz_sim_lib = _resolved('ros_gz_sim', 'lib')
    rover_arm_gazebo_lib = _resolved('rover_arm_gazebo', 'lib', 'rover_arm_gazebo')

    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ':'.join([gz_ros2_control_lib, ros_gz_sim_lib, rover_arm_gazebo_lib]),
        condition=IfCondition(gazebo_mode),
    )
    set_gazebo_mode = SetEnvironmentVariable('use_gazebo', 'true',
                                              condition=IfCondition(gazebo_mode))
    set_ctrl_yaml = SetEnvironmentVariable('arm_ctrl_yaml', arm_ctrl_yaml,
                                            condition=IfCondition(gazebo_mode))

    # Temp URDF files for spawning
    arm_file = tempfile.NamedTemporaryFile(
        suffix='_rover_arm.urdf', delete=False, mode='w'
    )
    arm_file.write(arm_urdf_true)
    arm_file.close()

    panel_file = tempfile.NamedTemporaryFile(
        suffix='_maintenance_panel.urdf', delete=False, mode='w'
    )
    panel_file.write(panel_urdf_true)
    panel_file.close()

    def _cleanup():
        os.unlink(arm_file.name)
        os.unlink(panel_file.name)

    import atexit
    atexit.register(_cleanup)

    world_path = PathJoinSubstitution([
        FindPackageShare('rover_arm_gazebo'), 'worlds', 'maintenance.world',
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-v 4 -r -s ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gazebo_mode),
    )

    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', arm_file.name,
            '-name', 'rover_arm',
            '-allow_renaming', 'true',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    spawn_panel = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', panel_file.name,
            '-name', 'maintenance_panel',
            '-allow_renaming', 'true',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    gazebo_arm_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    gazebo_arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    gazebo_panel_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/maintenance_panel/controller_manager',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    gazebo_panel_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'panel_controller',
            '--controller-manager', '/maintenance_panel/controller_manager',
        ],
        output='screen',
        condition=IfCondition(gazebo_mode),
    )

    # ================================================================
    #  MODE B — Mock hardware (default: RViz2 + ros2_control_node)
    # ================================================================
    mock_arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='arm_ros2_control_node',
        parameters=[
            {'robot_description': arm_urdf_false},
            arm_ctrl_yaml,
        ],
        remappings=[('~/robot_description', '/robot_description')],
        output='both',
        condition=UnlessCondition(gazebo_mode),
    )

    mock_panel_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='panel_ros2_control_node',
        parameters=[
            {'robot_description': panel_urdf_false},
            panel_ctrl_yaml,
        ],
        remappings=[('~/robot_description', '/panel/robot_description')],
        output='both',
        condition=UnlessCondition(gazebo_mode),
    )

    mock_arm_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/arm/controller_manager',
        ],
        condition=UnlessCondition(gazebo_mode),
    )

    mock_arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file', arm_ctrl_yaml,
            '--controller-manager', '/arm/controller_manager',
        ],
        condition=UnlessCondition(gazebo_mode),
    )

    mock_panel_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/panel/controller_manager',
        ],
        condition=UnlessCondition(gazebo_mode),
    )

    mock_panel_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'panel_controller',
            '--param-file', panel_ctrl_yaml,
            '--controller-manager', '/panel/controller_manager',
        ],
        condition=UnlessCondition(gazebo_mode),
    )

    delay_mock_arm_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mock_arm_jsb,
            on_exit=[mock_arm_ctrl],
        ),
        condition=UnlessCondition(gazebo_mode),
    )

    delay_mock_panel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mock_panel_jsb,
            on_exit=[mock_panel_ctrl],
        ),
        condition=UnlessCondition(gazebo_mode),
    )

    # ================================================================
    #  COMMON — always running regardless of mode
    # ================================================================
    arm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_robot_state_publisher',
        parameters=[{'robot_description': arm_urdf_false}],
    )

    panel_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='panel_robot_state_publisher',
        parameters=[{'robot_description': panel_urdf_false}],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            )
        ),
    )

    scene_publisher = Node(
        package='rover_arm_panel_description',
        executable='panel_scene_publisher.py',
        name='panel_scene_publisher',
        output='both',
    )

    panel_gui = Node(
        package='rover_arm_panel_description',
        executable='panel_gui.py',
        name='panel_gui',
        output='both',
        condition=IfCondition(gui),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    # Assemble
    nodes = [
        # Mode A — Gazebo
        set_plugin_path,
        gz_sim,
        spawn_arm,
        spawn_panel,
        gazebo_arm_jsb,
        gazebo_arm_ctrl,
        gazebo_panel_jsb,
        gazebo_panel_ctrl,
        # Mode B — Mock
        mock_arm_control_node,
        mock_panel_control_node,
        mock_arm_jsb,
        delay_mock_arm_ctrl,
        mock_panel_jsb,
        delay_mock_panel_ctrl,
        # Common
        arm_rsp,
        panel_rsp,
        move_group_launch,
        scene_publisher,
        panel_gui,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
