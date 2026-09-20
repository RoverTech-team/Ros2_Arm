import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _generate_urdf(xacro_path: str, ctrl_yaml: str) -> str:
    env = os.environ.copy()
    env['use_gazebo'] = 'true'
    env['arm_ctrl_yaml'] = ctrl_yaml
    urdf = subprocess.check_output(['xacro', xacro_path], env=env).decode('utf-8')
    # For mock demo MoveIt, keep package://. For Gazebo spawn via ros_gz_sim, file must be on disk.
    # ros_gz_sim create -file will read the file, mesh package:// URIs inside are resolved by Gazebo via file:// conversion at generation time:
    # Align with gazebo_demo.launch.py: package:// -> <pkg_share>/
    for pkg in ['rover_two_finger_arm_description', 'rover_arm_description']:
        try:
            pkg_dir = get_package_share_directory(pkg)
            urdf = urdf.replace(f'package://{pkg}/', pkg_dir + '/')
        except Exception:
            pass
    return urdf


def _resolved(pkg: str, *subpaths: str) -> str:
    return os.path.join(get_package_share_directory(pkg), *subpaths)


def generate_launch_description():
    arm_xacro = _resolved('rover_two_finger_arm_description', 'urdf', 'two_finger_arm.urdf.xacro')
    arm_ctrl_yaml = _resolved('rover_two_finger_arm_description', 'bringup', 'config', 'arm_controller.yaml')
    rviz_config = _resolved('rover_two_finger_arm_moveit_config', 'rviz', 'moveit_config.rviz')

    arm_urdf = _generate_urdf(arm_xacro, arm_ctrl_yaml)

    # Temp file for ros_gz_sim create -file (must persist)
    arm_file = tempfile.NamedTemporaryFile(suffix='_two_finger_arm.urdf', delete=False, mode='w')
    arm_file.write(arm_urdf)
    arm_file.close()
    import atexit
    atexit.register(lambda: os.unlink(arm_file.name) if os.path.exists(arm_file.name) else None)

    # Plugin paths: mirror gazebo_demo.launch.py working setup
    gz_ros2_control_lib = _resolved('gz_ros2_control', 'lib')
    ros_gz_sim_lib = _resolved('ros_gz_sim', 'lib')
    rover_arm_gazebo_lib = _resolved('rover_arm_gazebo', 'lib', 'rover_arm_gazebo')

    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ':'.join([gz_ros2_control_lib, ros_gz_sim_lib, rover_arm_gazebo_lib])
    )
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')
    ogre_prefix = os.environ.get('CONDA_PREFIX', '/Users/giuliomastromartino/Documents/Polispace/ros2_humble')
    ogre_path = os.path.join(ogre_prefix, 'lib', 'OGRE-Next')
    set_ogre_path = SetEnvironmentVariable('OGRE2_RESOURCE_PATH', ogre_path)
    set_use_gazebo = SetEnvironmentVariable('use_gazebo', 'true')
    set_ctrl_yaml = SetEnvironmentVariable('arm_ctrl_yaml', arm_ctrl_yaml)

    world_path = PathJoinSubstitution([
        FindPackageShare('rover_arm_gazebo'), 'worlds', 'arm_only.world',
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
    )

    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', arm_file.name,
            '-name', 'rover_two_finger_arm',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    arm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_robot_state_publisher',
        parameters=[{'robot_description': arm_urdf}],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_two_finger_arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            )
        ),
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', arm_ctrl_yaml,
        ],
        output='screen',
    )

    spawn_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_arm_ctrl',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', arm_ctrl_yaml,
        ],
        output='screen',
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_gripper_ctrl',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', arm_ctrl_yaml,
        ],
        output='screen',
    )

    demo_motion = Node(
        package='rover_two_finger_arm_moveit_config',
        executable='demo_motion',
        name='demo_motion',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        set_plugin_path,
        set_gz_ip,
        set_ogre_path,
        set_use_gazebo,
        set_ctrl_yaml,
        gz_sim,
        spawn_arm,
        arm_rsp,
        move_group_launch,
        spawn_jsb,
        spawn_ctrl,
        spawn_gripper,
        demo_motion,
        rviz,
    ])
