import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _generate_urdf(xacro_path: str, ctrl_yaml: str) -> str:
    env = os.environ.copy()
    env['use_gazebo'] = 'true'
    env['arm_ctrl_yaml'] = ctrl_yaml
    urdf = subprocess.check_output(['xacro', xacro_path], env=env).decode('utf-8')
    arm_pkg = get_package_share_directory('rover_arm_description')
    urdf = urdf.replace('package://rover_arm_description/', 'file://' + arm_pkg + '/')
    return urdf


def _resolved(pkg: str, *subpaths: str) -> str:
    return os.path.join(get_package_share_directory(pkg), *subpaths)


def generate_launch_description():
    arm_xacro = _resolved('rover_arm_description', 'urdf', 'arm.urdf.xacro')
    world_path = _resolved('rover_arm_gazebo', 'worlds', 'arm_only.world')
    rviz_config = _resolved('rover_arm_moveit_config', 'rviz', 'moveit_config.rviz')
    arm_ctrl_yaml = _resolved('rover_arm_description', 'bringup', 'config', 'arm_controller.yaml')

    arm_urdf = _generate_urdf(arm_xacro, arm_ctrl_yaml)

    arm_file = tempfile.NamedTemporaryFile(suffix='_rover_arm.urdf', delete=False, mode='w')
    arm_file.write(arm_urdf)
    arm_file.close()

    def _cleanup():
        os.unlink(arm_file.name)
    import atexit
    atexit.register(_cleanup)

    prefix = get_package_share_directory('gz_ros2_control').rsplit('/share/', 1)[0]
    gz_control_lib = os.path.join(prefix, 'lib')

    set_plugin_path = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_control_lib)

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_path],
        output='screen',
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-g', world_path],
        output='screen',
    )

    spawn_arm = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 5 && '
            'gz service -s /world/arm_demo/create '
            '--reqtype gz.msgs.EntityFactory '
            '--reptype gz.msgs.Boolean '
            '--timeout 10000 '
            '--req "$(printf \'sdf_filename: "%s" name: "rover_arm" allow_renaming: true\' '
            f"'{arm_file.name}'" ')" '
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
                get_package_share_directory('rover_arm_moveit_config'),
                'launch',
                'move_group.launch.py',
            )
        ),
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
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

    demo_motion = Node(
        package='rover_arm_moveit_config',
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

    set_gazebo_mode = SetEnvironmentVariable('use_gazebo', 'true')
    set_ctrl_yaml = SetEnvironmentVariable('arm_ctrl_yaml', arm_ctrl_yaml)
    clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='clock_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    output='screen',
    )
    return LaunchDescription([
        set_plugin_path,
        set_gazebo_mode,
        set_ctrl_yaml,
        gz_server,
        gz_gui,
        spawn_arm,
        clock_bridge,
        arm_rsp,
        move_group_launch,
        spawn_jsb,
        spawn_ctrl,
        demo_motion,
        rviz,
    ])
