"""camera_panel_arm: rover + braccio + pannello in Gazebo, con la camera Gemini.

Derivato da prova_demo_launch.py (panel_interaction_demo). Differenze:
  - il pannello viene spawnato in Gazebo davanti al rover (PANEL_X/Y/Z qui sotto),
    nello stesso punto in cui la TF statica lo mette per RViz/MoveIt;
  - bridge ROS<->Gazebo dei topic della Gemini (/camera/rgbd/...);
  - rqt_image_view che mostra cio' che vede la camera;
  - use_gazebo e' true di default e la demo automatica e' spenta (run_demo:=true per accenderla);
  - allow_renaming false: se in Gazebo c'e' gia' un robot con lo stesso nome lo spawn
    fallisce invece di creare rover_arm_0, rover_arm_1...;
  - OGRE2_RESOURCE_PATH impostato solo se la cartella esiste;
  - controlli all'avvio su URDF e world, stampati nel terminale.
"""
import atexit
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, Shutdown, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Single-host demo: force loopback-only DDS discovery. The host has multiple
# network interfaces (bridge0, PROMISC USB adapters) that break multicast
# discovery; without this, no ROS node can see any other.
os.environ["ROS_LOCALHOST_ONLY"] = "1"


# Posizione del pannello nel mondo (unica fonte: usata sia per lo spawn in
# Gazebo sia per la TF statica che vedono RViz e MoveIt). Se il pannello
# risulta nel posto sbagliato in Gazebo, cambia solo questi valori.
PANEL_X = 1.5
PANEL_Y = 0.0
PANEL_Z = 0.02


def _preflight(urdf_path, world_path):
    """Stampa avvisi se la camera o il sensore non sono nel modello/world."""
    problems = []
    try:
        with open(urdf_path) as f:
            urdf = f.read()
    except OSError:
        urdf = ""
    try:
        with open(world_path) as f:
            world = f.read()
    except OSError:
        world = ""

    if "gemini_camera_link" not in urdf:
        problems.append(
            "nell'URDF usato da Gazebo non c'e' gemini_camera_link: la Gemini non e' nel "
            "modello (manca l'include di camera_arm.urdf.xacro in my_robot_mock.urdf.xacro)")
    elif "rgbd_camera" not in urdf:
        problems.append(
            "gemini_camera_link c'e' ma manca il sensore rgbd_camera: Gazebo non generera' dati")
    if "gz-sim-sensors-system" not in urdf and "gz-sim-sensors-system" not in world:
        problems.append(
            "non trovo il plugin gz-sim-sensors-system ne' nell'URDF ne' nel world: "
            "i sensori camera non produrranno dati")
    if 'name="panel_demo"' not in world and "name='panel_demo'" not in world:
        problems.append(
            "il world non si chiama panel_demo: i comandi gz service di spawn falliranno")

    if problems:
        for p in problems:
            print("[camera_panel_arm] ATTENZIONE: " + p)
    else:
        print("[camera_panel_arm] controllo URDF/world: Gemini, sensore e world ok")


def _generate_gazebo_urdf() -> str:
    xacro_path = os.path.join(
        get_package_share_directory("my_robot_description"),
        "urdf", "my_robot_mock.urdf.xacro",
    )
    ctrl_yaml = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config", "my_robot_mock_controllers.yaml",
    )
    env = os.environ.copy()
    env["use_gazebo"] = "true"
    env["arm_ctrl_yaml"] = ctrl_yaml
    urdf = subprocess.check_output(["xacro", xacro_path], env=env).decode("utf-8")
    arm_pkg = get_package_share_directory("rover_arm_description")
    urdf = urdf.replace("package://rover_arm_description/", arm_pkg + "/")
    f = tempfile.NamedTemporaryFile(suffix="_my_robot.urdf", delete=False, mode="w")
    f.write(urdf)
    f.close()
    atexit.register(lambda: os.unlink(f.name))
    return f.name


def _generate_gazebo_panel_urdf() -> str:
    xacro_path = os.path.join(
        get_package_share_directory("rover_arm_panel_description"),
        "urdf", "panel.urdf.xacro",
    )
    ctrl_yaml = os.path.join(
        get_package_share_directory("rover_arm_panel_description"),
        "config", "panel_controller.yaml",
    )
    env = os.environ.copy()
    env["use_gazebo"] = "true"
    env["panel_ctrl_yaml"] = ctrl_yaml
    urdf = subprocess.check_output(["xacro", xacro_path], env=env).decode("utf-8")
    panel_pkg = get_package_share_directory("rover_arm_panel_description")
    urdf = urdf.replace("package://rover_arm_panel_description/", panel_pkg + "/")
    f = tempfile.NamedTemporaryFile(suffix="_panel.urdf", delete=False, mode="w")
    f.write(urdf)
    f.close()
    atexit.register(lambda: os.unlink(f.name))
    return f.name


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("use_nav2", default_value="true",
                              description="Launch Nav2 navigation stack (requires Nav2 packages installed)")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_gazebo", default_value="true",
                              description="Launch Gazebo Harmonic simulation for rover+arm physics")
    )
    declared_arguments.append(
        DeclareLaunchArgument("force_panel_facing", default_value="false",
                              description="Force planner to ensure manipulator end-effector always faces the maintenance panel")
    )
    declared_arguments.append(
        DeclareLaunchArgument("run_demo", default_value="false",
                              description="Avvia anche la demo automatica panel_interaction_demo (il braccio si muove)")
    )
    declared_arguments.append(
        DeclareLaunchArgument("show_camera_image", default_value="true",
                              description="Apre rqt_image_view su /camera/rgbd/image (solo con use_gazebo:=true)")
    )
    use_nav2 = LaunchConfiguration("use_nav2")
    use_gazebo = LaunchConfiguration("use_gazebo")
    force_panel_facing = LaunchConfiguration("force_panel_facing")
    run_demo = LaunchConfiguration("run_demo")
    show_camera_image = LaunchConfiguration("show_camera_image")

    # ── Panel (shared, unchanged between mock/gazebo) ────────────
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
        name="panel_ros2_control",
        parameters=[panel_description, panel_controllers],
        remappings=[
            ("robot_description", "/panel/robot_description"),
            ("joint_states", "/panel/joint_states"),
        ],
        output="both",
        condition=UnlessCondition(use_gazebo),
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
        condition=UnlessCondition(use_gazebo),
    )

    # Gazebo mode: the panel lives in gz; its gz_ros2_control plugin
    # (namespace /panel) fetches robot_description from the params
    # service of /panel/panel_robot_state_publisher, so the RSP must
    # serve the gazebo-expanded URDF (GazeboSimSystem hardware).
    gazebo_panel_urdf_file = _generate_gazebo_panel_urdf()
    gazebo_panel_description_content = open(gazebo_panel_urdf_file).read()
    gazebo_panel_description = {
        "robot_description": ParameterValue(gazebo_panel_description_content, value_type=str)
    }

    gazebo_panel_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="panel_robot_state_publisher",
        namespace="panel",
        parameters=[gazebo_panel_description],
        remappings=[
            ("robot_description", "/panel/robot_description"),
            ("joint_states", "/panel/joint_states"),
        ],
        output="both",
        condition=IfCondition(use_gazebo),
    )

    panel_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/panel_ros2_control",
        ],
        condition=UnlessCondition(use_gazebo),
    )

    panel_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panel_controller",
            "--param-file", panel_controllers,
            "--controller-manager", "/panel_ros2_control",
        ],
        condition=UnlessCondition(use_gazebo),
    )

    delay_panel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=panel_jsb_spawner,
            on_exit=[panel_ctrl_spawner],
        )
    )

    # ════════════════════════════════════════════════════════════
    # MOCK MODE: Combined rover + arm (use_gazebo=false)
    # ════════════════════════════════════════════════════════════
    mock_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"]
            ),
        ]
    )
    mock_robot_description = ParameterValue(mock_robot_description_content, value_type=str)

    mock_controllers_config = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config", "my_robot_controllers.yaml",
    ])

    mock_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": mock_robot_description}, mock_controllers_config],
        output="both",
        condition=UnlessCondition(use_gazebo),
    )
    camera_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    ],
    output="screen",
    condition=IfCondition(use_gazebo)
)
    mock_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=UnlessCondition(use_gazebo),
    )

    mock_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        condition=UnlessCondition(use_gazebo),
    )

    clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    output="screen",
    condition=IfCondition(use_gazebo)
)

    mock_arm_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", mock_controllers_config],
        condition=UnlessCondition(use_gazebo),
    )

    mock_gripper_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--param-file", mock_controllers_config],
        condition=UnlessCondition(use_gazebo),
    )

    mock_delay_arm_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mock_jsb_spawner,
            on_exit=[mock_arm_ctrl_spawner],
        )
    )

    mock_delay_gripper_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mock_jsb_spawner,
            on_exit=[mock_gripper_ctrl_spawner],
        )
    )

    # ════════════════════════════════════════════════════════════
    # GAZEBO MODE: Combined rover + arm (use_gazebo=true)
    # ════════════════════════════════════════════════════════════
    gazebo_urdf_file = _generate_gazebo_urdf()

    gazebo_controllers_config = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config", "my_robot_mock_controllers.yaml",
    ])

    gz_prefix = get_package_share_directory("gz_ros2_control").rsplit("/share/", 1)[0]
    gz_control_lib = os.path.join(gz_prefix, "lib")
    gazebo_plugins_lib = os.path.join(
        get_package_share_directory("rover_arm_gazebo").rsplit("/share/", 1)[0],
        "lib", "rover_arm_gazebo",
    )

    set_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", gz_control_lib + ":" + gazebo_plugins_lib)
    set_gz_ip = SetEnvironmentVariable("GZ_IP", "127.0.0.1")
    # Solo se la cartella esiste (il default originale puntava a un path macOS)
    ogre2_prefix = os.environ.get("CONDA_PREFIX", "")
    ogre2_path = os.path.join(ogre2_prefix, "lib", "OGRE-Next") if ogre2_prefix else ""
    ogre2_env = (
        [SetEnvironmentVariable("OGRE2_RESOURCE_PATH", ogre2_path)]
        if ogre2_path and os.path.isdir(ogre2_path) else []
    )

    gz_world_path = os.path.join(
        get_package_share_directory("rover_arm_gazebo"),
        "worlds", "panel_interaction.world",
    )
    _preflight(gazebo_urdf_file, gz_world_path)

    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-v", "3", "-r", "-s", gz_world_path],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "3", "-g", "--render-engine-gui", "ogre2"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 5 && "
            "gz service -s /world/panel_demo/create_multiple "
            "--reqtype gz.msgs.EntityFactory_V "
            "--reptype gz.msgs.Boolean "
            "--timeout 10000 "
            f'--req "$(printf \'data: [{{ sdf_filename: "%s" name: "my_robot" allow_renaming: false pose: {{ position: {{ x: 0.0, y: 0.0, z: 0.02 }}, orientation: {{ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }} }} }}]\' '
            f"'{gazebo_urdf_file}'" ')" '
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    spawn_panel = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 8 && "
            "gz service -s /world/panel_demo/create_multiple "
            "--reqtype gz.msgs.EntityFactory_V "
            "--reptype gz.msgs.Boolean "
            "--timeout 10000 "
            f'--req "$(printf \'data: [{{ sdf_filename: "%s" name: "maintenance_panel" allow_renaming: false pose: {{ position: {{ x: {PANEL_X}, y: {PANEL_Y}, z: {PANEL_Z} }}, orientation: {{ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }} }} }}]\' '
            f"'{gazebo_panel_urdf_file}'" ')" '
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    gazebo_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        condition=IfCondition(use_gazebo),
    )

    gazebo_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file", gazebo_controllers_config,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        condition=IfCondition(use_gazebo),
    )

    gazebo_arm_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file", gazebo_controllers_config,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        condition=IfCondition(use_gazebo),
    )

    gazebo_gripper_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--param-file", gazebo_controllers_config,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        condition=IfCondition(use_gazebo),
    )

    gazebo_delay_arm_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_jsb_spawner,
            on_exit=[gazebo_arm_ctrl_spawner],
        )
    )

    gazebo_delay_gripper_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_jsb_spawner,
            on_exit=[gazebo_gripper_ctrl_spawner],
        )
    )

    gazebo_delay_diff_drive_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_gripper_ctrl_spawner,
            on_exit=[gazebo_diff_drive_spawner],
        )
    )

    # Panel joint_state_broadcaster inside the panel's gz_ros2_control
    # controller manager (/panel/controller_manager). Spawned after the
    # panel model is in the world (plugins drive the switches, so the
    # panel_controller JTC is intentionally not spawned in Gazebo mode).
    gazebo_panel_jsb_spawner = TimerAction(
        period=18.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/panel/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            condition=IfCondition(use_gazebo),
        )],
    )

    # ── world_to_odom ────────────────────────────────────────────
    # Identity bridge world → odom (non-Nav2) or world → map (Nav2):
    # odom is anchored at the rover's start pose (world origin), so
    # the bridge carries no translation; the rover pose lives only in
    # odom → base_footprint (published by world_to_odom). Nav2's
    # map_to_odom publishes map → odom on /tf_static (identity).
    world_bridge_frame = PythonExpression([
        "'map' if '", use_nav2, "' == 'true' else 'odom'"
    ])
    world_to_odom = Node(
        package="my_robot_bringup",
        executable="world_to_odom.py",
        parameters=[{"world_bridge_frame": world_bridge_frame, "use_gazebo": use_gazebo}],
    )

    # ── Nav2 (shared, conditional on use_nav2) ───────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config", "nav2_params_mock_sim.yaml",
    ])

    map_to_odom = Node(
        package="my_robot_bringup",
        executable="map_to_odom.py",
        name="map_to_odom",
        parameters=[{"x": 0.0, "y": 0.0, "yaw": 0.0}],
        output="screen",
        condition=IfCondition(use_nav2),
    )

    cmd_vel_relay = Node(
        package="my_robot_bringup",
        executable="nav2_cmd_vel_relay.py",
        name="nav2_cmd_vel_relay",
        parameters=[{
            "input_topic": "/cmd_vel",
            "output_topic": "/diff_drive_controller/cmd_vel",
            "output_stamped": True,
            "frame_id": "base_footprint",
            "max_linear_x": 0.25,
            "max_angular_z": 0.50,
            "command_timeout": 0.5,
            "publish_zero_on_timeout": True,
        }],
        output="screen",
        condition=IfCondition(use_nav2),
    )

    nav2_services = [
        "controller_server", "planner_server", "bt_navigator",
        "behavior_server", "smoother_server", "waypoint_follower",
        "velocity_smoother",
    ]

    nav2_timed_nodes = []
    for pkg, exec_name, delay in [
        ("nav2_controller",         "controller_server",    4.0),
        ("nav2_planner",            "planner_server",       6.0),
        ("nav2_bt_navigator",       "bt_navigator",         8.0),
        ("nav2_behaviors",          "behavior_server",      10.0),
        ("nav2_smoother",           "smoother_server",      12.0),
        ("nav2_waypoint_follower",  "waypoint_follower",    14.0),
        ("nav2_velocity_smoother",  "velocity_smoother",    16.0),
    ]:
        remaps = []
        if exec_name == "controller_server":
            remaps = [("cmd_vel", "cmd_vel_nav")]
        elif exec_name == "velocity_smoother":
            remaps = [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
        nav2_timed_nodes.append(TimerAction(
            period=delay,
            actions=[Node(
                package=pkg, executable=exec_name, name=exec_name,
                output="screen", parameters=[nav2_params],
                remappings=remaps,
                condition=IfCondition(use_nav2),
            )],
        ))

    lifecycle_manager_nav = TimerAction(
        period=18.0,
        actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": nav2_services,
            }],
            condition=IfCondition(use_nav2),
        )],
    )

    # ── Blank map publisher for mock+Nav2 mode ───────────────────
    map_publisher = Node(
        package="my_robot_bringup",
        executable="map_publisher.py",
        name="map_publisher",
        condition=IfCondition(use_nav2),
    )

    # ── Joint-state restamper (Gazebo only) ──────────────────────
    # gz_ros2_control stamps /joint_states with sim time; everything
    # else runs on wall time. Restamp so MoveIt and the robot_state
    # publishers get fresh states and wall-stamped TF.
    joint_states_restamper = Node(
        package="my_robot_bringup",
        executable="joint_states_restamper.py",
        name="joint_states_restamper",
        condition=IfCondition(use_gazebo),
    )

    # ── MoveIt2 (conditional on use_gazebo for description_file) ─
    move_group_mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_arm_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "description_package": "my_robot_description",
            "description_file": "my_robot.urdf.xacro",
            "srdf_file": "rover_with_arm.srdf",
        }.items(),
        condition=UnlessCondition(use_gazebo),
    )

    move_group_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_arm_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "description_package": "my_robot_description",
            "description_file": "my_robot_mock.urdf.xacro",
            "srdf_file": "rover_with_arm.srdf",
            "joint_states_topic": "/joint_states_ros",
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    # ── Scene publisher (shared) ─────────────────────────────────
    scene_publisher_node = Node(
        package="rover_arm_panel_description",
        executable="panel_scene_publisher.py",
        name="panel_scene_publisher",
        output="both",
    )

    # ── Camera Gemini: bridge Gazebo -> ROS e visualizzazione immagine ──
    # Il sensore rgbd_camera di camera_arm.urdf.xacro ha <topic>/camera/rgbd</topic>
    # e Gazebo pubblica image, depth_image, camera_info e points sotto quel nome.
    gemini_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gemini_bridge",
        arguments=[
            "/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    camera_image_viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="camera_image_viewer",
        arguments=["/camera/rgbd/image"],
        output="log",
        condition=IfCondition(PythonExpression(
            ["'", use_gazebo, "' == 'true' and '", show_camera_image, "' == 'true'"])),
    )

    # ── RViz / interaction demo (shared) ─────────────────────────
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rover_arm_panel_description"), "rviz", "maintenance_config.rviz"]
    )
    moveit_common_params = PathJoinSubstitution(
        [FindPackageShare("rover_arm_moveit_config"), "config", "moveit_common_params.yaml"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[moveit_common_params],
    )

    interaction_demo_node = Node(
        package="rover_arm_moveit_config",
        executable="panel_interaction_demo",
        name="panel_interaction_demo",
        parameters=[
            moveit_common_params,
            {"force_panel_facing": force_panel_facing},
        ],
        output="both",
        condition=IfCondition(run_demo),
    )

    mock_delay_interaction_demo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mock_arm_ctrl_spawner,
            on_exit=[interaction_demo_node],
        )
    )

    gazebo_delay_interaction_demo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_arm_ctrl_spawner,
            on_exit=[interaction_demo_node],
        )
    )

# shutdown_on_demo_exit = RegisterEventHandler(
#     event_handler=OnProcessExit(
#         target_action=interaction_demo_node,
#         # on_exit=[Shutdown(reason="Demo finished")],
#     )
# )

    panel_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="panel_static_tf",
        arguments=[str(PANEL_X), str(PANEL_Y), "0.0", "0.0", "0.0", "0.0", "odom", "panel_base_frame"],
    output="screen"
)

    nodes = [
        panel_control_node,
        panel_rsp,
        gazebo_panel_rsp,
        panel_jsb_spawner,
        delay_panel_ctrl,

        set_plugin_path,
        set_gz_ip,
        *ogre2_env,
        gz_server,
        gz_gui,
        spawn_robot,
        spawn_panel,

        mock_ros2_control_node,
        camera_bridge,
        gemini_bridge,
        camera_image_viewer,
        mock_jsb_spawner,
        mock_diff_drive_spawner,
        mock_delay_arm_ctrl,
        mock_delay_gripper_ctrl,

        gazebo_jsb_spawner,
        gazebo_delay_arm_ctrl,
        gazebo_delay_gripper_ctrl,
        gazebo_delay_diff_drive_ctrl,
        gazebo_panel_jsb_spawner,

        world_to_odom,
        map_to_odom,
        cmd_vel_relay,
        map_publisher,
        joint_states_restamper,
        clock_bridge,

        *nav2_timed_nodes,
        lifecycle_manager_nav,

        move_group_mock,
        move_group_gazebo,

        scene_publisher_node,

        mock_delay_interaction_demo,
        gazebo_delay_interaction_demo,
        # shutdown_on_demo_exit,

        rviz_node,
        panel_static_tf,
    ]

    return LaunchDescription(declared_arguments + nodes)
