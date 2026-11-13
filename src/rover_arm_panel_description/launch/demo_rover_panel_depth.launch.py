import atexit
import math
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


# ════════════════════════════════════════════════════════════════
# PIAZZAMENTO ROVER / PANNELLO (Gazebo mode)
# Il pannello ha GIA' nel suo Xacro il joint fisso:
#   panel_base_frame -> panel_base : xyz=(1.3,0,0), yaw=1.571
# Per conservare la posizione ORIGINALE del pannello in RViz,
# lasciamo il suo frame radice all'origine, anche in Gazebo.
# NON aggiungere qui nuovamente i 1.3 m e la rotazione di 90 gradi.
#
# La faccia frontale del pannello e' sul piano locale y=0 di panel_base:
# world x ~ 1.3; normale diretta verso -x; larghezza lungo +y.
# La posa iniziale del rover e' calcolata rispetto a quella faccia.
# ATTENZIONE: se si modifica il joint nello Xacro, aggiornare qui i
# riferimenti geometrici, oppure centralizzarli con una lettura URDF.
# ════════════════════════════════════════════════════════════════
PANEL_FACE_X = 1.30          # x della faccia (dato dal joint dello Xacro)
PANEL_WIDTH = 0.33           # larghezza del pannello nello Xacro
PANEL_FACE_CENTER_Y = PANEL_WIDTH / 2.0

PANEL_DISTANCE = 0.70        # distanza rover (origine) -> faccia del pannello, provata con RViz/Gazebo
ROBOT_X = PANEL_FACE_X - PANEL_DISTANCE   # 0.70 con PANEL_DISTANCE=0.60
ROBOT_Y = PANEL_FACE_CENTER_Y            # 0.165 (centrato davanti al pannello)
ROBOT_Z = 0.02

# Spawn del frame radice panel_base_frame; l'offset reale e' nello Xacro.
PANEL_X = 0.0
PANEL_Y = 0.0
PANEL_Z = 0.0
PANEL_YAW = 0.0
PANEL_QZ = math.sin(PANEL_YAW / 2.0)
PANEL_QW = math.cos(PANEL_YAW / 2.0)


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
        DeclareLaunchArgument("use_gazebo", default_value="false",
                              description="Launch Gazebo Harmonic simulation for rover+arm physics")
    )
    declared_arguments.append(
        DeclareLaunchArgument("force_panel_facing", default_value="false",
                              description="Force planner to ensure manipulator end-effector always faces the maintenance panel")
    )
    declared_arguments.append(
        DeclareLaunchArgument("auto_execute", default_value="false",
                              description="True: dopo ArUco/depth, calcola IK e muove il braccio SOLO in Gazebo; false: pianifica senza muovere")
    )
    declared_arguments.append(
        DeclareLaunchArgument("final_approach", default_value="false",
                              description="True: abilita seconda traiettoria IK+OMPL da 8 a 5 cm dopo il pre-approach; no contatto")
    )
    declared_arguments.append(
        DeclareLaunchArgument("attempt_contact", default_value="false",
                              description="Opt-in: dopo i 5cm richiede nuove RGB-D e prova contatto dal basso in Gazebo")
    )
    declared_arguments.append(
        DeclareLaunchArgument("contact_mode", default_value="observe",
                              description="observe=solo RGB-D nuova, touch=anche il test di contatto fisico")
    )
    declared_arguments.append(
        DeclareLaunchArgument("camera_lift", default_value="0.006",
                              description="Fallback: piccolo sollevamento camera in metri, massimo 0.008")
    )
    declared_arguments.append(
        DeclareLaunchArgument("final_standoff", default_value="0.05",
                              description="Distanza punta-leva (m) della seconda fase, dove si ristima la leva con la depth (0.045..0.075)")
    )
    declared_arguments.append(
        DeclareLaunchArgument("contact_depth", default_value="-0.001",
                              description="Profondita della punta sotto la leva rispetto alla sua faccia anteriore (m); negativo = verso il pannello")
    )
    declared_arguments.append(
        DeclareLaunchArgument("keep_visible", default_value="true",
                              description="True: dopo 8 e 5 cm controlla che il main_switch sia visibile dalla camera e, se no, inclina il polso tenendo ferma la punta")
    )
    use_nav2 = LaunchConfiguration("use_nav2")
    use_gazebo = LaunchConfiguration("use_gazebo")
    force_panel_facing = LaunchConfiguration("force_panel_facing")
    auto_execute = LaunchConfiguration("auto_execute")
    final_approach = LaunchConfiguration("final_approach")
    attempt_contact = LaunchConfiguration("attempt_contact")
    camera_lift = LaunchConfiguration("camera_lift")
    contact_mode = LaunchConfiguration("contact_mode")
    final_standoff = LaunchConfiguration("final_standoff")
    keep_visible = LaunchConfiguration("keep_visible")
    contact_depth = LaunchConfiguration("contact_depth")

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
    # "[" = solo Gazebo -> ROS (con "@" il bridge era bidirezionale senza motivo)
    arguments=[
        "/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
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
    ogre2_prefix = os.environ.get("CONDA_PREFIX", "/Users/giuliomastromartino/Documents/Polispace/ros2_humble")
    ogre2_path = os.path.join(ogre2_prefix, "lib", "OGRE-Next")
    set_ogre2_path = SetEnvironmentVariable("OGRE2_RESOURCE_PATH", ogre2_path)

    gz_world_path = os.path.join(
        get_package_share_directory("rover_arm_gazebo"),
        "worlds", "panel_interaction.world",
    )

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
            f'--req "$(printf \'data: [{{ sdf_filename: "%s" name: "my_robot" allow_renaming: true pose: {{ position: {{ x: {ROBOT_X}, y: {ROBOT_Y}, z: {ROBOT_Z} }}, orientation: {{ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }} }} }}]\' '
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
            f'--req "$(printf \'data: [{{ sdf_filename: "%s" name: "maintenance_panel" allow_renaming: true pose: {{ position: {{ x: {PANEL_X:.4f}, y: {PANEL_Y:.4f}, z: {PANEL_Z:.4f} }}, orientation: {{ x: 0.0, y: 0.0, z: {PANEL_QZ:.6f}, w: {PANEL_QW:.6f} }} }} }}]\' '
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
        parameters=[{
            "world_bridge_frame": world_bridge_frame,
            "use_gazebo": use_gazebo,
            "robot_spawn_x": ROBOT_X,
            "robot_spawn_y": ROBOT_Y,
            "robot_spawn_yaw": 0.0,
        }],
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
                "use_sim_time": True,
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
            "srdf_file": "my_robot_mock.srdf",
            "joint_states_topic": "/joint_states_ros",
            "use_sim_time": "true", # <-- FONDAMENTALE
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

    # Ricerca VISIVA autonoma: nessuna posizione degli ArUco nota in partenza.
    # Il C++ ruota a piccoli passi, poi centra il PANNELLO quando vede
    # entrambi i marker; solo alla fine pubblica /panel/align_done.
    interaction_demo_node = Node(
        package="rover_arm_moveit_config",
        executable="panel_interaction_demo_aim_aruco",
        name="panel_interaction_demo",
        parameters=[
            moveit_common_params,
            {"force_panel_facing": force_panel_facing},
            {"use_sim_time": True},
            {"joint_speed": 1.40},
            # Preorientamento iniziale della camera: coordinate nel frame base_footprint.
            # Dal setup attuale: pannello (1.30, 0.165), rover (0.90, 0.165).
            {"preaim_enabled": True},
            {"preaim_target_x": PANEL_FACE_X - ROBOT_X},
            {"preaim_target_y": PANEL_FACE_CENTER_Y - ROBOT_Y},
            # Quota iniziale da puntare (m in base_footprint):
            # centro basso del pannello, da rifinire con la misura/TF marker.
            {"preaim_target_z": 0.65},
            {"preaim_max_rotation": 2.8},
            {"max_arm_bend": 0.5},
            {"search_yaw_step": 0.20},
            {"search_pitch_step": 0.12},
            {"search_max_steps": 85},
            {"search_observe_wall_s": 0.7},
            {"center_after_found": True},
            {"center_tolerance": 0.08},
            {"center_max_steps": 15},
            {"center_gain": 0.7},
            {"center_max_step": 0.18},
            {"center_settle_time": 0.7},
            {"camera_hfov": 1.535890},
            {"camera_aspect": 848.0 / 530.0},
        ],
        output="both",
    )

    # Detector ArUco + depth avviato insieme alla simulazione.
    # Va installato nel pacchetto rover_arm_panel_description, vedi istruzioni.
    aruco_detector_node = Node(
        package="rover_arm_panel_description",
        executable="test_aruco_depth_search_center.py",
        name="aruco_detector",
        parameters=[
            {"use_sim_time": True},
            {"enable_gui": True},
            {"assume_vertical_panel": True},
        ],
        output="both",
    )

    # PRE-APPROACH AUTONOMO: sostituisce il vecchio nodo C++ che assumeva
    # erroneamente una rotazione TCP costante. La versione Python usa la stessa
    # IK della punta e lo stesso servizio OMPL/controller Gazebo gia' validati.
    # Non esistono angoli goal fissi: ogni esecuzione legge la leva LIVE.
    # Il file Python e' installato ACCANTO a questo .launch.py.
    # --wait-align: parte solo quando il demo C++ pubblica /panel/align_done.
    # auto_execute:=false -> solo piano; true -> movimento automatico in Gazebo.
    auto_script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "main_switch_pre_approach_AUTO.py")
    auto_run_mode = PythonExpression([
        "'--execute' if '", auto_execute, "' == 'true' else '--plan-only'"
    ])
    main_switch_pre_approach_node = ExecuteProcess(
        cmd=[FindExecutable(name="python3"), auto_script, "--wait-align", auto_run_mode,
             "--final-approach", final_approach,
             "--attempt-contact", attempt_contact, "--camera-lift", camera_lift,
             "--contact-mode", contact_mode,
             "--final-standoff", final_standoff, "--contact-depth", contact_depth,
             "--keep-visible", keep_visible],
        output="screen",
        condition=IfCondition(use_gazebo),
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
        arguments=[
            f"{PANEL_X:.4f}", f"{PANEL_Y:.4f}", f"{PANEL_Z:.4f}",
            f"{PANEL_YAW:.6f}", "0.0", "0.0",   # yaw pitch roll (nessuna rotazione esterna)
            "odom", "panel_base_frame",
        ],
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
        set_ogre2_path,
        gz_server,
        gz_gui,
        spawn_robot,
        spawn_panel,

        mock_ros2_control_node,
        camera_bridge,
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
        aruco_detector_node,
        main_switch_pre_approach_node,

        mock_delay_interaction_demo,
        gazebo_delay_interaction_demo,
        # shutdown_on_demo_exit,

        rviz_node,
        panel_static_tf,
    ]

    return LaunchDescription(declared_arguments + nodes)
