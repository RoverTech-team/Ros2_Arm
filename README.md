# Rover Maintenance Panel + Arm

ROS 2 Humble workspace with a 5-DOF arm + 1-DOF gripper (6 actuated, 3 mimic fingers) and a 12-joint maintenance panel (switches, knobs, sockets, tags) for robotic manipulation R&D.

**Gazebo Harmonic (gz-sim8)** simulation integration: spawn the arm in a physics-enabled world, `gz_ros2_control` bridges to `ros2_control`, and MoveIt2 plans + executes trajectories on the simulated arm. All 5 demo poses (5-DOF `j1,j2,j3,ee_x,ee_y` + gripper `slider`) plan via CHOMP and execute on the real joint trajectory controller running inside the Gazebo server process.

## Prerequisites

- **ROS 2 Humble Hawksbill** installed via RoboStack (Micromamba)
- **macOS Sequoia**, Apple M2 (Apple Silicon)
- **Shell**: zsh
- **`ROS_LOCALHOST_ONLY=1`** required on macOS (blocks multicast)
- **Gazebo Harmonic (gz-sim8)** included with RoboStack

See `Development_env.md` for full environment setup details.

## Components

### `rover_arm_description`
- **Arm URDF**: 5-DOF arm (`j1_joint` yaw continuous, `j2_joint`/`j3_joint` ±90°, `ee_base_joint_x` pitch 0–180°, `ee_base_joint_y` roll continuous) + prismatic `slider_joint` (0–50 mm) driving 3-finger gripper via `mimic` (`31.42 rad/m`) via `arm.urdf.xacro`/`rover_arm.urdf.xacro` + `arm.urdf.xacro` now includes `arm_base_link`/`tool0`
- **ros2_control**: Mock hardware `rover_arm` (position command, position+velocity state). Config: `bringup/config/arm_controller.yaml` now `arm_controller: j1/j2/j3/ee_x/ee_y` + `gripper_controller: slider_joint`; `joints` list `j1,j2,j3,ee_x,ee_y,slider`
- **Controller manager**: `ros2_control_node` with `joint_state_broadcaster` + `arm_controller` + `gripper_controller` (all `JointTrajectoryController`)
- **Joint state broadcaster**: Publishes all state interfaces to `/joint_states`; `gripper_mimic_bridge.py` bridges `/joint_states` → `/joint_states_mimic` (`slider*31.42 → finger1/2/3`) and `robot_state_publisher` remapped to `mimic` for correct TF (fixes `GenericSystem` mimic not propagated)
- **Manual teleop**: `arm_keyboard_manual.launch.py` (mock, no Gazebo/MoveIt) + `teleop/arm_keyboard_teleop.py` (q/a j1, w/s j2, e/d j3, r/f pitch, t/g roll, y/h gripper, o/c open/close, 0 home), `teleop/arm_joy_teleop.py` + `teleop/arm_joy_publisher.py` (pygame DS4 on macOS, `joy_node` on Jetson, deadman L2/R2 0.5, watchdog 0.5s, e-stop triangle), `doc/arm_keyboard_manual_guide.html`, `scripts/gripper_mimic_bridge.py`
- **RViz**: `rviz/urdf_config.rviz` fixed `Fixed Frame: world` (was `base_link` → `No transform`), TF tree `world→arm_base_link→base→j1→j2→j3→ee_dummy→ee→slider/fingers/tool0`, `Background 48;48;48` (was `255;255;255`)

### `rover_arm_panel_description`
- **Panel URDF**: 729-line xacro with 5 lever switches, 5 rotary knobs, 2 power switches, 5 indicators, electromagnet, metal plate, 2 aruco tags, DIN rail, sockets, side handle
- **ros2_control**: Mock hardware `maintenance_panel` (12 position command interfaces, 24 state interfaces). Config: `config/panel_controller.yaml`
- **Controller manager**: `panel_controller_manager` (separate from arm CM)
- **Joint state broadcaster**: Publishes to `/panel/joint_states` (remapped to avoid topic collision)
- **Robot state publisher**: Reads `/panel/robot_description`, outputs TF for panel
- **Panel GUI**: Tkinter GUI for interactive switch/knob control (`panel_gui.py`)
- **Scene publisher**: Publishes `panel_enclosure` collision object to `/collision_object` for MoveIt planning scene
- **RViz config**: Shows both arm and panel in world frame with MotionPlanning plugin

### `rover_arm_gazebo`
- **Gazebo world**: `arm_only.world` with directional light and ground plane (no external model dependencies)
- **Launch file**: `arm_gazebo_demo.launch.py` — starts Gazebo server+GUI, spawns arm via `gz service`, runs `robot_state_publisher`, MoveIt2, controller spawners, `demo_motion`, and RViz
- **Mesh handling**: `package://` URIs converted to `file://` absolute paths at generation time (`arm_gazebo_demo.launch.py:14`); missing `j4.dae` replaced with cylinder primitive
- **ROS discovery**: Requires `ROS_LOCALHOST_ONLY=1` so the `gz_ros2_control` plugin's `AsyncParametersClient` can discover the `robot_state_publisher` node

### `gz_ros2_control`
- Built from source (`humble` branch of `ros-controls/gz_ros2_control`)
- `libgz_ros2_control-system.dylib` provides `gz_ros2_control::GazeboSimROS2ControlPlugin`
- Connects to `robot_state_publisher` via ROS 2 services to fetch the URDF
- Starts `controller_manager` inside the Gazebo server process

### `rover_arm_moveit_config`
- **MoveGroup**: Launches via `move_group.launch.py` with `ParameterValue` for `robot_description`/`semantic` and `joint_states_topic` param (for Gazebo `use_gazebo` remap)
- **`demo_motion`**: C++ node 30 s hold then 5-pose 5-DOF sequence `j1,j2,j3,ee_x,ee_y` via MoveIt + direct action client (`/controller_manager`)
- **`panel_interaction_demo`**: C++ demo Nav2 `NavigateToPose` + 9 panel targets Cartesian `tool0` + joint fallback `j1/j2/j3/jx/jy` (was `j5`, now `ee_x/jy`)
- **`ik_grasp_probe`**: Comment `slider-driven 3-finger` + joints `[j1 j2 j3 ee_x ee_y]`
- **Planning pipeline**: CHOMP/OMPL (CHOMP selected when OMPL unavailable); `merged_moveit_params.yaml` `request_adapters: AddTimeOptimalParameterization`, `projection_evaluator: joints(j1_joint,j2_joint)`, joint limits `j1 continuous` `j2/j3 ±1.5708` `ee_x 0→3.1416` `slider 0→0.05`
- **SRDF**: `rover_arm.srdf`/`rover_with_arm.srdf` groups `arm: j1/j2/j3/ee_x/ee_y` `gripper: slider` `open 0.05/closed 0.0`, `arm_chain base→tool0`, `passive mount_joint`, collision disables `j3→ee_dummy→ee→slider/fingers`
- **Launch**: `demo.launch.py` spawner `--controller-manager /controller_manager` (was `/ros2_control_node`), `rover_with_arm_demo.launch.py` new

### `my_robot_bringup` + `my_robot_description` + `rover_body_description`
- **Nav2 integration**: Full Nav2 stack via `panel_interaction_demo.launch.py` `use_nav2:=true`
- **Nav2 params**: No AMCL/EKF, `/diff_drive_controller/odom` direct, inflation-only costmap on static blank map
- **TF publishers**: `world_to_odom.py`, `map_to_odom.py`, `nav2_cmd_vel_relay.py` (`Twist→TwistStamped`)
- **Blank map**: 10×10 m `panel_workspace.yaml/.pgm`
- **Controllers**: `my_robot_controllers.yaml`/`my_robot_mock_controllers.yaml` now `body_*_wheel_joint` (6 wheels), `j1/j2/j3/ee_x/ee_y/slider`, `wheel_separation 0.977` `wheel_radius 0.125` (was `0.45/0.1` with `base_*_wheel_joint`/`joint_1..gripper_jaw`)
- **URDF**: `my_robot.urdf.xacro`/`my_robot_mock`/`rover_with_arm` now `rover_body_description/rover_body.urdf.xacro` (`body_frame`), `world→base_footprint→base_link` + `body_frame→arm_mount(0 0 0.15)→arm_base_link` + `rover_arm` + `tool0`; `rover_with_arm` arm mount `-0.16 0.24 0.20`
- **Rover body**: New `rover_body_description/` (`frame.stl`, `suspension.stl`, `wheel.stl`) + `rover_body.ros2_control.xacro`

## Current State

### Working (Mock Hardware)
- Both arm and panel controller managers load, hardware initialises/configures/activates
- `joint_state_broadcaster`, `arm_controller` (JointTrajectoryController), and `panel_controller` (JointTrajectoryController) all load, configure, and activate
- MoveIt2 starts, loads robot model, exposes planning group `arm`
- RViz displays arm and panel TF, robot models, planning scene
- Panel scene publisher adds enclosure collision object
- Robot state publishers output TF for both arm and panel
- **`panel_interaction_demo.launch.py`**: Launches all processes with correct event ordering — panel/arm controller managers, spawners (staggered via `OnProcessExit`), `move_group`, scene publisher, RViz, and the interaction demo (delayed until after arm controller spawner exits)
- **`ARM controller action server works`**: `JointTrajectoryController` creates the `/arm_controller/follow_joint_trajectory` action server; `panel_interaction_demo` sends targets and executes the full 9-target sequence
- **`ROS_LOCALHOST_ONLY=1` required** for all DDS discovery (spawners, services, topics, actions) — without it, controller_manager services are unreachable
- **`panel_interaction_demo.cpp`**: Uses `SingleThreadedExecutor` + `wait_for_action_server` for robust DDS discovery handling
- **Joint state topic collision fixed**: Panel `joint_states` remapped to `/panel/joint_states` to avoid 1500+ "Joint not found" errors in move_group
- **Nav2 integration**: Launches full Nav2 stack (controller_server, planner_server, bt_navigator, behavior_server, smoother_server, waypoint_follower, velocity_smoother) with lifecycle manager when `use_nav2:=true`
- **Nav2 graceful fallback**: When TF chain is incomplete (mock sim without physics), Nav2 goal rejection is handled gracefully — demo continues with arm-only targets
- **Blank map costmaps**: 10×10 m all-free static map for global costmap, inflation-only local costmap

### Blocked (Mock Hardware)
- **Nav2 `base_footprint→map` TF chain**: Without physics simulation (Gazebo), the `diff_drive_controller` never receives wheel odometry, so `odom→base_footprint` is never published. Nav2 costmaps time out waiting for this transform, and `NavigateToPose` goals are rejected. Fix: run with Gazebo/Ignition physics, or provide a mock odometry source.

### Working (Gazebo Harmonic Simulation)
- Arm spawns in Gazebo via `gz service`; world has inline light + ground plane
- All visual meshes (`base.dae`, `j1.dae`, `j2.dae`, `j3.dae`) render correctly via absolute filesystem paths; `j4` and `j5` use cylinder primitives
- `gz_ros2_control` plugin loads, connects to `robot_state_publisher`, receives URDF, parses 4 joints, initialises hardware, starts `controller_manager`
- `joint_state_broadcaster` and `arm_controller` (JointTrajectoryController) load, configure, and activate at `/controller_manager`
- Action server `/arm_controller/follow_joint_trajectory` available
- MoveIt2 `move_group` plans using CHOMP; `demo_motion` executes the full 5-pose sequence (all reach without collision)
- ROS topics: `/joint_states`, `/clock`, `/tf`, `/tf_static`, `/_controller/transition_event`
- Controller spawners, RSP, MoveIt2, and RViz launch alongside the Gazebo processes

### Working (Gazebo GUI on macOS — Fixed)
- OGRE2 rendering now works on Apple Silicon via **Metal** render system
- Fix: `OGRE2_RESOURCE_PATH` env var at `/path/to/ros2_humble/lib/OGRE-Next` works around a conda-build null-byte corruption in the hardcoded plugin path
- All 18 default GUI plugins load, including `MinimalScene` (3D view) — renders with native Metal, shaders compile, scene renders
- The `panel_interaction_demo.launch.py` sets this automatically when `use_gazebo:=true`
- GUI runs in a separate process from the server (macOS requirement — `gz sim -g` and `gz sim -s` cannot use Ruby `fork()`)

### `demo_motion` sequence (5-DOF `j1,j2,j3,ee_x,ee_y`, 30 s initial hold)
1. `upright` → `{0, 0.2, 0, 0.3, 0}`
2. `reach_forward` → `{0, -0.8, -0.8, 0.6, 0}`
3. `swing_left` → `{0.8, -0.4, -0.4, 0.4, 0.5}`
4. `swing_right` → `{-0.8, -0.4, -0.4, 0.4, -0.5}`
5. `upright` → `{0, 0.2, 0, 0.3, 0}`

## Building

```bash
# Source ROS 2 first (env lives outside the repo, on the internal disk)
source ~/Documents/Polispace/ros2_humble/local_setup.zsh
export ROS_LOCALHOST_ONLY=1

# Build specific packages (symlink for Python scripts)
colcon build --symlink-install --packages-select rover_arm_description rover_arm_panel_description rover_arm_moveit_config rover_arm_gazebo

# Build all packages (requires gz_ros2_control built first — see below)
colcon build --symlink-install

# Build gz_ros2_control separately (required by rover_arm_gazebo)
# Uses a CMake workaround for the TINYXML2::TINYXML2 missing target on macOS:
cmake -DCMAKE_PROJECT_INCLUDE=/tmp/cmake_tinyxml2_fix.cmake -S src/gz_ros2_control -B build/gz_ros2_control
colcon build --packages-select gz_ros2_control
# Re-source and then build remaining packages

# Re-source after build
source install/setup.zsh
```

> **Note**: For Gazebo GUI on macOS, set `OGRE2_RESOURCE_PATH` to the OGRE-Next plugin directory
> (the launch file sets this automatically when `use_gazebo:=true`):
> ```bash
> export OGRE2_RESOURCE_PATH="$PWD/../ros2_humble/lib/OGRE-Next"
> ```

> **Note**: `--symlink-install` symlinks Python scripts, URDFs, configs, and launch files, so edits in `src/` take effect without rebuilding. You only need to rebuild when modifying C++ code or message definitions.

## Launch

### Mock Hardware (no simulation)

> **Important**: `ROS_LOCALHOST_ONLY=1` is required for all mock hardware launches on macOS (DDS discovery fails without it).

```bash
# Source workspace and set required env vars
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1

# Full arm + panel interaction demo (RViz, controller_managers, panel_interaction_demo)
# Without Nav2 (default on macOS — Nav2 stack not running, arm-only targets)
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=false

# With Nav2 (requires Nav2 installed; base drives to position before arm targets)
# Note: needs Gazebo physics or mock odometry for full TF chain
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=true

# Full panel demo
ros2 launch rover_arm_panel_description maintenance_demo.launch.py

# Panel demo headless (no GUI)
ros2 launch rover_arm_panel_description maintenance_demo.launch.py gui:=false

# Full demo with arm motion (RViz, controller_manager, demo_motion)
ros2 launch rover_arm_moveit_config demo.launch.py

# Arm-only manual keyboard control (mock hardware, no Gazebo/MoveIt)
# Opens a keyboard teleop in its own xterm window + RViz, plus gamepad teleop
ros2 launch rover_arm_description arm_keyboard_manual.launch.py
# Headless variant: run the teleop yourself with
#   ros2 run rover_arm_description arm_keyboard_teleop.py
ros2 launch rover_arm_description arm_keyboard_manual.launch.py teleop:=false
# Disable the gamepad (joy_source: auto | ros | pygame | none)
ros2 launch rover_arm_description arm_keyboard_manual.launch.py joy_source:=none
# Key map, gamepad map, joint limits and troubleshooting:
#   src/rover_arm_description/doc/arm_keyboard_manual_guide.html
```

> **Manual control packages**: keyboard teleop reads single key presses from an xterm and
> publishes to the trajectory-controller topics. Gamepad teleop subscribes to
> `sensor_msgs/Joy`: on Linux (production Jetson Orin NX, Ubuntu 22.04 + Humble) it uses
> the standard `joy_node` (`sudo apt install ros-humble-joy`, user in the `input` group,
> `/dev/input/js0`); on macOS it uses the bundled pygame-based `arm_joy_publisher`
> (DS4 over Bluetooth). Both have a L2/R2 deadman, a 0.5 s stale-`/joy` watchdog that
> freezes the arm if the controller disconnects, a triangle e-stop latch, and URDF limit
> clamping. The launch auto-picks the joy source with `joy_source:=auto`.

### Gazebo Simulation

#### Combined launch with panel interaction demo (`use_gazebo:=true`)

Launches Gazebo server + OGRE2 Metal GUI, spawns the 6-wheel rover + arm, runs
`robot_state_publisher`, MoveIt2, controller spawners, and the panel interaction demo.

```bash
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_gazebo:=true use_nav2:=false
```

The launch file sets `OGRE2_RESOURCE_PATH`, `GZ_IP`, and `GZ_SIM_SYSTEM_PLUGIN_PATH`
automatically. The GUI connects to the running Gazebo server via transport.

#### Arm-only demo (individual processes)

```bash
# Source workspace first
source install/setup.zsh

# Launch everything: Gazebo server+GUI, RSP, MoveIt2, controllers, demo_motion, RViz
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION=8
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/gz_ros2_control/lib"
ros2 launch rover_arm_gazebo arm_gazebo_demo.launch.py rviz:=true
```

#### Manual server + GUI (separate terminals, macOS requirement)

```bash
# Set env vars for discovery
export ROS_LOCALHOST_ONLY=1 GZ_PARTITION=8 GZ_IP=127.0.0.1
export OGRE2_RESOURCE_PATH="/path/to/ros2_humble/lib/OGRE-Next"

# Terminal 1: Gazebo server
gz sim -v 3 -r -s install/rover_arm_gazebo/share/rover_arm_gazebo/worlds/arm_only.world

# Terminal 2: Gazebo GUI (requires OGRE2_RESOURCE_PATH on macOS)
gz sim -v 3 -g --render-engine-gui ogre2
```

## Common Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# List parameters on a node
ros2 param list /controller_manager

# List hardware interfaces
ros2 control list_hardware_interfaces

# List loaded controllers
ros2 control list_controllers

# Set a switch position (e.g., main_switch to 1.57 rad)
ros2 topic pub /main_switch/command std_msgs/Float64MultiArray "data: [1.57]"

# Echo joint states
ros2 topic echo /joint_states
ros2 topic echo /panel/joint_states
```

## File Layout

```
src/
  rover_arm_description/
    bringup/config/arm_controller.yaml         # arm 5-DOF + slider split
    bringup/launch/arm_controller.launch.py
    bringup/launch/arm_keyboard_manual.launch.py # NEW mock manual + teleop + joy + mimic bridge
    teleop/arm_keyboard_teleop.py              # NEW q/a j1 w/s j2 e/d j3 r/f pitch t/g roll y/h gripper
    teleop/arm_joy_teleop.py                   # NEW sdl/joydev L2/R2 deadman watchdog e-stop
    teleop/arm_joy_publisher.py                # NEW pygame DS4 -> /joy (fix get_numaxes)
    scripts/gripper_mimic_bridge.py            # NEW slider*31.42 -> fingers for GenericSystem
    doc/arm_keyboard_manual_guide.html         # NEW full guide
    meshes/visual/*.stl/*.dae                  # NEW base/j1/j2/j3/end_effector
    rviz/urdf_config.rviz                      # FIX world fixed, TF tree, bg 48
    urdf/arm.urdf.xacro                        # FIX arm_base_link+tool0 + rover_arm macro
    urdf/arm_mounted.urdf.xacro                # FIX same
    urdf/rover_arm.urdf.xacro                  # REWRITE 5-DOF + slider + 3-finger mimic
    urdf/rover_arm.ros2_control.xacro          # REWRITE finger state only, slider command
    CMakeLists.txt                             # + doc/teleop/scripts install
    test/test_topic_based_hw_interface.py      # j1/j2/j3/ee_x/ee_y
  rover_arm_panel_description/
    config/panel_controller.yaml
    launch/panel_interaction_demo.launch.py    # + diff_drive --param-file + gazebo_delay_diff_drive
    launch/maintenance_demo.launch.py          # + world_to_panel_tf + joint_states_topic /arm/joint_states
    urdf/panel.urdf.xacro
    urdf/panel.ros2_control.xacro
    scripts/panel_gui.py                       # fix get('size')/get('radius')
    scripts/panel_scene_publisher.py
    rviz/maintenance_config.rviz
    panel_nav.html                             # joint label update
  rover_arm_gazebo/
    worlds/arm_only.world
    launch/arm_gazebo_demo.launch.py           # file:// URIs
    scripts/demo_arm_gazebo.sh
  gz_ros2_control/
    gz_ros2_control/src/gz_ros2_control_plugin.cpp
    gz_ros2_control_demos/
  rover_body_description/                      # NEW 6-wheel rover body
    urdf/rover_body.urdf.xacro
    meshes/visual/frame.suspension.wheel.stl
  my_robot_description/
    urdf/my_robot.urdf.xacro                   # + rover_body + world/base_footprint/base_link + arm mount
    urdf/my_robot_mock.urdf.xacro              # same gazebo true
    urdf/rover_with_arm.urdf.xacro             # same, mount -0.16 0.24 0.20
    urdf/rover_body*.ros2_control.xacro        # NEW wheel + arm
  my_robot_bringup/
    config/my_robot_controllers.yaml           # body_*_wheel 0.977/0.125 + j1/j2/j3/ee_x/ee_y/slider
    config/my_robot_mock_controllers.yaml      # same 6 wheels
    config/nav2_params_mock_sim.yaml
    config/panel_workspace.yaml/.pgm
    scripts/map_to_odom.py/world_to_odom.py/nav2_cmd_vel_relay.py
  rover_arm_moveit_config/
    src/panel_interaction_demo.cpp             # j1/j2/j3/jx/jy + 30s hold
    src/demo_motion.cpp                        # 5-DOF 30s hold
    src/ik_grasp_probe.cpp                     # 3-finger comment
    config/moveit_controllers.yaml             # j1/j2/j3/ee_x/ee_y + slider
    config/joint_limits.yaml                   # j1 continuous, j2/j3 ±1.57, ee_x 0→3.14, slider 0→0.05
    config/merged_moveit_params.yaml           # AddTimeOptimal, projection j1/j2
    config/ompl_planning.yaml                  # projection j1/j2
    config/rover_arm.srdf/rover_with_arm.srdf  # arm j1/j2/j3/ee_x/ee_y gripper slider
    launch/move_group.launch.py                # ParameterValue, joint_states_topic
    launch/demo.launch.py                      # /controller_manager fix
    launch/rover_with_arm_demo.launch.py       # NEW
    rviz/moveit_config.rviz
```

## Known Issues

- **`moveit_simple_controller_manager` parameter mismatch**: The plugin requires `controller_names` to be passed via the node parameter system, but the inline dict in `move_group.launch.py` is not reachable. `move_group.execute()` therefore fails. Bypassed by `demo_motion`/`panel_interaction_demo` which plan via MoveIt and send trajectories directly to the controller's action server.
- **`ROS_LOCALHOST_ONLY=1` required for all ROS 2 communication on macOS**: Without this env var, multicast-based DDS discovery fails on macOS Sequoia. All nodes in a launch — controller_manager, spawners, RSP, MoveIt2, RViz — fail to discover each other's services/topics. Required for both mock hardware and Gazebo launches.
- **Trajectory timing**: CHOMP generates trajectories with `time_from_start = 0` for all waypoints. `demo_motion`/`panel_interaction_demo` regenerates `time_from_start` with linear spacing before sending to the controller.
- **RViz DAE mesh loading on macOS**: RViz cannot load COLLADA (`.dae`) mesh files on macOS with the default OpenGL 2.1 renderer. Only Gazebo renders the visual meshes; RViz shows collision geometry or bounding boxes instead.
- **Planning plugin ambiguity**: The `planning_plugin` parameter in `ompl_planning.yaml` is not propagated to the move_group node; CHOMP is used as fallback. OMPL can be forced by setting the `~planning_plugin` parameter explicitly.
- **Code Signature Warnings**: `install_name_tool` warnings during `colcon build` on Apple Silicon are harmless.
- **Gazebo GUI on macOS**: Requires `OGRE2_RESOURCE_PATH` env var pointing to `lib/OGRE-Next` (conda-build null-byte corruption in hardcoded string). The launch file sets it automatically when `use_gazebo:=true`. OGRE2 Metal render system works on Apple Silicon.
- **Gazebo plugin `name` attribute**: Gazebo uses the SDF `<plugin name="...">` attribute as the C++ class name to instantiate. If `name` is non-empty it must match a registered plugin class. Set `name=""` to use the first/only plugin in the library.
- **`gz_ros2_control` service timing**: The plugin's `getURDF()` method uses ROS 2 services to connect to `robot_state_publisher`. The RSP must be running before (or very shortly after) the arm is spawned; `SetEnvironmentVariable('use_gazebo', 'true')` ensures the move_group's xacro call propagates the correct env.
- **Controller_manager namespace**: `gz_ros2_control` creates the controller_manager at `/controller_manager` (global namespace). Spawners must target that path (`--controller-manager /controller_manager`), not a namespaced path.
- **`model://` URIs**: Gazebo `model://` URIs (e.g. `model://sun`, `model://ground_plane`) fail on this setup because the default model database is not reachable. Use inline `<light>` and `<model>` elements instead.
- **macOS multicast (Gazebo)**: Suppress multicast errors by setting `GZ_PARTITION=8` and `GZ_IP=127.0.0.1`.
- **Clock sync warning (Gazebo)**: MoveIt2 uses system time while Gazebo uses simulation time, causing a "Didn't receive robot state with recent timestamp" warning. Non-fatal — planning and execution work correctly.
- **Nav2 mock sim TF chain**: Without physics simulation, `diff_drive_controller` never publishes `odom→base_footprint`. Nav2 costmaps and `NavigateToPose` need this transform. Use `use_nav2:=false` on macOS, or run with Gazebo + `use_nav2:=true`.
- **Nav2 packages on macOS**: `nav2_msgs` and the full Nav2 stack are available via conda (`ros-humble-nav2-*` from `robostack-staging`). The `use_nav2` launch argument gates Nav2 node startup; set it to `true` only on machines with Nav2 installed.
- **`nav2_cmd_vel_relay.py`**: A Python relay node bridges Nav2's `/cmd_vel` output to the `diff_drive_controller`'s stamped velocity input topic. Required because Nav2 outputs `Twist` while the controller expects `TwistStamped`.
- **Gripper mimic on mock hardware**: `mock_components/GenericSystem` does not propagate `<mimic joint="slider_joint">` → fingers stay `0`. Fix: `scripts/gripper_mimic_bridge.py` (`slider*31.42 → finger1/2/3`) bridges `/joint_states` → `/joint_states_mimic` and `robot_state_publisher` remapped; verify `ros2 topic echo /joint_states_mimic` fingers `1.57` at `slider 0.05`.
- **RViz `base_link` TF**: `rover_arm_description` URDF has `world→arm_base_link→base→j1…`, not `base_link`; old `rviz/urdf_config.rviz` `Fixed Frame: base_link` caused `No transform`. Fixed to `world` + TF tree `world→arm_base_link→base→j1→j2→j3→ee_dummy→ee→slider/fingers/tool0`, `Background 48;48;48`.
- **Pygame `arm_joy_publisher`**: Older code used `get_num_axes()` (not in pygame 2.6.1 SDL 2.28) → crash. Fixed to `get_numaxes()/get_numbuttons()/get_numhats()`; `arm_joy_teleop` `sdl` profile `0:lx 1:ly 2:L2 3:rx 4:ry 5:R2` + `L1/R1` roll, `D-pad→gripper`, deadman, watchdog.
- **Controller renames**: Old `joint_1/2/3/5`, `gripper_left/right_jaw` → new `j1/j2/j3/ee_x/ee_y/slider` (arm 5-DOF + gripper 1-DOF); `my_robot_*_controllers.yaml` `body_*_wheel` `0.977/0.125`, `moveit_controllers.yaml`/`joint_limits.yaml`/`SRDF` updated accordingly.
