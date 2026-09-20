# rover_arm_panel_description

ERC 2026 Maintenance Task Panel simulation for the rover arm sandbox. Models a physical
control panel with switches, indicators, sockets, and an electromagnet — designed for
robotic arm interaction (flipping levers, turning knobs, inserting plugs, etc.).

---

## Developer Machine

Development is done on **macOS (Apple Silicon)**. Due to DDS multicast discovery issues on
this platform, all ROS2 commands **must** be run with:

```bash
export ROS_LOCALHOST_ONLY=1
```

This disables network discovery and restricts communication to the local machine. Commands
below assume this variable is set.

### Build

From the workspace root:

```bash
source install/setup.zsh
colcon build --symlink-install
```

### Quick Launch (panel + rover + arm + demo)

```bash
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
# Without Nav2 (default on macOS — arm-only targets)
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=false
# With Nav2 (needs Gazebo physics for full TF chain)
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=true
```

---

## Package Structure

```
rover_arm_panel_description/
├── CMakeLists.txt              # Build: generates msg/srv interfaces, installs dirs
├── package.xml                 # ROS2 package manifest
├── config/
│   ├── panel_controller.yaml   # JointTrajectoryController config (12 joints)
│   └── panel_layout.yaml       # Panel geometry, element positions, task sequence
├── launch/
│   ├── panel.launch.py         # Standalone panel (no arm), optional GUI/RViz
│   ├── maintenance_demo.launch.py      # Panel + arm + MoveIt2 + GUI
│   └── panel_interaction_demo.launch.py # Full demo with C++ interaction node
├── meshes/visual/
│   └── panel_base.dae          # 3D mesh for the panel enclosure
├── msg/
│   └── PanelState.msg          # Composite state: switches, indicators, tasks, damage
├── rviz/
│   └── maintenance_config.rviz # RViz config: two RobotModel + MotionPlanning plugin
├── scripts/
│   ├── panel_gui.py            # Tkinter GUI for manual panel interaction
│   └── panel_scene_publisher.py # Publishes panel collision object to planning scene
├── srv/
│   └── SetSwitch.srv           # Service to set any element (switch, indicator, etc.)
└── urdf/
    ├── panel.urdf.xacro        # Full panel URDF: links, joints, visuals, collisions
    └── panel.ros2_control.xacro # ros2_control hardware interfaces (mock/real)
```

---

## Panel Layout

### Actuated Joints (12 total, via `JointTrajectoryController`)

| Type           | Joints                         | Control         |
|----------------|--------------------------------|-----------------|
| Main lever     | `main_switch`                  | ±1.57 rad       |
| Lever switches | `lever_switch_[1-4]`           | ±1.57 rad       |
| Rotary switches| `rotary_switch_[1-5]`          | continuous      |
| Power switches | `power_switch_[1-2]`           | continuous      |

### Fixed Elements

- **5 indicators** — `ind_1` (PWR) through `ind_5` (ERR), green spheres
- **3 sockets** — `socket_grey`, `socket_pink_1`, `socket_pink_2`
- **Electromagnet** + detachable **metal plate**
- **2 ArUco tags** — visual fiducials at top corners
- **DIN rail**, **side handle**, **black cutout** — visual/collision elements

### Task Sequence (8 steps)

1. Set MAIN switch ON
2. Set 4 lever switches ON
3. Turn 3 rotary switches
4. Insert plug into socket
5. Turn Power Switch 1 ON
6. Turn Power Switch 2 ON
7. Place plate on electromagnet
8. All 5 indicators lit

---

## Custom Interfaces

### `PanelState.msg`

Published on `/panel/state`. Contains switch positions (rad), indicator/socket/electromagnet
states, task completion booleans, element damage tracking, and damage info strings.

### `SetSwitch.srv`

Service on `/panel/set_switch`. Request: `element_id` (string), `position` (float64),
`state` (bool). Response: `success` (bool), `message` (string). Supports switches,
indicators, sockets, electromagnet, and metal plate.

---

## Nodes

### `panel_gui.py`

Tkinter-based graphical panel simulator. Displays the panel face with clickable
switches, sockets, electromagnet, and plate. Updates indicator colors, knob/pointer
positions, task checklist, and damage status in real time.

- Publishes to `/panel_controller/joint_trajectory`
- Publishes `PanelState` to `/panel/state`
- Subscribes to `/panel/joint_states`
- Provides `/panel/set_switch` service

### `panel_scene_publisher.py`

Adds a `panel_enclosure` collision object (`0.33 × 0.20 × 0.45` m box at world pose
`(1.560, 0.165, 0.225)`) to the MoveIt2 planning scene via `/collision_object`.

---

## Launch Files

### `panel.launch.py`

Standalone panel without arm. Arguments:
- `gui` (default: `true`) — Start the Tkinter GUI
- `rviz` (default: `true`) — Start RViz2

```bash
ros2 launch rover_arm_panel_description panel.launch.py
```

### `maintenance_demo.launch.py`

Panel + arm + MoveIt2 + RViz + GUI. Starts both controller managers, spawns controllers,
launches MoveIt2's move_group, adds the panel collision object to the planning scene,
and runs the autonomous panel interaction motions with MoveIt2 path orientation constraints
forcing the manipulator to always face the maintenance panel.

Arguments:
- `gui` (default: `true`) — Start the Tkinter GUI
- `prefix` (default: `""`) — Prefix for arm links/joints
- `run_demo` (default: `true`) — Run autonomous panel interaction motion demo with MoveIt2 in arm-only mode
- `force_panel_facing` (default: `true`) — Force planner to ensure manipulator end-effector always faces the maintenance panel

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch rover_arm_panel_description maintenance_demo.launch.py
```

### `panel_interaction_demo.launch.py`

Full autonomous demo. Same as `maintenance_demo` but replaces the arm subsystem with
a combined mock rover+arm system (`my_robot_mock.urdf.xacro` from `my_robot_description`)
using a single `controller_manager` with `joint_state_broadcaster`, `diff_drive_controller`,
and `arm_controller`. Includes `world_to_odom` (dynamic TF) and `map_to_odom` (static TF)
broadcasters, `nav2_cmd_vel_relay` (cmd_vel bridge), and optionally launches the full Nav2
navigation stack. The C++ interaction executable (`panel_interaction_demo` from
`rover_arm_moveit_config`) plans and executes trajectories to all 9 panel targets, with
an initial base-repositioning step via Nav2 when `use_nav2:=true`. Shuts down automatically
when done.

Arguments:
- `use_nav2` (default: `true`) — Launch Nav2 navigation stack (controller_server,
  planner_server, bt_navigator, behavior_server, smoother_server, waypoint_follower,
  velocity_smoother, lifecycle_manager). Requires Nav2 packages installed.
  Set `false` on macOS without Nav2.

```bash
export ROS_LOCALHOST_ONLY=1
# Without Nav2
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=false
# With Nav2
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py use_nav2:=true
```

---

## Configuration

### `panel_controller.yaml`

ROS2 controller config for `controller_manager`. Defines:
- `joint_state_broadcaster` — type `JointStateBroadcaster`
- `panel_controller` — type `JointTrajectoryController`
- 12 joints with position command and position+velocity state interfaces
- `allow_partial_joints_goal: true`

### `panel_layout.yaml`

Panel geometry definition. Contains positions, sizes, limits for all elements plus
the 8-step task sequence with check conditions.

---

## Dependencies

- **ROS2**: `controller_manager`, `robot_state_publisher`, `joint_state_publisher`,
  `rviz2`, `xacro`, `ros2_control`, `ros2_controllers`
- **MoveIt2**: `moveit_ros_planning_interface`
- **Nav2** (optional, `use_nav2:=true`): `nav2_msgs`, `nav2_controller`, `nav2_planner`,
  `nav2_bt_navigator`, `nav2_behaviors`, `nav2_smoother`, `nav2_velocity_smoother`,
  `nav2_lifecycle_manager`, `nav2_map_server`, `nav2_waypoint_follower`
- **Internal**: `rover_arm_description`, `rover_arm_moveit_config`, `my_robot_description`,
  `my_robot_bringup`
- **ROS2 Control**: `diff_drive_controller`
- **Runtime**: `python3-tk` (for Tkinter GUI)

---

## Topics / Services / Parameters

| Interface          | Type                        | Direction       |
|--------------------|-----------------------------|-----------------|
| `/panel/joint_states` | `sensor_msgs/JointState`  | input           |
| `/panel_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | output |
| `/panel/state`     | `rover_arm_panel_description/PanelState` | output |
| `/panel/set_switch` | `rover_arm_panel_description/SetSwitch` | service |
| `/collision_object` | `moveit_msgs/CollisionObject` | output       |
| `robot_description` | string parameter          | loaded by xacro |
| `/joint_states`     | `sensor_msgs/JointState`  | published by combined system JSB |
| `/tf`               | `tf2_msgs/TFMessage`      | rover+arm + panel transforms + `world → odom` |
| `/tf_static`        | `tf2_msgs/TFMessage`      | fixed transforms (base, arm mount, panel) |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | action server |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | odometry topic |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | velocity command input |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 velocity output (relayed by `nav2_cmd_vel_relay`) |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 action server (from `bt_navigator`) |
| `/map` → `/odom` | TF static transform | Published by `map_to_odom.py` |
| `/world` → `/odom` | TF dynamic transform | Published by `world_to_odom.py` |
| `/odom` → `/base_footprint` | TF dynamic transform | Published by `diff_drive_controller` |

### TF Tree (Gazebo mode)

```
world
 ├── odom ─────────────────── (world_to_odom.py, dynamic from wheel odometry)
 │    └── base_footprint ──── (diff_drive_controller, dynamic)
 │         └── base_link ──── (robot_state_publisher, fixed)
 │              ├── wheels   (robot_state_publisher, continuous)
 │              └── arm      (robot_state_publisher, revolute)
 └── panel_base_frame ─────── (world_to_odom.py, identity)
      └── panel_base ──────── (robot_state_publisher, fixed at 1.5, 0, 0.571)
           ├── switches      (robot_state_publisher, revolute/continuous)
           └── indicators    (robot_state_publisher, fixed)
```

All `world`-rooted transforms come from a single `world_to_odom.py` broadcaster to avoid
`TF_OLD_DATA` conflicts. The `odom → base_footprint` link is published by
`diff_drive_controller` (via `enable_odom_tf: true`). The panel's root frame is
`panel_base_frame` (no `<link name="world"/>` in panel URDF) — bridged to `world` at
identity by `world_to_odom.py`.
