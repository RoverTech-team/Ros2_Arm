# Plan: Mock / Simulated Environment — Full Movement with Nav2

Enable the mock (non-Gazebo) demo to move the rover with Nav2 navigation.

---

## 1. Current State

| Mode | TF Chain | Status |
|---|---|---|
| `use_gazebo:=true` | `world → map → odom → base_footprint` | ✅ Working |
| `use_gazebo:=false` (mock) | `world → map → odom → base_footprint` | ⚠️ Nav2 times out |

### Root Causes (mock non-Gazebo Nav2 failure)

1. **Disconnected TF trees** (now fixed) — `world → odom` and `map → odom` gave `odom` two parents
2. **No map data** — global costmap static layer subscribes to `/map` with no publisher
3. **Potential odometry stuck at identity** — `GenericSystem` with `calculate_dynamics: true` should integrate velocity → position, but hasn't been verified

---

## 2. Architecture (Target State)

### TF Tree — Mock Mode with Nav2

```
world
 ├── map ───────────────── (world_to_odom.py, identity → dynamic from cmd_vel)
 │    └── odom ─────────── (map_to_odom.py, static identity)
 │         └── base_footprint (diff_drive_controller, dynamic from wheel odom)
 │              └── base_link ─── wheels, arm (robot_state_publisher)
 └── panel_base_frame ──── (world_to_odom.py, identity)
      └── panel_base ───── (panel robot_state_publisher, fixed 1.5, 0, 0.571)
```

### Data Flow

```
bt_navigator → velocity_smoother → /cmd_vel
    → nav2_cmd_vel_relay → /diff_drive_controller/cmd_vel
    → diff_drive_controller → write velocity → GenericSystem (calc dynamics)
    → read position → odometry → /diff_drive_controller/odom + TF
    → world_to_odom.py → world → map TF

map_server (empty 5×5m) → /map → global_costmap → planner → path → local → cmd_vel
```

---

## 3. Phase Plan

### Phase 0 — Verify Current State

Launch Nav2 in mock mode now (with TF fix) to see what breaks.

```
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py \
  use_nav2:=true use_gazebo:=false
```

**Success criteria:** bt_navigator receives goal, costmap initializes, TF tree connected.

> **Expected:** Nav2 may fail to plan without map data. Phase 1 fixes this.

---

### Phase 1 — Provide Map for Global Costmap

The global costmap uses `plugins: [static_layer, inflation_layer]`. Without a `/map` publisher, the static layer has no data.

#### Option A: Generate blank map at launch time (recommended)

- Add helper function in `panel_interaction_demo.launch.py` that writes a 5×5m free-space YAML + 1-pixel PNG to tempfile
- Launch `nav2_map_server/map_server` with this map when `use_nav2:=true && use_gazebo:=false`
- Map: 1×1 pixel, value 205 (free), resolution 5.0

```
# empty.yaml
image: empty.png
resolution: 5.0
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

#### Option B: Remove static layer

- Change `plugins: [static_layer, inflation_layer]` → `plugins: [inflation_layer]` in `nav2_params_mock_sim.yaml`
- Simpler but less realistic — planner plans in pure free space

| | Option A (map server) | Option B (no static layer) |
|---|---|---|
| Complexity | Medium | Low |
| Realism | Higher — standard Nav2 | Lower — free space only |
| Reliability | High | Medium — straight paths always work |

---

### Phase 2 — Ensure Mock Odometry Updates

The `mobile_base.ros2_control.xacro` uses `mock_components/GenericSystem` with `calculate_dynamics: true`. This enables position integration:

```
position_n += velocity_command × dt
```

The diff_drive_controller reads wheel positions and computes odometry:

1. Receive `/diff_drive_controller/cmd_vel` (TwistStamped)
2. Convert twist → individual wheel velocities
3. Write velocity commands to wheel joints
4. Read updated wheel positions (GenericSystem integration)
5. Compute odometry pose → publish `/diff_drive_controller/odom` + `odom→base_footprint` TF

> ✅ **This should work out of the box.** Phase 2 is verification only.

**Fallback** (if odometry doesn't update): Add cmd_vel integration to `world_to_odom.py`:

- Parameter `use_gazebo` (default: `false`)
- When false: subscribe to `/diff_drive_controller/cmd_vel`, integrate twist → pose via unicycle model
- When true: subscribe to `/diff_drive_controller/odom` (current behavior)

---

## 4. Files Changed

| File | Phase | Change |
|---|---|---|
| `nav2_params_mock_sim.yaml` | P1 | Option A: no change. Option B: remove `static_layer` from global costmap |
| `panel_interaction_demo.launch.py` | P1 | Add map generator helper + map_server node for mock+Nav2 mode |
| `world_to_odom.py` | P2 | Only if needed — add `use_gazebo` param + cmd_vel integration fallback |
| `README.md` | P2 | Document mock mode Nav2 setup |

---

## 5. Verification

| Test | Expected | Check |
|---|---|---|
| `use_nav2:=true use_gazebo:=false` (full launch) | Nav2 navigates rover to goal | global_costmap initializes, planner computes path, odometry updates |
| TF tree connectivity | No "unconnected trees" errors | `ros2 run tf2_ros tf2_monitor` |
| Odometry updates | `/diff_drive_controller/odom` shows changing position | `ros2 topic echo /diff_drive_controller/odom` |
| RViz visualization | Robot model visually moves | Visual observation |

---

## 6. Risks & Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| `GenericSystem` with `calculate_dynamics=true` doesn't update fast enough (100 Hz) | Odometry stuck | Phase 2 fallback: cmd_vel integration |
| `GenericSystem` doesn't support `calculate_dynamics` in Humble | No position integration | Check source; use fallback |
| No map → planner fails despite `allow_unknown: true` | Nav2 times out | Phase 1 Option A (map_server) or B (remove static layer) |
| Progress checker timeout (12s, 0.4m radius) | Nav2 aborts early | Increase `required_movement_radius` or `movement_time_allowance` |
| local_costmap rolling window limits escape | Robot can't plan beyond 2m | Increase width/height or check target within bounds |

---

## 7. Quickstart Commands

```bash
# Build
source install/setup.zsh && colcon build --symlink-install

# Phase 0: Test current state
export ROS_LOCALHOST_ONLY=1 GZ_IP=127.0.0.1
ros2 launch rover_arm_panel_description panel_interaction_demo.launch.py \
  use_nav2:=true use_gazebo:=false

# Watch odometry
ros2 topic echo /diff_drive_controller/odom

# Watch TF tree
ros2 run tf2_ros tf2_monitor

# Send manual cmd_vel (if Nav2 not running)
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_footprint}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

---

*Status: Planning Phase — no edits made yet.*
*Generated 2026-07-26*
