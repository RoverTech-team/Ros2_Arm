# Testing Plan

This document focuses on compile/build risks and runtime validation for the Simco EtherCAT path.

**Top 10 Most Probable Issues (Ranked)**
1. `diff_drive_controller` cmd_vel type mismatch. `teleop_twist_keyboard` is run with `stamped:=true`, but the controller config does not enable stamped input.
2. `zed_wrapper` include is unconditional in `my_robot.launch.xml`. If the package is missing, the launch fails before ros2_control is usable.
3. Xacro path resolution for slave config uses `$(find ...)` instead of `$(find-pkg-share ...)` in ROS 2, which can break runtime parameter resolution.
4. EtherCAT master privileges or device access missing. `ethercat` tools and real-time network interface often require root and udev rules.
5. Mixed hardware definitions. The URDF includes `simco_ros2_control.xacro` by default, but developers may expect mock or Arduino without updating config.
6. EtherCAT slave config mismatch. Wrong vendor/product IDs or PDO mappings prevent drive enable or velocity control.
7. Controller activation vs hardware interface availability. Controller can be active while command interfaces are not claimed or unavailable.
8. ROS 2 dependencies not installed (`ros2_control`, controllers, teleop, ethercat tools), leading to build or launch errors.
9. Joint naming mismatches between URDF and controller YAML prevent the controller from binding to interfaces.
10. Controller update rate vs hardware control frequency mismatch can lead to timeouts or unstable behavior.

**Environment Prerequisites**
1. Source ROS 2 Humble.
2. Ensure required OS packages are installed.

Commands:
```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-teleop-twist-keyboard \
  ros-humble-xacro \
  ethercat-master
```

**Build and Dependency Checks**
1. Clean previous artifacts.
2. Build the workspace and source it.

Commands:
```bash
cd /Users/giuliomastromartino/Documents/Polispace/RovertechGit/ros2_control
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

**Controller Bringup Checks**
1. Launch without ZED to isolate ros2_control.
2. Confirm controllers are active.
3. Confirm hardware interfaces are available and claimed.
4. Confirm `cmd_vel` topic type.

Commands:
```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /diff_drive_controller/cmd_vel
```

**EtherCAT Hardware Checks**
1. Confirm EtherCAT master can see devices.
2. Check slave states.

Commands:
```bash
sudo ethercat master
sudo ethercat slaves
sudo ethercat state
```

**Commanded Motion Verification**
1. Publish a `Twist` command if the controller expects `geometry_msgs/msg/Twist`.
2. Publish a `TwistStamped` command if the controller expects `geometry_msgs/msg/TwistStamped`.

Commands (Twist):
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5
```

Commands (TwistStamped):
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 5
```

**Troubleshooting Aids**
1. Increase ros2_control logging.
2. Capture driver logs.

Commands:
```bash
ros2 launch my_robot_bringup my_robot.launch.xml --ros-args --log-level debug
ros2 topic echo /rosout
```
