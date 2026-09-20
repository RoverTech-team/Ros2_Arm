# Nav2 Mapping And Navigation On The Rover Bringup

This is the real-rover version of the simulation workflow. It uses the existing `my_robot_bringup` stack for robot state, `ros2_control`, odometry, and the ZED wrapper, then adds:

- `pointcloud_to_laserscan`: `/zed/zed_node/point_cloud/cloud_registered` to `/zed/scan`
- `slam_toolbox`: mapping from `/zed/scan`
- Nav2: localization, planning, control, and costmaps
- `nav2_cmd_vel_relay.py`: clamps Nav2 `/cmd_vel` and republishes it as stamped velocity to `/diff_drive_controller/cmd_vel`

Use this only after the rover is mechanically safe to command. Start with tiny goals.

## 1. Build

On the Jetson:

```bash
cd ~/ros2_control
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select my_robot_bringup
source install/local_setup.bash
```

If you changed `my_robot_description` or controller files too, build the full workspace:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

## 2. Real-Rover Mapping

Start mapping:

```bash
ros2 launch my_robot_bringup nav2_mapping.launch.py
```

This starts the normal robot bringup, the ZED wrapper, pointcloud-to-scan conversion, SLAM, and RViz.

In another terminal, verify the required topics and frames:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_control/install/local_setup.bash

ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /zed/scan
ros2 topic info /zed/zed_node/point_cloud/cloud_registered
ros2 node info /pointcloud_to_laserscan
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 run tf2_ros tf2_echo map odom
```

Move the rover slowly with your safest teleop method. Keep speeds low. Avoid fast spins.

Check that a map exists before saving:

```bash
ros2 topic echo --qos-durability transient_local --once /map
```

Save while `nav2_mapping.launch.py` is still running:

```bash
mkdir -p ~/ros2_control/src/my_robot_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_control/src/my_robot_bringup/maps/rover_real_map \
  --ros-args \
  -p save_map_timeout:=10.0 \
  -p map_subscribe_transient_local:=true
```

Expected files:

```text
~/ros2_control/src/my_robot_bringup/maps/rover_real_map.yaml
~/ros2_control/src/my_robot_bringup/maps/rover_real_map.pgm
```

Stop mapping with `Ctrl+C`.

## 3. Real-Rover Navigation

Start navigation with the saved map:

```bash
ros2 launch my_robot_bringup nav2_navigation.launch.py \
  map:=$HOME/ros2_control/src/my_robot_bringup/maps/rover_real_map.yaml
```

In RViz:

1. Set `Fixed Frame` to `map`.
2. Click `2D Pose Estimate`.
3. Click the rover's current location on the map and drag in its heading direction.
4. Send a very short nearby `Nav2 Goal`.

The command relay limits Nav2 commands to:

- `linear.x`: `+/-0.25 m/s`
- `angular.z`: `+/-0.50 rad/s`

Check the active navigation chain:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /diff_drive_controller/cmd_vel
ros2 run tf2_ros tf2_echo map odom
```

If Nav2 plans but the rover does not move, check the relay:

```bash
ros2 node info /nav2_cmd_vel_relay
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /diff_drive_controller/cmd_vel
```

## 4. If The Base Bringup Is Already Running

If you already started `my_robot.launch.py` in another terminal, do not start it again. Launch only the Nav2 layer:

```bash
ros2 launch my_robot_bringup nav2_mapping.launch.py start_robot_bringup:=false
```

or:

```bash
ros2 launch my_robot_bringup nav2_navigation.launch.py \
  start_robot_bringup:=false \
  map:=$HOME/ros2_control/src/my_robot_bringup/maps/rover_real_map.yaml
```

## 5. Troubleshooting

If `/zed/scan` does not publish:

```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic info /zed/zed_node/point_cloud/cloud_registered
ros2 node info /pointcloud_to_laserscan
ros2 run tf2_ros tf2_echo base_footprint zed_camera_link
```

If the point cloud topic is different on your ZED wrapper, find it and relaunch with the correct topic:

```bash
ros2 topic list | grep -E "point|cloud|zed"
ros2 launch my_robot_bringup nav2_mapping.launch.py \
  pointcloud_topic:=/ACTUAL/ZED/POINTCLOUD/TOPIC
```

If `/map` does not publish during mapping:

```bash
ros2 topic hz /zed/scan
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 node list | grep slam
```

If RViz says `Frame [map] does not exist` during navigation:

```bash
ros2 topic echo --qos-durability transient_local --once /map
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Then set the initial pose in RViz.

If the rover moves too aggressively, lower the limits in:

```text
config/nav2_params_rover.yaml
launch/nav2_navigation.launch.py
scripts/nav2_cmd_vel_relay.py
```
