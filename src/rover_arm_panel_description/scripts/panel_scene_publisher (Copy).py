#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import math


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


class PanelScenePublisher(Node):
    def __init__(self):
        super().__init__('panel_scene_publisher')
        self._pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self._setup_timer = self.create_timer(1.0, self.setup_scene)
        self._setup_done = False

    def setup_scene(self):
        if self._setup_done:
            if self._setup_timer:
                self._setup_timer.cancel()
            return

        try:
            # The panel_base visual mesh is at (0.165, -0.060, 0.225) in panel_base frame.
            # world_to_panel_base: xyz="1.5 0 0" rpy="0 0 1.571" (90° Z rotation).
            # In world frame: x = 1.5 - (-0.060) = 1.560, y = 0.165, z = 0.225
            q = quaternion_from_euler(0.0, 0.0, 1.571)

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.33, 0.10, 0.45]

            pose = Pose()
            pose.position.x = 1.560
            pose.position.y = 0.165
            pose.position.z = 0.225
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            obj = CollisionObject()
            obj.id = 'panel_enclosure'
            obj.header.frame_id = 'odom'
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.primitives = [box]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD

            self._pub.publish(obj)
            self.get_logger().info('Panel collision object added to planning scene.')
            self._setup_done = True
            self._setup_timer.cancel()

        except Exception as e:
            self.get_logger().warn(f'Planning scene setup failed (retrying): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PanelScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
