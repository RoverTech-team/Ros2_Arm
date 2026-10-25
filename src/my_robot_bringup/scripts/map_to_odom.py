#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class MapToOdomPublisher(Node):
    def __init__(self):
        # Abilitiamo l'importazione automatica per prendere use_sim_time senza far arrabbiare Jazzy
        super().__init__("map_to_odom", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Leggiamo i parametri in modo sicuro, senza ridichiararli
        x = self.get_parameter_or("x", rclpy.parameter.Parameter("x", rclpy.Parameter.Type.DOUBLE, 0.0)).value
        y = self.get_parameter_or("y", rclpy.parameter.Parameter("y", rclpy.Parameter.Type.DOUBLE, 0.0)).value
        yaw = self.get_parameter_or("yaw", rclpy.parameter.Parameter("yaw", rclpy.Parameter.Type.DOUBLE, 0.0)).value

        q = quaternion_from_euler(0, 0, yaw)

        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static map->odom: ({x}, {y}, {yaw})")

def main():
    rclpy.init()
    node = MapToOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
