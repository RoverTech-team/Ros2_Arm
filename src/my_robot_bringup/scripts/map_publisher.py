#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(OccupancyGrid, "map", qos)
        self._timer = self.create_timer(5.0, self._publish)
        self._publish()

    def _publish(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = 0.1
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.data = [0] * (100 * 100)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
