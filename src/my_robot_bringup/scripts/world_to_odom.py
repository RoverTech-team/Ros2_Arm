#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def euler_from_quaternion(q):
    x, y, z, w = q
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

class WorldToOdom(Node):
    def __init__(self):
        # In Jazzy è essenziale abilitare questi flag per ereditare correttamente use_sim_time dai file di lancio
        super().__init__("world_to_odom", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self._world_bridge_frame = self.get_parameter_or("world_bridge_frame", rclpy.parameter.Parameter("world_bridge_frame", rclpy.Parameter.Type.STRING, "odom")).value
        self._use_gazebo = self.get_parameter_or("use_gazebo", rclpy.parameter.Parameter("use_gazebo", rclpy.Parameter.Type.BOOL, False)).value

        # Separiamo nettamente: broadcaster dinamico per il rover, statico per il pannello
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_cmd_time = None

        # 1. Inviamo le TF statiche del pannello UNA SOLA VOLTA all'avvio
        self._publish_static_tfs()

        # 2. Impostiamo timer e sub solo per aggiornare dinamicamente il rover
        if self._use_gazebo:
            self._sub = self.create_subscription(Odometry, "/diff_drive_controller/odom", self._odom_callback, 10)
            self._timer = self.create_timer(0.05, self._publish_dynamic_tf)
        else:
            self._sub = self.create_subscription(TwistStamped, "/diff_drive_controller/cmd_vel", self._cmd_vel_callback, 10)
            self._timer = self.create_timer(0.05, self._publish_dynamic_tf)

    def _odom_callback(self, msg: Odometry):
        _, _, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = yaw

    def _cmd_vel_callback(self, msg: TwistStamped):
        now = self.get_clock().now()
        if self._last_cmd_time is not None:
            dt = (now - self._last_cmd_time).nanoseconds / 1e9
            dt = min(dt, 0.1)
            vx = msg.twist.linear.x
            wz = msg.twist.angular.z
            if abs(vx) > 0.001 or abs(wz) > 0.001:
                self._x += vx * math.cos(self._yaw) * dt
                self._y += vx * math.sin(self._yaw) * dt
                self._yaw += wz * dt
        self._last_cmd_time = now

    def _make_tf(self, parent, child, x, y, yaw, stamp):
        q = quaternion_from_euler(0, 0, yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def _publish_static_tfs(self):
        stamp = self.get_clock().now().to_msg()
        # Array di TF statiche processate e inviate insieme
        tfs = [
            self._make_tf("world", "panel_base_frame", 0.0, 0.0, 0.0, stamp),
            self._make_tf("world", self._world_bridge_frame, 0.0, 0.0, 0.0, stamp)
        ]
        self._static_broadcaster.sendTransform(tfs)

    def _publish_dynamic_tf(self):
        stamp = self.get_clock().now().to_msg()
        # Il loop aggiorna esclusivamente la trasformata in movimento (odom -> base_footprint)
        self._tf_broadcaster.sendTransform(
            self._make_tf("odom", "base_footprint", self._x, self._y, self._yaw, stamp)
        )

def main():
    rclpy.init()
    node = WorldToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
