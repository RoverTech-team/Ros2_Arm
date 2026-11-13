#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    qx = (
        math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
    qy = (
        math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    )
    qz = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    )
    qw = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
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
        super().__init__(
            "world_to_odom",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self._world_bridge_frame = self.get_parameter_or(
            "world_bridge_frame",
            rclpy.parameter.Parameter(
                "world_bridge_frame",
                rclpy.Parameter.Type.STRING,
                "odom"
            )
        ).value

        self._use_gazebo = self.get_parameter_or(
            "use_gazebo",
            rclpy.parameter.Parameter(
                "use_gazebo",
                rclpy.Parameter.Type.BOOL,
                False
            )
        ).value

        # Posa iniziale del rover ricevuta dal launch.
        self._spawn_x = float(
            self.get_parameter_or(
                "robot_spawn_x",
                rclpy.parameter.Parameter(
                    "robot_spawn_x",
                    rclpy.Parameter.Type.DOUBLE,
                    0.0
                )
            ).value
        )

        self._spawn_y = float(
            self.get_parameter_or(
                "robot_spawn_y",
                rclpy.parameter.Parameter(
                    "robot_spawn_y",
                    rclpy.Parameter.Type.DOUBLE,
                    0.0
                )
            ).value
        )

        self._spawn_yaw = float(
            self.get_parameter_or(
                "robot_spawn_yaw",
                rclpy.parameter.Parameter(
                    "robot_spawn_yaw",
                    rclpy.Parameter.Type.DOUBLE,
                    0.0
                )
            ).value
        )

        # Broadcaster delle TF.
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # Il rover parte dalla posa di spawn in modalita Gazebo.
        self._x = self._spawn_x if self._use_gazebo else 0.0
        self._y = self._spawn_y if self._use_gazebo else 0.0
        self._yaw = self._spawn_yaw if self._use_gazebo else 0.0

        self._last_cmd_time = None
        self._odom_origin = None

        # Pubblica soltanto world -> map/odom.
        # La TF del pannello viene gestita dal launch.
        self._publish_static_tfs()

        if self._use_gazebo:
            self._sub = self.create_subscription(
                Odometry,
                "/diff_drive_controller/odom",
                self._odom_callback,
                10
            )

            self._timer = self.create_timer(
                0.05,
                self._publish_dynamic_tf
            )

        else:
            self._sub = self.create_subscription(
                TwistStamped,
                "/diff_drive_controller/cmd_vel",
                self._cmd_vel_callback,
                10
            )

            self._timer = self.create_timer(
                0.05,
                self._publish_dynamic_tf
            )

    def _odom_callback(self, msg: Odometry):
        # Legge la posa pubblicata dal diff_drive_controller.
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        _, _, odom_yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])

        # Memorizza la prima posa odometrica.
        # In questo modo lo spostamento viene misurato
        # rispetto alla prima lettura ricevuta.
        if self._odom_origin is None:
            self._odom_origin = (
                odom_x,
                odom_y,
                odom_yaw
            )

        origin_x, origin_y, origin_yaw = self._odom_origin

        dx = odom_x - origin_x
        dy = odom_y - origin_y

        # Compensa la differenza tra l'orientamento iniziale
        # del rover e quello iniziale dell'odometria.
        theta = self._spawn_yaw - origin_yaw

        self._x = (
            self._spawn_x
            + math.cos(theta) * dx
            - math.sin(theta) * dy
        )

        self._y = (
            self._spawn_y
            + math.sin(theta) * dx
            + math.cos(theta) * dy
        )

        self._yaw = math.atan2(
            math.sin(
                self._spawn_yaw + odom_yaw - origin_yaw
            ),
            math.cos(
                self._spawn_yaw + odom_yaw - origin_yaw
            ),
        )

    def _cmd_vel_callback(self, msg: TwistStamped):
        now = self.get_clock().now()

        if self._last_cmd_time is not None:
            dt = (
                now - self._last_cmd_time
            ).nanoseconds / 1e9

            dt = min(dt, 0.1)

            vx = msg.twist.linear.x
            wz = msg.twist.angular.z

            if abs(vx) > 0.001 or abs(wz) > 0.001:
                self._x += (
                    vx * math.cos(self._yaw) * dt
                )

                self._y += (
                    vx * math.sin(self._yaw) * dt
                )

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

        # Non pubblicare qui panel_base_frame!
        # Il suo unico publisher deve essere quello del launch.
        self._static_broadcaster.sendTransform(
            self._make_tf(
                "world",
                self._world_bridge_frame,
                0.0,
                0.0,
                0.0,
                stamp
            )
        )

    def _publish_dynamic_tf(self):
        stamp = self.get_clock().now().to_msg()

        self._tf_broadcaster.sendTransform(
            self._make_tf(
                "odom",
                "base_footprint",
                self._x,
                self._y,
                self._yaw,
                stamp
            )
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
