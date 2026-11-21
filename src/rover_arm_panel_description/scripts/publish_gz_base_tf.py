#!/usr/bin/env python3
"""Pubblica odom -> base_footprint leggendo la posa VERA del modello in Gazebo.

Perche': con il modello spawnato dinamicamente, /joint_states descrive i giunti
ma la posa della BASE non arriva dal simulatore. Il risultato era base_link
fissa al valore nominale di spawn, mentre il modello in Gazebo si assestava
altrove: ogni proiezione (mirini, IK, view search) era calcolata su un rover che
non esiste, e la stessa posa dava frame buoni o vuoti a seconda del run.

Questo nodo legge /world/<world>/pose/info da gz-transport e trasmette la posa
del modello come odom -> base_footprint. Con questo TF segue la simulazione.

Uso:
  ros2 run rover_arm_panel_description publish_gz_base_tf.py \
      --ros-args -p world:=panel_demo -p model:=my_robot
"""
import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode


class GzBaseTf(Node):
    def __init__(self, world: str, model: str):
        super().__init__('gz_base_tf')
        self.model = model
        self.broadcaster = TransformBroadcaster(self)
        self.frames_seen = 0
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.gz = GzNode()
        topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_poses):
            raise RuntimeError(f'subscribe gz {topic} fallita')
        self.get_logger().info(f'In ascolto di {topic} per il modello {model}')

    def on_poses(self, msg):
        for pose in msg.pose:
            if pose.name != self.model:
                continue
            tf = TransformStamped()
            # Orologio di SISTEMA, non di simulazione: il resto dell'albero TF
            # gira su wall time (joint_states_restamper ristampa /joint_states
            # di gz_ros2_control). Con use_sim_time=true queste pose nascono
            # con timestamp di sim (~decine di s) e tf2 le scarta come
            # TF_OLD_DATA: il frame odom non entra MAI nell'albero e
            # lever_from_tf() resta senza base_link->odom.
            tf.header.stamp = self.system_clock.now().to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation.x = pose.orientation.x
            tf.transform.rotation.y = pose.orientation.y
            tf.transform.rotation.z = pose.orientation.z
            tf.transform.rotation.w = pose.orientation.w
            self.broadcaster.sendTransform(tf)
            self.frames_seen += 1
            if self.frames_seen == 1:
                self.get_logger().info(
                    f'prima posa: z={pose.position.z:.3f} m')
            return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='panel_demo')
    parser.add_argument('--model', default='my_robot')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GzBaseTf(args.world, args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()