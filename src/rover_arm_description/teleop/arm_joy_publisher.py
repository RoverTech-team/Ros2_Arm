#!/usr/bin/env python3
"""Publish sensor_msgs/Joy from a gamepad via pygame.

The ROS `joy` driver only works with the Linux kernel joystick API, so on
macOS this node bridges pygame/SDL gamepads (e.g. a DualShock 4 over
Bluetooth) to the standard /joy topic. Axis indices follow the SDL layout:
0/1 left stick, 2 L2, 3/4 right stick, 5 R2 — triggers normalised to 0..1.

  ros2 run rover_arm_description arm_joy_publisher.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import pygame


class JoyPublisher(Node):

    def __init__(self):
        super().__init__('arm_joy_publisher')
        self.declare_parameter('rate', 30.0)     # publish Hz
        self.declare_parameter('device', 0)      # pygame joystick index
        self.declare_parameter('num_buttons', 16)

        pygame.init()
        pygame.joystick.init()
        self.js = None
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.create_timer(0.5, self.try_connect)
        self.create_timer(
            1.0 / self.get_parameter('rate').value, self.publish_joy)

    def try_connect(self):
        if self.js is not None:
            return
        if pygame.joystick.get_count() == 0:
            self.get_logger().warning(
                'no gamepad found - connect one and it will be picked up '
                'automatically', throttle_duration_sec=5.0)
            return
        index = self.get_parameter('device').value
        if index >= pygame.joystick.get_count():
            self.get_logger().error(
                f'joystick device {index} does not exist '
                f'({pygame.joystick.get_count()} connected)')
            return
        self.js = pygame.joystick.Joystick(index)
        self.js.init()
        self.get_logger().info(
            f'connected: {self.js.get_name()} '
            f'({self.js.get_numaxes()} axes, '
            f'{self.js.get_numbuttons()} buttons)')

    def publish_joy(self):
        msg = Joy()
        if self.js is None:
            msg.axes = [0.0] * 6
            msg.buttons = [0] * self.get_parameter('num_buttons').value
            self.pub.publish(msg)
            return

        pygame.event.pump()

        num_axes = self.js.get_numaxes()
        msg.axes = [float(self.js.get_axis(i)) for i in range(num_axes)]
        # SDL triggers rest at -1; normalise to 0..1 for the teleop deadman
        for trigger in (2, 5):
            if len(msg.axes) > trigger:
                msg.axes[trigger] = (msg.axes[trigger] + 1.0) / 2.0

        buttons = [0] * self.get_parameter('num_buttons').value
        for i in range(min(self.js.get_numbuttons(), len(buttons))):
            buttons[i] = int(self.js.get_button(i))
        # expose each connected hat (D-pad) as buttons 11-14 like joydev does
        for hat_idx in range(self.js.get_numhats()):
            x, y = self.js.get_hat(hat_idx)
            base = 11 + 4 * hat_idx
            if y > 0 and len(buttons) > base:
                buttons[base] = 1        # up
            if y < 0 and len(buttons) > base + 1:
                buttons[base + 1] = 1    # down
            if x < 0 and len(buttons) > base + 2:
                buttons[base + 2] = 1    # left
            if x > 0 and len(buttons) > base + 3:
                buttons[base + 3] = 1    # right
        msg.buttons = buttons
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.js is not None:
            node.js.quit()
        pygame.joystick.quit()
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
