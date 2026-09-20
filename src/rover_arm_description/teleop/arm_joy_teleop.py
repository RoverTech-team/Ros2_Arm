#!/usr/bin/env python3
"""Gamepad teleop for the rover arm.

Subscribes to sensor_msgs/Joy and streams position targets to the running
trajectory controllers while a deadman trigger is held:

  /arm_controller/joint_trajectory     (j1, j2, j3, wrist pitch, wrist roll)
  /gripper_controller/joint_trajectory (slider)

The Joy topic comes from the standard `joy_node` on Linux (Jetson/Ubuntu) or
from the bundled arm_joy_publisher.py (pygame) on macOS.

  ros2 run rover_arm_description arm_joy_teleop.py
"""

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_keyboard_teleop import (  # sibling script, installed in the same dir
    ARM_JOINTS,
    GRIPPER_JOINTS,
    JOINT_LIMITS,
    SLIDER_CLOSED,
    SLIDER_OPEN,
)

# Axis/button indices for the two supported layouts. Sticks are -1..1 with
# rest 0; triggers are 0..1 after the publisher normalises them.
PROFILES = {
    # macOS/Windows via SDL (pygame publisher, DS4 and XInput pads)
    'sdl': {
        'lx': 0, 'ly': 1, 'rx': 3, 'ry': 4, 'l2': 2, 'r2': 5,
        'dpad_axis_x': None, 'dpad_axis_y': None,
        'dpad_up': 11, 'dpad_down': 12, 'dpad_left': 13, 'dpad_right': 14,
    },
    # Linux via the kernel joystick API (joy_node, /dev/input/js0)
    'joydev': {
        'lx': 0, 'ly': 1, 'rx': 2, 'ry': 3, 'l2': 4, 'r2': 5,
        'dpad_axis_x': 6, 'dpad_axis_y': 7,
        'dpad_up': 11, 'dpad_down': 12, 'dpad_left': 13, 'dpad_right': 14,
    },
}

# Buttons are identical on both layouts
BTN_CROSS = 0     # home: all joints to 0
BTN_CIRCLE = 1    # open gripper fully
BTN_SQUARE = 2    # close gripper fully
BTN_TRIANGLE = 3  # software e-stop toggle
BTN_L1 = 4        # wrist roll -
BTN_R1 = 5        # wrist roll +

HELP_TEXT = """
--------------------------------------------------------------------
 Rover Arm - Gamepad Control
--------------------------------------------------------------------
 left stick  X  -> j1 base yaw        right stick X -> wrist pitch
 left stick  Y  -> j2 shoulder        right stick Y -> j3 elbow
 L1 / R1         -> wrist roll -/+    D-pad up/down -> gripper open/close

 cross  = home (all joints 0)         circle = open gripper fully
 square = close gripper fully         triangle = e-stop toggle

 HOLD L2 or R2 (deadman) to enable motion - sticks do nothing otherwise
--------------------------------------------------------------------
"""

HELP_NO_JOY = """
Waiting for sensor_msgs/Joy on /joy ...

  Linux (Jetson/Ubuntu):  ros2 run joy joy_node
  macOS:                  ros2 run rover_arm_description arm_joy_publisher.py
"""


class ArmJoyTeleop(Node):

    def __init__(self):
        super().__init__('arm_joy_teleop')

        self.declare_parameter('profile', 'joydev')
        self.declare_parameter('rate', 50.0)          # control loop Hz
        self.declare_parameter('duration', 0.0)       # trajectory time [s],
                                                      # 0 -> 0.9 control periods
        self.declare_parameter('max_speed', 0.8)      # rad/s arm joints
        self.declare_parameter('wrist_speed', 1.0)    # rad/s wrist roll
        self.declare_parameter('slider_speed', 0.02)  # m/s gripper
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter(
            'joy_timeout', 0.5)  # s without /joy -> deadman released

        profile = self.get_parameter('profile').value
        if profile not in PROFILES:
            raise ValueError(
                f"unknown profile '{profile}' (expected one of "
                f"{sorted(PROFILES)})")
        self.ax = PROFILES[profile]

        self.positions = {name: 0.0 for name in ARM_JOINTS + GRIPPER_JOINTS}
        self.last_joy = None
        self.last_joy_time = 0.0
        self.prev_buttons = ()
        self.estop = False

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.create_timer(
            1.0 / self.get_parameter('rate').value, self.update)

    def on_joint_states(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = position

    def on_joy(self, msg: Joy):
        buttons = list(msg.buttons)
        # pad shorter arrays so taps on unmapped buttons still edge-detect
        buttons += [0] * (16 - len(buttons))

        if self.prev_buttons:
            for btn, action in ((BTN_CROSS, 'home'),
                                (BTN_CIRCLE, 'open_grip'),
                                (BTN_SQUARE, 'close_grip'),
                                (BTN_TRIANGLE, 'estop')):
                if buttons[btn] and not self.prev_buttons[btn]:
                    self.handle_button(action)
        self.prev_buttons = buttons
        self.last_joy = msg
        self.last_joy_time = time.monotonic()

    def handle_button(self, action: str):
        if action == 'home':
            for name in self.positions:
                self.positions[name] = 0.0
            self.publish()
            self.get_logger().info('home position commanded')
        elif action == 'open_grip':
            self.positions['slider_joint'] = SLIDER_OPEN
            self.publish()
            self.get_logger().info('gripper fully open')
        elif action == 'close_grip':
            self.positions['slider_joint'] = SLIDER_CLOSED
            self.publish()
            self.get_logger().info('gripper fully closed')
        elif action == 'estop':
            self.estop = not self.estop
            self.get_logger().warn(
                'E-STOP ENGAGED - motion disabled' if self.estop
                else 'e-stop released - motion enabled')

    def deadman_pressed(self) -> bool:
        if self.last_joy is None:
            return False
        # stale data (gamepad disconnected/publisher died) releases the
        # deadman - never keep moving on the last cached message
        if time.monotonic() - self.last_joy_time > \
                self.get_parameter('joy_timeout').value:
            return False
        axes = list(self.last_joy.axes)
        return any(
            len(axes) > self.ax[t] and axes[self.ax[t]] >= 0.5
            for t in ('l2', 'r2'))

    def stick(self, key: str) -> float:
        if self.last_joy is None:
            return 0.0
        axes = list(self.last_joy.axes)
        idx = self.ax[key]
        if len(axes) <= idx:
            return 0.0
        value = axes[idx]
        deadzone = self.get_parameter('deadzone').value
        if abs(value) < deadzone:
            return 0.0
        # rescale so output ramps from 0 at the deadzone edge to 1 at full
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def pressed(self, button: int) -> bool:
        if self.last_joy is None:
            return False
        buttons = list(self.last_joy.buttons)
        return len(buttons) > button and bool(buttons[button])

    def dpad(self) -> int:
        """+1 up (open), -1 down (close), 0 neutral."""
        axes = list(self.last_joy.axes) if self.last_joy else []
        y_axis = self.ax['dpad_axis_y']
        if y_axis is not None and len(axes) > y_axis and abs(axes[y_axis]) > 0.5:
            return 1 if axes[y_axis] > 0 else -1
        buttons = list(self.last_joy.buttons) if self.last_joy else []
        if len(buttons) > self.ax['dpad_up'] and buttons[self.ax['dpad_up']]:
            return 1
        if len(buttons) > self.ax['dpad_down'] and buttons[self.ax['dpad_down']]:
            return -1
        return 0

    def step(self, joint: str, rate: float, dt: float):
        """Advance a joint by rate*dt, clamped to its URDF limits."""
        limits = JOINT_LIMITS[joint]
        self.positions[joint] = min(
            max(self.positions[joint] + rate * dt,
                limits[0] if limits else -1e9),
            limits[1] if limits else 1e9)

    def update(self):
        dt = 1.0 / self.get_parameter('rate').value
        if not self.estop and self.deadman_pressed():
            arm_speed = self.get_parameter('max_speed').value
            wrist_speed = self.get_parameter('wrist_speed').value
            slider_speed = self.get_parameter('slider_speed').value

            self.step('j1_joint', self.stick('lx') * arm_speed, dt)
            self.step('j2_joint', -self.stick('ly') * arm_speed, dt)
            self.step('j3_joint', -self.stick('ry') * arm_speed, dt)
            self.step('ee_base_joint_x', self.stick('rx') * wrist_speed, dt)

            roll = (self.pressed(BTN_R1) - self.pressed(BTN_L1))
            self.step('ee_base_joint_y', roll * wrist_speed, dt)

            self.step('slider_joint', -self.dpad() * slider_speed, dt)

            self.publish()

    def publish(self):
        # The horizon must not exceed the publish period: each new message
        # preempts the previous trajectory, so a longer horizon makes the
        # controller oscillate between the old and the new setpoint.
        horizon = self.get_parameter('duration').value
        if horizon <= 0.0:
            horizon = 0.9 / self.get_parameter('rate').value
        duration_ns = int(horizon * 1e9)
        arm_point = JointTrajectoryPoint(
            positions=[self.positions[name] for name in ARM_JOINTS],
            time_from_start=Duration(sec=0, nanosec=duration_ns))
        self.arm_pub.publish(JointTrajectory(
            joint_names=ARM_JOINTS, points=[arm_point]))

        grip_point = JointTrajectoryPoint(
            positions=[self.positions['slider_joint']],
            time_from_start=Duration(sec=0, nanosec=duration_ns))
        self.gripper_pub.publish(JointTrajectory(
            joint_names=GRIPPER_JOINTS, points=[grip_point]))

    def status_line(self) -> str:
        deg = lambda rad: f'{math.degrees(rad):+7.1f}'
        return (
            f"{'E-STOP' if self.estop else 'ready '} | "
            f"deadman {'ON ' if self.deadman_pressed() else 'off'} | "
            f"j1 {deg(self.positions['j1_joint'])}  "
            f"j2 {deg(self.positions['j2_joint'])}  "
            f"j3 {deg(self.positions['j3_joint'])}  "
            f"pitch {deg(self.positions['ee_base_joint_x'])}  "
            f"roll {deg(self.positions['ee_base_joint_y'])}  "
            f"grip {self.positions['slider_joint'] * 1000.0:5.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = ArmJoyTeleop()
    print(HELP_TEXT)
    print(HELP_NO_JOY)
    last_status = ''
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            status = node.status_line()
            if status != last_status:
                print('\r' + status + '   ', end='', flush=True)
                last_status = status
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
