#!/usr/bin/env python3
"""Manual keyboard teleop for the rover arm (mock hardware).

Reads single key presses from the terminal and publishes JointTrajectory
messages to the running trajectory controllers:

  /arm_controller/joint_trajectory     (j1, j2, j3, wrist pitch, wrist roll)
  /gripper_controller/joint_trajectory (slider)

Run it in a real terminal (ros2 run or xterm) — it needs keyboard input:

  ros2 run rover_arm_description arm_keyboard_teleop.py
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    'j1_joint',
    'j2_joint',
    'j3_joint',
    'ee_base_joint_x',
    'ee_base_joint_y',
]
GRIPPER_JOINTS = ['slider_joint']

# (lower, upper) in rad (slider in m); None -> continuous joint, unclamped
JOINT_LIMITS = {
    'j1_joint': None,
    'j2_joint': (-math.pi / 2.0, math.pi / 2.0),
    'j3_joint': (-math.pi / 2.0, math.pi / 2.0),
    'ee_base_joint_x': (0.0, math.pi),
    'ee_base_joint_y': None,
    'slider_joint': (0.0, 0.05),
}

SLIDER_CLOSED = 0.05  # m, fingers fully closed
SLIDER_OPEN = 0.0     # m, fingers fully open

KEY_BINDINGS = {
    'q': ('j1_joint', +1),
    'a': ('j1_joint', -1),
    'w': ('j2_joint', +1),
    's': ('j2_joint', -1),
    'e': ('j3_joint', +1),
    'd': ('j3_joint', -1),
    'r': ('ee_base_joint_x', +1),
    'f': ('ee_base_joint_x', -1),
    't': ('ee_base_joint_y', +1),
    'g': ('ee_base_joint_y', -1),
    'y': ('slider_joint', +1),
    'h': ('slider_joint', -1),
}

HELP_TEXT = """
------------------------------------------------------------
 Rover Arm - Manual Keyboard Control
------------------------------------------------------------
 joint                        increase  decrease
 j1_joint   (base yaw)           q         a
 j2_joint   (shoulder)           w         s
 j3_joint   (elbow)              e         d
 ee_base_joint_x (wrist pitch)   r         f
 ee_base_joint_y (wrist roll)    t         g
 slider_joint (gripper close/
              open)              y         h

 o          open gripper fully
 c          close gripper fully
 0          home position (all joints to 0)
 + / =      double the step size
 - / _      halve the step size
 ?          show this help again
 CTRL-C     quit
------------------------------------------------------------
"""


def get_key(settings):
    """Return a pressed key, '' for a consumed escape sequence, None on EOF."""
    if settings is not None:
        tty.setcbreak(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == '':
        return None  # stdin closed (e.g. launched without a terminal)
    if ch == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
        # drain arrow/function key escape sequences
        sys.stdin.read(1)
        if select.select([sys.stdin], [], [], 0.01)[0]:
            sys.stdin.read(1)
        return ''
    return ch


class ArmKeyboardTeleop(Node):

    def __init__(self):
        super().__init__('arm_keyboard_teleop')

        self.declare_parameter('step', 0.05)        # rad per key press
        self.declare_parameter('slider_step', 0.005)  # m per key press
        self.declare_parameter('duration', 0.3)     # trajectory duration s

        self.positions = {name: 0.0 for name in ARM_JOINTS + GRIPPER_JOINTS}
        self.received_states = False
        self.step_scale = 1.0

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

        # spin in the background so positions stay fresh while we block on keys
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

    def on_joint_states(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = position
                self.received_states = True

    def step_for(self, joint: str) -> float:
        base = (
            self.get_parameter('slider_step').value
            if joint == 'slider_joint'
            else self.get_parameter('step').value
        )
        return base * self.step_scale

    def clamp(self, joint: str, value: float) -> float:
        limits = JOINT_LIMITS[joint]
        if limits is None:
            return value
        return min(max(value, limits[0]), limits[1])

    def send_arm(self):
        duration_ns = int(self.get_parameter('duration').value * 1e9)
        point = JointTrajectoryPoint(
            positions=[self.positions[name] for name in ARM_JOINTS],
            time_from_start=Duration(sec=0, nanosec=duration_ns))
        traj = JointTrajectory(joint_names=ARM_JOINTS, points=[point])
        self.arm_pub.publish(traj)

    def send_gripper(self):
        duration_ns = int(self.get_parameter('duration').value * 1e9)
        point = JointTrajectoryPoint(
            positions=[self.positions['slider_joint']],
            time_from_start=Duration(sec=0, nanosec=duration_ns))
        traj = JointTrajectory(joint_names=GRIPPER_JOINTS, points=[point])
        self.gripper_pub.publish(traj)

    def move(self, joint: str, direction: int):
        target = self.clamp(
            joint, self.positions[joint] + direction * self.step_for(joint))
        clamped = (
            target == self.positions[joint]
            and JOINT_LIMITS[joint] is not None
            and direction != 0
        )
        self.positions[joint] = target
        if joint == 'slider_joint':
            self.send_gripper()
        else:
            self.send_arm()
        return clamped

    def home(self):
        for name in ARM_JOINTS + GRIPPER_JOINTS:
            self.positions[name] = 0.0
        self.send_arm()
        self.send_gripper()

    def status_line(self) -> str:
        deg = lambda rad: f'{math.degrees(rad):+7.1f}'
        return (
            f"step {math.degrees(self.step_for('j1_joint')):.1f} deg "
            f"({self.step_for('slider_joint') * 1000.0:.1f} mm) | "
            f"j1 {deg(self.positions['j1_joint'])}  "
            f"j2 {deg(self.positions['j2_joint'])}  "
            f"j3 {deg(self.positions['j3_joint'])}  "
            f"pitch {deg(self.positions['ee_base_joint_x'])}  "
            f"roll {deg(self.positions['ee_base_joint_y'])}  "
            f"grip {self.positions['slider_joint'] * 1000.0:5.1f} mm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardTeleop()

    print(HELP_TEXT)
    print('Waiting for controller(s)... publish a command once they are active.')
    print('status: ' + node.status_line())

    try:
        settings = termios.tcgetattr(sys.stdin)
    except termios.error:
        settings = None  # no terminal attached (stdin is a pipe or /dev/null)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key is None:
                print('\nstdin closed (no keyboard available). '
                      'Run me in a terminal instead:\n'
                      '  ros2 run rover_arm_description arm_keyboard_teleop.py')
                break
            if key == '':
                continue  # escape sequence, ignore
            if key == '\x03':  # CTRL-C
                break
            elif key in KEY_BINDINGS:
                joint, direction = KEY_BINDINGS[key]
                clamped = node.move(joint, direction)
                if clamped:
                    print(f'  [{joint}] joint limit reached')
            elif key == 'o':
                node.positions['slider_joint'] = node.clamp(
                    'slider_joint', SLIDER_OPEN)
                node.send_gripper()
            elif key == 'c':
                node.positions['slider_joint'] = node.clamp(
                    'slider_joint', SLIDER_CLOSED)
                node.send_gripper()
            elif key == '0':
                node.home()
            elif key in ('+', '='):
                node.step_scale = min(node.step_scale * 2.0, 16.0)
            elif key in ('-', '_'):
                node.step_scale = max(node.step_scale / 2.0, 1.0 / 16.0)
            elif key == '?':
                print(HELP_TEXT)
            print('\rstatus: ' + node.status_line() + '   ')
    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
