#!/usr/bin/env python3
"""Bridge slider_joint -> finger mimic for mock hardware.

GenericSystem mock does not propagate <mimic> tags, so fingers stay at 0.
This node subscribes to /joint_states_raw (broadcaster) and republishes to
/joint_states with fingers computed as slider * (pi/2 / 0.05).
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SLIDER = "slider_joint"
FINGERS_DEFAULT = ["finger1_joint", "finger2_joint", "finger3_joint"]
MULT = (math.pi / 2.0) / 0.05  # 31.4159
FINGER_LIMIT = (0.0, math.pi / 2.0)

class MimicBridge(Node):
    def __init__(self):
        super().__init__("gripper_mimic_bridge")
        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("fingers", FINGERS_DEFAULT)
        self.declare_parameter("num_fingers", 0)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        # Launch arg support: fingers as list, or num_fingers truncates default
        try:
            fingers_param = self.get_parameter("fingers").value
            if not isinstance(fingers_param, list) or not fingers_param:
                fingers_param = FINGERS_DEFAULT
        except Exception:
            fingers_param = FINGERS_DEFAULT
        try:
            num_fingers = int(self.get_parameter("num_fingers").value)
        except Exception:
            num_fingers = 0
        if num_fingers > 0:
            self.fingers = FINGERS_DEFAULT[: max(0, min(num_fingers, len(FINGERS_DEFAULT)))]
        else:
            # filter to known fingers to avoid typos
            self.fingers = [f for f in fingers_param if f in FINGERS_DEFAULT]
            if not self.fingers:
                self.fingers = FINGERS_DEFAULT
        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.sub = self.create_subscription(JointState, in_topic, self.cb, 10)
        self.get_logger().info(f"bridging {in_topic} -> {out_topic} fingers={self.fingers} mimic x{MULT:.2f}")

    def cb(self, msg: JointState):
        # find slider position
        try:
            idx = msg.name.index(SLIDER)
            slider = msg.position[idx]
        except ValueError:
            # no slider, forward as-is
            self.pub.publish(msg)
            return
        finger_val = max(FINGER_LIMIT[0], min(FINGER_LIMIT[1], slider * MULT))
        # build new message with fingers added/updated
        out = JointState()
        out.header = msg.header
        # copy existing names/positions and update/add fingers
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        name_to_eff = dict(zip(msg.name, msg.effort)) if msg.effort else {}
        for f in self.fingers:
            name_to_pos[f] = finger_val
            name_to_vel[f] = 0.0
            name_to_eff[f] = 0.0
        # remove stale fingers (e.g., 3-finger state when running 2-finger variant)
        for stale in FINGERS_DEFAULT:
            if stale not in self.fingers and stale in name_to_pos:
                # keep if original msg already had it? For 2-finger we drop extra finger
                if stale not in msg.name or stale in self.fingers:
                    continue
                # only drop if we are in 2-finger mode and stale is finger3
                if len(self.fingers) == 2 and stale == "finger3_joint":
                    del name_to_pos[stale]
                    name_to_vel.pop(stale, None)
                    name_to_eff.pop(stale, None)
        # ensure slider still present
        out.name = list(name_to_pos.keys())
        out.position = [float(name_to_pos[n]) for n in out.name]
        out.velocity = [float(name_to_vel.get(n, 0.0)) for n in out.name]
        out.effort = [float(name_to_eff.get(n, 0.0)) for n in out.name]
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = MimicBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
