#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MockHardwareInterface(Node):
    def __init__(self):
        super().__init__('mock_hardware_interface')

        # 1. Define the topics (Must match the <param> tags in your URDF)
        self.command_topic = '/hardware_level/commands'
        self.state_topic = '/hardware_level/states'

        # 2. Setup Publisher and Subscriber
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.cmd_sub = self.create_subscription(
            JointState,
            self.command_topic,
            self.command_callback,
            10
        )

        # 3. Setup the internal state loop (50 Hz is standard for RViz visualization)
        self.timer = self.create_timer(0.02, self.publish_state)

        # 4. Initialize Joint States
        # IMPORTANT: ros2_control often requires initial states before it sends commands. 
        # Replace these with your actual URDF joint names!
        self.current_state = JointState()
        self.current_state.name = ['j1_joint', 'j2_joint', 'j3_joint', 'ee_base_joint_x', 'ee_base_joint_y'] # <-- UPDATE THESE
        self.current_state.position = [0.0, 0.0, 0.0]    # Default starting positions
        self.current_state.velocity = [0.0, 0.0, 0.0]
        self.current_state.effort = [0.0, 0.0, 0.0]

        self.get_logger().info("Mock Hardware Interface started. Looping commands to states.")

    def command_callback(self, msg: JointState):
        """
        Whenever a command arrives from ros2_control, update our internal 'physical' state 
        to match exactly what was commanded (simulating perfect hardware).
        """
        self.current_state.name = msg.name
        
        # Only update fields that the controller actually sent
        if msg.position:
            self.current_state.position = msg.position
        if msg.velocity:
            self.current_state.velocity = msg.velocity
        if msg.effort:
            self.current_state.effort = msg.effort

    def publish_state(self):
        """
        Continuously publish the current state back to ros2_control.
        """
        # Always update the timestamp to the current ROS time
        self.current_state.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.current_state)

def main(args=None):
    rclpy.init(args=args)
    node = MockHardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()