#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class OpenLoopVisualizerNode(Node):
    """
    This node enforces open-loop control visualization.
    Since true joint states (position/velocity) are hidden to prevent the gait code
    from "cheating" (as real PTK7465 servos have no feedback), RViz would break.
    This node bridges the gap by listening to the commanded positions and publishing
    them to /joint_states so robot_state_publisher can update the TF tree.
    """
    def __init__(self):
        super().__init__('open_loop_visualizer_node')

        self.joint_names = [
            "Revolute 13", # FL
            "Revolute 14", # RL
            "Revolute 16", # FR
            "Revolute 32"  # RR
        ]

        # Subscribe to the commands sent by the gait controller
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_position_controller/commands',
            self.command_callback,
            10
        )

        # Publish mock joint states for visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info('Open-Loop Visualizer Node initialized.')

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warning(f"Expected {len(self.joint_names)} commands, got {len(msg.data)}")
            return

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = self.joint_names
        # Assume the servos instantly reach the commanded position (perfect open-loop)
        state_msg.position = list(msg.data)
        
        self.joint_state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
