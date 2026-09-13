#!/usr/bin/env python3
"""
Crawl Gait Node — 4-Beat Creep Gait with Pseudo-Waddle
=======================================================

A cute, slow walking gait where one leg swings forward at a time, 
while the other three support the body and push backward.
The continuous variation of leg angles (which changes their effective 
vertical heights) naturally induces a "pseudo-waddle" sway.

Sequence: Front-Left -> Rear-Right -> Front-Right -> Rear-Left
(Phase offsets: 0.0, 0.25, 0.50, 0.75)

Note on Multipliers (Empirically Verified):
- Index 0 (FL): -1.0
- Index 1 (RL): -1.0
- Index 2 (FR):  1.0
- Index 3 (RR):  1.0
(Negative target = FORWARD, Positive target = BACKWARD)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CrawlGaitNode(Node):

    JOINT_LIMIT = 0.785398

    def __init__(self):
        super().__init__('crawl_gait_node')

        # Tunable parameters
        self.declare_parameter('cycle_duration', 3.0)       # Time for one full 4-beat cycle (s)
        self.declare_parameter('reach_angle', -0.20)        # Forward reach angle (Negative)
        self.declare_parameter('push_angle', 0.10)          # Backward push angle (Positive)
        self.declare_parameter('yaw_compensation', 0.08)    # Steer left to fix rightward drift
        self.declare_parameter('control_rate', 50.0)

        self.cycle_duration = self.get_parameter('cycle_duration').value
        self.reach_angle = self.get_parameter('reach_angle').value
        self.push_angle = self.get_parameter('push_angle').value
        self.yaw_compensation = self.get_parameter('yaw_compensation').value
        self.control_rate = self.get_parameter('control_rate').value

        self.start_time = None

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        
        self.get_logger().info(f'Crawl Gait Node initialized (Yaw Comp: {self.get_parameter("yaw_compensation").value})')

    def _clamp(self, val):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, val))

    def _get_leg_angle(self, cycle_phase):
        """
        cycle_phase: 0.0 to 1.0
        0.0 to 0.25: Swing phase (move from push_angle to reach_angle)
        0.25 to 1.0: Push phase (move from reach_angle to push_angle)
        """
        if cycle_phase < 0.25:
            # Swing (25% of cycle)
            progress = cycle_phase / 0.25
            smooth = (1.0 - math.cos(math.pi * progress)) / 2.0
            return self.push_angle + (self.reach_angle - self.push_angle) * smooth
        else:
            # Push (75% of cycle)
            progress = (cycle_phase - 0.25) / 0.75
            smooth = (1.0 - math.cos(math.pi * progress)) / 2.0
            return self.reach_angle + (self.push_angle - self.reach_angle) * smooth

    def _control_loop(self):
        # Fetch parameters dynamically so ros2 param set works on the fly
        cycle_duration = self.get_parameter('cycle_duration').value
        yaw_comp = self.get_parameter('yaw_compensation').value

        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now

        # Settle for 1 second before moving
        if now - self.start_time < 1.0:
            target_fl = 0.0
            target_rl = 0.0
            target_fr = 0.0
            target_rr = 0.0
        else:
            elapsed = (now - self.start_time) - 1.0
            master_phase = (elapsed % cycle_duration) / cycle_duration

            # 4-Beat Sequence: FL (0.0) -> RR (0.25) -> FR (0.50) -> RL (0.75)
            phase_fl = (master_phase - 0.0) % 1.0
            phase_rr = (master_phase - 0.25) % 1.0
            phase_fr = (master_phase - 0.50) % 1.0
            phase_rl = (master_phase - 0.75) % 1.0

            target_fl = self._get_leg_angle(phase_fl) * (1.0 - yaw_comp)
            target_rl = self._get_leg_angle(phase_rl) * (1.0 - yaw_comp)
            target_fr = self._get_leg_angle(phase_fr) * (1.0 + yaw_comp)
            target_rr = self._get_leg_angle(phase_rr) * (1.0 + yaw_comp)

        # Apply Empirically Verified Hardware Multipliers
        # Left Legs (Index 0, 1) -> -1.0
        # Right Legs (Index 2, 3) -> 1.0
        cmd0_fl = target_fl * -1.0
        cmd1_rl = target_rl * -1.0
        cmd2_fr = target_fr * 1.0
        cmd3_rr = target_rr * 1.0

        cmd = Float64MultiArray()
        cmd.data = [
            self._clamp(cmd0_fl),
            self._clamp(cmd1_rl),
            self._clamp(cmd2_fr),
            self._clamp(cmd3_rr)
        ]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CrawlGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Crawl Gait Node...')
    finally:
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
