#!/usr/bin/env python3
"""
Wheelie / Sit & Wave Node for Gazebo & Hardware
===============================================

1. Robot sits down by swinging rear legs forward, dropping the rear chassis.
2. Once seated, the front legs alternate in a "waving" or "begging" motion.
This avoids extreme physics impulses that might crash Gazebo.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelieSitWaveNode(Node):

    JOINT_LIMIT = 0.785398  # ±45 deg limit
    SETTLE_DURATION = 1.0   # Startup settle time in seconds
    SIT_DURATION = 1.5      # Time to smoothly sit down

    def __init__(self):
        super().__init__('wheelie_glide_node')

        self.declare_parameter('sit_rear_angle', -0.70)       # Rear legs swing forward to drop body
        self.declare_parameter('wave_base_angle', -0.20)      # Front legs base angle when waving
        self.declare_parameter('wave_amplitude', 0.40)        # Amplitude of the wave
        self.declare_parameter('wave_frequency', 1.5)         # Waving speed in Hz
        self.declare_parameter('control_rate', 50.0)

        self.sit_rear_angle = self.get_parameter('sit_rear_angle').value
        self.wave_base_angle = self.get_parameter('wave_base_angle').value
        self.wave_amplitude = self.get_parameter('wave_amplitude').value
        self.wave_frequency = self.get_parameter('wave_frequency').value
        self.control_rate = self.get_parameter('control_rate').value

        self.state = 'SETTLE'
        self.state_start_time = None
        self.start_time = None

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        
        self.get_logger().info('Wheelie Sit & Wave Node initialized')

    def _clamp(self, val):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, val))

    def _quintic_smooth(self, p):
        p = max(0.0, min(1.0, p))
        return 6 * (p ** 5) - 15 * (p ** 4) + 10 * (p ** 3)

    def _control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now
            self.state_start_time = now
            self.state = 'SETTLE'

        elapsed_in_state = now - self.state_start_time

        front_left_angle = 0.0
        front_right_angle = 0.0
        rear_angle = 0.0

        if self.state == 'SETTLE':
            if elapsed_in_state >= self.SETTLE_DURATION:
                self.state = 'SIT'
                self.state_start_time = now

        elif self.state == 'SIT':
            progress = elapsed_in_state / self.SIT_DURATION
            smooth = self._quintic_smooth(progress)
            
            # Rear legs swing forward to drop the rear
            rear_angle = 0.0 + (self.sit_rear_angle - 0.0) * smooth
            
            # Front legs raise up to the wave base angle
            front_left_angle = 0.0 + (self.wave_base_angle - 0.0) * smooth
            front_right_angle = 0.0 + (self.wave_base_angle - 0.0) * smooth

            if elapsed_in_state >= self.SIT_DURATION:
                self.state = 'WAVE'
                self.state_start_time = now

        elif self.state == 'WAVE':
            rear_angle = self.sit_rear_angle
            
            # Alternate waving using sine waves out of phase by Pi
            phase = elapsed_in_state * self.wave_frequency * 2.0 * math.pi
            
            front_left_angle = self.wave_base_angle + self.wave_amplitude * math.sin(phase)
            front_right_angle = self.wave_base_angle + self.wave_amplitude * math.sin(phase + math.pi)

        # Correct Hardware Mapping:
        # Index 0 is FL -> -1.0
        # Index 1 is RL -> -1.0
        # Index 2 is FR -> 1.0
        # Index 3 is RR -> 1.0
        cmd0_fl = front_left_angle * -1.0
        cmd1_rl = rear_angle * -1.0
        cmd2_fr = front_right_angle * 1.0
        cmd3_rr = rear_angle * 1.0

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
    node = WheelieSitWaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Wheelie Node...')
    finally:
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
