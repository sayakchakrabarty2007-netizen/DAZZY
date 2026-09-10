#!/usr/bin/env python3
"""
Walk Forward Node — Trot gait with IMU pitch balancing
======================================================

Generates a trot gait for the Mini Robot Dog (DAZZY) while using IMU
feedback to compensate for forward/backward pitch during locomotion.

Trot Pattern:
  Phase A (0–50%):  FL + RR swing forward,  FR + RL are stance
  Phase B (50–100%): FR + RL swing forward,  FL + RR are stance

Startup Sequence:
  1. SETTLE  — hold neutral (0.0 rad) for 2 seconds
  2. RAMP-UP — linearly increase gait amplitude over 2 seconds
  3. WALK    — full amplitude trot with pitch correction

Joint Order (matches controllers.yaml):
  [0] Revolute 13 — Front-Left   (axis +Z)
  [1] Revolute 14 — Rear-Left    (axis +Z)
  [2] Revolute 16 — Front-Right  (axis -Z, mirrored)
  [3] Revolute 32 — Rear-Right   (axis -Z, mirrored)

Topics:
  Subscribes:  /imu/data_raw  (sensor_msgs/Imu)
  Publishes:   /joint_position_controller/commands  (Float64MultiArray)

Usage:
  ros2 run Mini_Dog_URDF_description walk_forward_node
  ros2 run Mini_Dog_URDF_description walk_forward_node --ros-args \
      -p stride_amplitude:=0.3 -p gait_frequency:=1.2
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu


# ---------------------------------------------------------------------------
# Quaternion → Euler utility
# ---------------------------------------------------------------------------
def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw).

    Uses the ZYX convention.  Input is a geometry_msgs/Quaternion with
    fields x, y, z, w.

    Returns:
        (roll, pitch, yaw) in radians.
    """
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class WalkForwardNode(Node):

    # Joint limits for PTK-7465 (±45°)
    JOINT_LIMIT = 0.785398

    # Startup timing (seconds) - Reduced to prevent long waits if simulation is slow
    SETTLE_DURATION = 0.2     # hold neutral position
    RAMP_DURATION = 0.5       # linearly ramp up amplitude

    def __init__(self):
        super().__init__('walk_forward_node')

        # ---- Declare tunable parameters ----
        # ULTRA-LOW AMPLITUDE SHUFFLE: Prevents rigid legs from pole-vaulting!
        self.declare_parameter('stride_amplitude', 0.15)   # Tiny steps to prevent vertical bouncing
        self.declare_parameter('gait_frequency', 2.0)      # Fast shuffling (2 Hz)
        self.declare_parameter('duty_factor', 0.65)        # 65% time on ground, 35% time in air
        
        # Long-Term Posture Drift Correction
        self.declare_parameter('kp_pitch', 0.3)            # Gentle correction gain
        
        self.declare_parameter('control_rate', 50.0)       # Hz

        self.stride_amplitude = self.get_parameter('stride_amplitude').value
        self.gait_frequency = self.get_parameter('gait_frequency').value
        self.duty_factor = self.get_parameter('duty_factor').value
        self.kp_pitch = self.get_parameter('kp_pitch').value
        
        self.control_rate = self.get_parameter('control_rate').value

        # ---- Internal state ----
        self.tick_count = 0
        self.filtered_pitch = 0.0
        self.imu_received = False

        # ---- ROS interfaces ----
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self._imu_cb, 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands', 10)

        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_loop)

        # Periodic status log (every 3 s)
        self.log_timer = self.create_timer(3.0, self._log_status)

        self.get_logger().info(
            f'Walk Forward Node started [amp={self.stride_amplitude:.2f}, '
            f'freq={self.gait_frequency}Hz, kp_pitch={self.kp_pitch}]')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        """Process incoming IMU data with Low-Pass Filter."""
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        # Low-pass filter to smooth out individual step vibrations.
        # We only care about long-term drift (falling backward).
        if not self.imu_received:
            self.filtered_pitch = pitch
            self.imu_received = True
        else:
            self.filtered_pitch = 0.95 * self.filtered_pitch + 0.05 * pitch
    def _clamp(self, val):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, val))

    def _get_leg_angle(self, local_phase, amplitude):
        """
        Calculates the PHYSICAL angle for a single leg based on its local phase (0.0 to 1.0).
        - Swing phase (fast): move from +A (back) to -A (front)
        - Stance phase (slow): move from -A (front) to +A (back)
        """
        swing_time = 1.0 - self.duty_factor
        
        if local_phase < swing_time:
            # FAST Phase (Swing / Recovery)
            p = local_phase / swing_time
            smooth_prog = (1.0 - math.cos(math.pi * p)) / 2.0
            return amplitude - (2.0 * amplitude) * smooth_prog
        else:
            # SLOW Phase (Stance / Propulsion)
            p = (local_phase - swing_time) / self.duty_factor
            return -amplitude + (2.0 * amplitude) * p

    def _control_loop(self):
        """Generate open-loop duty-factor trot gait with STATIC pitch balancing."""
        self.tick_count += 1
        dt = 1.0 / self.control_rate
        elapsed = self.tick_count * dt

        # ---- Determine amplitude based on startup phase ----
        if elapsed < self.SETTLE_DURATION:
            amplitude = 0.0
        elif elapsed < self.SETTLE_DURATION + self.RAMP_DURATION:
            ramp_progress = (elapsed - self.SETTLE_DURATION) / self.RAMP_DURATION
            amplitude = self.stride_amplitude * ramp_progress
        else:
            amplitude = self.stride_amplitude

        # ---- Duty-Factor Trot Gait Trajectory ----
        walk_elapsed = max(0.0, elapsed - self.SETTLE_DURATION)
        
        t_cycle = walk_elapsed % (1.0 / self.gait_frequency)
        cycle_fraction = t_cycle * self.gait_frequency
        
        math_pair1 = self._get_leg_angle(cycle_fraction, amplitude)
        math_pair2 = self._get_leg_angle((cycle_fraction + 0.5) % 1.0, amplitude)

        # ---- Low-Pass Filtered Pitch Balancing ----
        # If nose is UP (pitch < 0), we need to shift CoM FORWARD.
        # To shift CoM FORWARD, we shift feet BACKWARD (positive bias).
        # So: bias = -kp * pitch
        bias = -self.kp_pitch * self.filtered_pitch
        
        # Limit the bias so it doesn't cause pole-vaulting
        bias = max(-0.15, min(0.15, bias))

        # ---- Blend & Publish ----
        target_pair1 = math_pair1 + bias
        target_pair2 = math_pair2 + bias
        
        # Map physical targets to URDF joint commands.
        # Left axes are +1. Right axes are -1.
        fl = target_pair1 * 1.0
        rl = target_pair2 * 1.0
        
        fr = target_pair2 * -1.0
        rr = target_pair1 * -1.0

        # ---- Clamp & publish ----
        fl = self._clamp(fl)
        rl = self._clamp(rl)
        fr = self._clamp(fr)
        rr = self._clamp(rr)

        cmd = Float64MultiArray()
        cmd.data = [fl, rl, fr, rr]
        self.cmd_pub.publish(cmd)

    def _log_status(self):
        """Periodically log gait state for debugging."""
        dt = 1.0 / self.control_rate
        elapsed = self.tick_count * dt

        if elapsed < self.SETTLE_DURATION:
            phase_name = 'SETTLE'
        elif elapsed < self.SETTLE_DURATION + self.RAMP_DURATION:
            phase_name = 'RAMP-UP'
        else:
            phase_name = 'WALKING'

        pitch_deg = math.degrees(self.filtered_pitch)
        self.get_logger().info(f'[{phase_name}] t={elapsed:.1f}s | Filtered Pitch: {pitch_deg:+.1f}°')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _clamp(self, value):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, value))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WalkForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Walk Forward Node...')
    finally:
        # Return to neutral on shutdown
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
