#!/usr/bin/env python3
"""
Walk Forward Node — Trot gait with IMU pitch balancing
======================================================

Generates a trot gait for the Mini Robot Dog (DAZZY) while using IMU
feedback to compensate for forward/backward pitch during locomotion.

Foot/Ratchet Interaction During Walking:
  During normal walking (swing + stance phases), the leg presses DOWN
  during stance. This means the foot tip (TPU pad) is the primary
  ground contact. The ratchet wheel, mounted slightly behind/above
  the foot tip on the bearing holder, lifts off the ground when the
  leg pushes down. Therefore, during normal walking, the ratchet
  wheels do NOT roll — the robot walks purely on its foot tips.

  This is correct real-world behavior: the ratchet wheels only engage
  when the body leans forward enough to transfer weight off the foot
  tips and onto the wheels (see skateboard_forward.py for that mode).

Trot Pattern:
  Phase A (0–50%):  FL + RR swing forward,  FR + RL are stance
  Phase B (50–100%): FR + RL swing forward,  FL + RR are stance

Startup Sequence:
  1. SETTLE  — hold neutral (0.0 rad) for 0.2 seconds
  2. RAMP-UP — linearly increase gait amplitude over 0.5 seconds
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
  ros2 run Mini_Dog_URDF_description walk_forward_node --ros-args \\
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
    SETTLE_DURATION = 0.2     # wait before walking
    RAMP_DURATION = 3.0       # linearly ramp up amplitude

    def __init__(self):
        super().__init__('walk_forward_node')

        # ---- Declare tunable parameters ----
        # Moderate amplitude: enough for a real stride but not so much that
        # the foot tips lose ground contact and the ratchet wheels engage.
        # If amplitude is too large, the legs tilt so far forward that the
        # body transitions from foot-tip contact to ratchet-wheel contact,
        # which is the skateboard mode, NOT walking.
        self.declare_parameter('stride_amplitude', 0.15)
        self.declare_parameter('gait_frequency', 2.0)      # 2 Hz trot
        self.declare_parameter('duty_factor', 0.65)         # 65% stance, 35% swing

        # Stance-phase downward bias: during the stance phase, each leg
        # gets a small additional positive angle to push the foot tip
        # firmly into the ground. This ensures the ratchet wheel stays
        # lifted off the ground during normal walking.
        self.declare_parameter('stance_down_bias', 0.03)    # ~1.7° extra push-down

        # Long-Term Posture Drift Correction
        self.declare_parameter('kp_pitch', 0.3)

        self.declare_parameter('control_rate', 50.0)        # Hz

        self.stride_amplitude = self.get_parameter('stride_amplitude').value
        self.gait_frequency = self.get_parameter('gait_frequency').value
        self.duty_factor = self.get_parameter('duty_factor').value
        self.stance_down_bias = self.get_parameter('stance_down_bias').value
        self.kp_pitch = self.get_parameter('kp_pitch').value

        self.control_rate = self.get_parameter('control_rate').value

        # ---- Internal state ----
        self.start_time = None
        self.filtered_pitch = 0.0
        self.filtered_pitch_bias = 0.0
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
            f'freq={self.gait_frequency}Hz, kp_pitch={self.kp_pitch}, '
            f'stance_down_bias={self.stance_down_bias:.3f}]')

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
        Calculates the PHYSICAL angle for a single leg based on its
        local phase (0.0 to 1.0).

        The gait is divided into:
          - Swing phase (fast):  move from +A (back) to -A (front)
            → leg in the air, ratchet naturally disengaged
          - Stance phase (slow): move from -A (front) to +A (back)
            → leg pressing DOWN on foot tip, ratchet lifted off ground

        Returns:
            (angle, is_stance) tuple. is_stance is True when the leg
            is in the ground-contact stance phase.
        """
        swing_time = 1.0 - self.duty_factor

        if local_phase < swing_time:
            # FAST Phase (Swing / Recovery) — leg in air
            p = local_phase / swing_time
            smooth_prog = (1.0 - math.cos(math.pi * p)) / 2.0
            angle = amplitude - (2.0 * amplitude) * smooth_prog
            return angle, False  # Not in stance
        else:
            # SLOW Phase (Stance / Propulsion) — leg on ground
            p = (local_phase - swing_time) / self.duty_factor
            angle = -amplitude + (2.0 * amplitude) * p
            return angle, True   # In stance

    def _control_loop(self):
        """Generate open-loop duty-factor trot gait with STATIC pitch
        balancing and stance-phase ratchet suppression."""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

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

        # Diagonal pair 1: FL + RR
        angle_pair1, stance_pair1 = self._get_leg_angle(cycle_fraction, amplitude)
        # Diagonal pair 2: FR + RL (offset by 0.5 = 180°)
        angle_pair2, stance_pair2 = self._get_leg_angle(
            (cycle_fraction + 0.5) % 1.0, amplitude)

        # ---- Stance-phase downward bias ----
        # During stance, apply a small extra positive angle to push the
        # foot tip harder into the ground. This ensures the ratchet wheel
        # (which is mounted slightly behind and higher than the foot tip)
        # stays lifted clear of the ground during normal walking.
        # During swing, no bias is applied (leg is in the air anyway).
        down_bias_pair1 = self.stance_down_bias if stance_pair1 else 0.0
        down_bias_pair2 = self.stance_down_bias if stance_pair2 else 0.0

        # ---- Low-Pass Filtered Pitch Stabilization ----
        # If nose is DOWN (pitch > 0), we need to shift CoM BACKWARD.
        # To shift CoM BACKWARD, we shift feet FORWARD.
        # A POSITIVE target bias swings all legs FORWARD.
        # So: bias = +kp * pitch
        raw_pitch_bias = self.kp_pitch * self.filtered_pitch
        raw_pitch_bias = max(-0.15, min(0.15, raw_pitch_bias))

        # Ramp up pitch bias to prevent violent kicking on spawn
        if elapsed < self.SETTLE_DURATION:
            pitch_bias_factor = 0.0
        elif elapsed < self.SETTLE_DURATION + self.RAMP_DURATION:
            pitch_bias_factor = (elapsed - self.SETTLE_DURATION) / self.RAMP_DURATION
        else:
            pitch_bias_factor = 1.0

        target_pitch_bias = raw_pitch_bias * pitch_bias_factor

        # FORCIBLY DISABLE PITCH BIAS. 
        # Swinging legs forward to catch a forward fall rolls the wheels freely,
        # which shifts the support base too far forward and causes a backflip!
        pitch_bias = 0.0

        # ---- Blend & Publish ----
        target_pair1 = angle_pair1 + pitch_bias + down_bias_pair1
        target_pair2 = angle_pair2 + pitch_bias + down_bias_pair2

        # Map physical targets to URDF joint commands based on controllers.yaml order:
        # [0] Revolute 13 (Front-Right) -> Pair 2, Axis -Z
        # [1] Revolute 14 (Rear-Right)  -> Pair 1, Axis -Z
        # [2] Revolute 16 (Front-Left)  -> Pair 1, Axis +Z
        # [3] Revolute 32 (Rear-Left)   -> Pair 2, Axis +Z
        
        cmd0_fr = target_pair2 * -1.0
        cmd1_rr = target_pair1 * -1.0
        cmd2_fl = target_pair1 * 1.0
        cmd3_rl = target_pair2 * 1.0

        # ---- Clamp & publish ----
        cmd = Float64MultiArray()
        cmd.data = [
            self._clamp(cmd0_fr),
            self._clamp(cmd1_rr),
            self._clamp(cmd2_fl),
            self._clamp(cmd3_rl)
        ]
        self.cmd_pub.publish(cmd)

    def _log_status(self):
        """Periodically log gait state for debugging."""
        if self.start_time is None:
            return
        
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        if elapsed < self.SETTLE_DURATION:
            phase_name = 'SETTLE'
        elif elapsed < self.SETTLE_DURATION + self.RAMP_DURATION:
            phase_name = 'RAMP-UP'
        else:
            phase_name = 'WALKING'

        pitch_deg = math.degrees(self.filtered_pitch)
        self.get_logger().info(f'[{phase_name}] t={elapsed:.1f}s | Filtered Pitch: {pitch_deg:+.1f}°')


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
