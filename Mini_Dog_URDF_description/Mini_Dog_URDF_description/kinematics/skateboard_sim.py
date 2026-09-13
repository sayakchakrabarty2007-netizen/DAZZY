#!/usr/bin/env python3
"""
Skateboard Sim Node — Asymmetric Velocity Gait for Gazebo
=========================================================

Exploits the `damping="0.05"` on the wheel joints in Gazebo.
Because Gazebo continuous joints lack true one-way ratchet physics, 
symmetric movements cancel out perfectly. 
This node uses a VERY SLOW forward reach (low damping resistance = wheels roll)
and an EXPLOSIVE backward push (high damping resistance = wheels lock).

Driving Force Mechanism (Synchronous Scooter Kick & Glide):
  Instead of alternating diagonal pairs (which cause body twist & slipping),
  ALL 4 LEGS MOVE SYNCHRONOUSLY TOGETHER:

  1. REACH PHASE (Forward Swing):
     All 4 legs swing forward simultaneously to -reach_amplitude.
     The one-way ratchet wheels roll forward on the ground without pulling
     the body backward.

  2. PUSH PHASE (Power Thrust):
     All 4 legs push backward simultaneously from -reach_amplitude to +push_amplitude.
     The high-friction TPU foot tips grip the ground, imparting a strong forward
     impulse to propel the body forward.

  3. SKATE / COAST PHASE (Glide):
     All 4 legs hold straight posture (coast_angle).
     The body glides forward freely on its 4 one-way ratchet wheels.

  4. IMU STILLNESS RESET TRIGGER:
     The node monitors IMU acceleration & angular velocity.
     When linear motion dies down (velocity & acceleration drop to ~0), indicating
     the dog has finished coasting and stopped, the state machine triggers the next
     reach -> push -> skate cycle!

Joint Order (matches controllers.yaml):
  [0] Revolute 13 — Front-Right Hip  (axis -Z)
  [1] Revolute 14 — Rear-Right Hip   (axis -Z)
  [2] Revolute 16 — Front-Left Hip   (axis +Z)
  [3] Revolute 32 — Rear-Left Hip    (axis +Z)

Topics:
  Subscribes:  /imu/data_raw  (sensor_msgs/Imu)
  Publishes:   /joint_position_controller/commands  (Float64MultiArray)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu


def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SkateboardForwardNode(Node):

    JOINT_LIMIT = 0.785398  # ±45 deg limit

    SETTLE_DURATION = 0.5   # Startup settle time in seconds

    def __init__(self):
        super().__init__('skateboard_sim_node')

        # Tunable parameters (Gazebo Asymmetric Damping Exploit)
        # Slow reach minimizes wheel damping. Explosive push maximizes wheel damping (faking a lock).
        self.declare_parameter('stroke_amplitude', 0.18)    # Large stroke for max impulse
        self.declare_parameter('front_offset', -0.15)       # Front biased FORWARD (Wide stance)
        self.declare_parameter('rear_offset', 0.15)         # Rear biased BACKWARD (Wide stance)
        self.declare_parameter('reach_duration', 1.20)      # VERY SLOW REACH (Wheels roll freely)
        self.declare_parameter('push_duration', 0.12)       # EXPLOSIVE PUSH (High velocity = High damping = Wheels lock!)
        self.declare_parameter('direction_multiplier', -1.0) # Invert direction to convert Gazebo joint-damping reaction into forward motion
        self.declare_parameter('min_coast_duration', 0.25)
        self.declare_parameter('max_coast_duration', 1.5)
        self.declare_parameter('stillness_threshold', 0.5)
        self.declare_parameter('control_rate', 50.0)

        self.stroke_amplitude = self.get_parameter('stroke_amplitude').value
        self.front_offset = self.get_parameter('front_offset').value
        self.rear_offset = self.get_parameter('rear_offset').value
        self.reach_duration = self.get_parameter('reach_duration').value
        self.push_duration = self.get_parameter('push_duration').value
        self.direction_multiplier = self.get_parameter('direction_multiplier').value
        self.min_coast_duration = self.get_parameter('min_coast_duration').value
        self.max_coast_duration = self.get_parameter('max_coast_duration').value
        self.stillness_threshold = self.get_parameter('stillness_threshold').value
        self.control_rate = self.get_parameter('control_rate').value

        self.state = 'SETTLE'
        self.state_start_time = None
        self.start_time = None

        self.motion_level = 1.0
        self.filtered_pitch = 0.0

        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.log_timer = self.create_timer(3.0, self._log_status)

        self.get_logger().info('Skateboard Sim Node initialized (Asymmetric Damping Exploit)')

    def _imu_cb(self, msg: Imu):
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        self.filtered_pitch = 0.8 * self.filtered_pitch + 0.2 * pitch
        g_proj = 9.81 * math.sin(pitch)
        ax_net = abs(msg.linear_acceleration.x - g_proj)
        wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        gyro_mag = math.sqrt(wx*wx + wy*wy + wz*wz)
        self.motion_level = 0.85 * self.motion_level + 0.15 * (ax_net + gyro_mag)

    def _clamp(self, val):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, val))

    def _quintic_smooth(self, p):
        # Quintic smoothstep: 6p^5 - 15p^4 + 10p^3
        # Guarantees zero velocity AND zero acceleration at p=0 and p=1
        return 6 * (p ** 5) - 15 * (p ** 4) + 10 * (p ** 3)

    def _control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now
            self.state_start_time = now
            self.state = 'SETTLE'

        elapsed_in_state = now - self.state_start_time
        stroke = 0.0

        # Note: NEGATIVE target = FORWARD REACH. POSITIVE target = BACKWARD PUSH.
        if self.state == 'SETTLE':
            stroke = 0.0
            if elapsed_in_state >= self.SETTLE_DURATION:
                self.state = 'REACH'
                self.state_start_time = now

        elif self.state == 'REACH':
            progress = min(1.0, elapsed_in_state / self.reach_duration)
            smooth = self._quintic_smooth(progress)
            # Swing FORWARD from +stroke_amplitude to -stroke_amplitude
            # (First cycle starts at 0, so we use start_stroke)
            start_stroke = self.stroke_amplitude if self.start_time != self.state_start_time else 0.0
            stroke = start_stroke + (-self.stroke_amplitude - start_stroke) * smooth

            if elapsed_in_state >= self.reach_duration:
                self.state = 'PUSH'
                self.state_start_time = now

        elif self.state == 'PUSH':
            progress = min(1.0, elapsed_in_state / self.push_duration)
            smooth = self._quintic_smooth(progress)
            # Swing BACKWARD from -stroke_amplitude to +stroke_amplitude
            stroke = -self.stroke_amplitude + (self.stroke_amplitude - (-self.stroke_amplitude)) * smooth

            if elapsed_in_state >= self.push_duration:
                self.state = 'SKATE'
                self.state_start_time = now

        elif self.state == 'SKATE':
            # HOLD the leg at +stroke_amplitude while the body glides forward!
            # Do NOT aggressively reset to 0, that kills the momentum.
            stroke = self.stroke_amplitude

            if elapsed_in_state >= self.min_coast_duration:
                is_stopped = (self.motion_level < self.stillness_threshold)
                is_timeout = (elapsed_in_state >= self.max_coast_duration)
                if is_stopped or is_timeout:
                    self.state = 'REACH'
                    self.state_start_time = now

        # Apply direction_multiplier to invert stroke direction for Gazebo damping physics
        effective_stroke = stroke * self.direction_multiplier

        # WIDE STANCE SQUAT:
        # Front biased FORWARD (-), Rear biased BACKWARD (+).
        # This lowers the CoM and significantly increases the support base,
        # making it physically much harder to wheelie.
        
        front_angle = effective_stroke + self.front_offset
        rear_angle = effective_stroke + self.rear_offset

        # Correct Mapping:
        # Index 0 is FL -> -1.0
        # Index 1 is RL -> -1.0
        # Index 2 is FR -> 1.0
        # Index 3 is RR -> 1.0
        cmd0_fl = front_angle * -1.0
        cmd1_rl = rear_angle * -1.0
        cmd2_fr = front_angle * 1.0
        cmd3_rr = rear_angle * 1.0

        cmd = Float64MultiArray()
        cmd.data = [
            self._clamp(cmd0_fl),
            self._clamp(cmd1_rl),
            self._clamp(cmd2_fr),
            self._clamp(cmd3_rr)
        ]
        self.cmd_pub.publish(cmd)

    def _log_status(self):
        now = self.get_clock().now().nanoseconds / 1e9
        total_t = now - (self.start_time if self.start_time else now)
        pitch_deg = math.degrees(self.filtered_pitch)
        self.get_logger().info(
            f'[{self.state}] t={total_t:.1f}s | '
            f'Motion metric: {self.motion_level:.3f} | '
            f'Filtered Pitch: {pitch_deg:+.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = SkateboardForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Skateboard Sim Node...')
    finally:
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
