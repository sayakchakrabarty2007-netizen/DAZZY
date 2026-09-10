#!/usr/bin/env python3
"""
Stand Straight Node — IMU-stabilized standing for Mini Robot Dog (DAZZY)
========================================================================

Reads IMU quaternion → Euler angles → PD pitch correction → commands
all 4 hip servos to keep the body level.

Joint Order (matches controllers.yaml):
  [0] Revolute 13 — Front-Left   (axis +Z)
  [1] Revolute 14 — Rear-Left    (axis +Z)
  [2] Revolute 16 — Front-Right  (axis -Z, mirrored)
  [3] Revolute 32 — Rear-Right   (axis -Z, mirrored)

Topics:
  Subscribes:  /imu/data_raw  (sensor_msgs/Imu)
  Publishes:   /joint_position_controller/commands  (Float64MultiArray)

NOTE: This robot has 4 single-DOF hip joints only (no knees). Roll
correction is NOT possible with sagittal-plane hip joints alone — only
pitch (forward/backward tilt) can be corrected.

Usage:
  ros2 run Mini_Dog_URDF_description stand_straight_node
  ros2 run Mini_Dog_URDF_description stand_straight_node --ros-args \
      -p kp_pitch:=1.0 -p kd_pitch:=0.08
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
    # Roll (rotation about X)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    # Yaw (rotation about Z)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class StandStraightNode(Node):

    # Joint limits for PTK-7465 (±45°)
    JOINT_LIMIT = 0.785398

    def __init__(self):
        super().__init__('stand_straight_node')

        # ---- Declare tunable parameters ----
        self.declare_parameter('kp_pitch', 1.5)        # Increased gain to fight rear sag
        self.declare_parameter('kd_pitch', 0.1)
        self.declare_parameter('control_rate', 50.0)   # Hz

        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.kd_pitch = self.get_parameter('kd_pitch').value
        control_rate = self.get_parameter('control_rate').value

        # ---- Internal state ----
        self.ref_pitch = 0.0           # Force exact level target (no sag capturing)
        self.current_pitch = 0.0       # pitch deviation from reference
        self.prev_pitch_error = 0.0    # for derivative term
        self.imu_received = False

        # ---- ROS interfaces ----
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self._imu_cb, 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands', 10)

        self.control_timer = self.create_timer(
            1.0 / control_rate, self._control_loop)

        # Periodic status log (every 2 s)
        self.log_timer = self.create_timer(2.0, self._log_status)

        self.get_logger().info(
            f'Stand Straight Node started  '
            f'[Kp={self.kp_pitch}, Kd={self.kd_pitch}, '
            f'rate={control_rate:.0f} Hz]')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        """Process incoming IMU data."""
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)

        self.current_pitch = pitch - self.ref_pitch
        self.imu_received = True

    def _control_loop(self):
        """PD controller — runs at *control_rate* Hz."""
        if not self.imu_received:
            return

        # --- Pitch PD ---
        pitch_error = 0.0 - self.current_pitch
        pitch_derivative = pitch_error - self.prev_pitch_error
        self.prev_pitch_error = pitch_error

        correction = (self.kp_pitch * pitch_error
                       + self.kd_pitch * pitch_derivative)

        # Pitch correction mapping:
        #   Nose-down (pitch > 0) → front legs push backward (−),
        #                           rear  legs push forward  (+)
        fl = 0.0 - correction   # Front-Left
        rl = 0.0 + correction   # Rear-Left
        fr = 0.0 - correction   # Front-Right
        rr = 0.0 + correction   # Rear-Right

        # Clamp to servo limits
        fl = self._clamp(fl)
        rl = self._clamp(rl)
        fr = self._clamp(fr)
        rr = self._clamp(rr)

        cmd = Float64MultiArray()
        cmd.data = [fl, rl, fr, rr]
        self.cmd_pub.publish(cmd)

    def _log_status(self):
        """Periodically log pitch deviation for debugging."""
        if self.imu_received:
            self.get_logger().info(
                f'Pitch deviation: {math.degrees(self.current_pitch):+.2f}°')

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
    node = StandStraightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Stand Straight Node...')
    finally:
        # Send zero command on shutdown so the robot doesn't freeze mid-correction
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
