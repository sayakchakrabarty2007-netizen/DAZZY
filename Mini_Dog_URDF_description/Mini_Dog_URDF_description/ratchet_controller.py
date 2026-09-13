#!/usr/bin/env python3
"""
Ratchet Controller Node for DAZZY Mini Robot Dog
=================================================

Simulates one-way mechanical ratchets on the foot wheels.

Physical behavior:
  - Forward rotation (wheel rolls forward): FREE — zero braking effort
  - Backward rotation (wheel tries to spin backward): LOCKED — large braking
    effort applied to prevent reverse rotation

*DISCLAIMER on Simulation Fidelity*:
This is a software approximation of a mechanical one-way roller clutch/pawl.
A real one-way clutch locks mechanically and instantaneously. Because this is
emulated via a software node subscribing to joint states and publishing
efforts, there is unavoidable sensing and control-loop latency. To minimize
this, the control rate should be set very high (e.g., 500+ Hz).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RatchetControllerNode(Node):
    """Simulates one-way ratchet locking on foot wheel joints."""

    # Joint names for the 4 ratchet wheels (must match controllers.yaml order)
    RATCHET_JOINTS = [
        'Revolute 82',   # FL foot wheel
        'Revolute 86',   # FR foot wheel
        'Revolute 87',   # BR foot wheel
        'Revolute 88',   # BL foot wheel
    ]

    def __init__(self):
        super().__init__('ratchet_controller_node')

        # ---- Parameters ----
        self.declare_parameter('braking_effort', 0.5)      # N·m braking torque
        self.declare_parameter('velocity_threshold', 0.01)  # rad/s deadzone
        self.declare_parameter('control_rate', 500.0)       # Hz, high frequency to minimize latency


        self.braking_effort = self.get_parameter('braking_effort').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.control_rate = self.get_parameter('control_rate').value

        # Per-joint locking direction:
        #   +1 means lock backward (negative velocity → brake)
        #   -1 means lock forward (positive velocity → brake)
        # These depend on the joint axis direction in the URDF.
        # Revolute 82: axis z=-1, so positive velocity = wheel spins one way
        # We want the wheel to ONLY roll forward (propelling the robot).
        # Forward motion of robot → bottom of wheel moves backward →
        # wheel spins in the "forward" direction.
        # Revolute 82 (FL): axis z=-1, forward=positive, backward=negative (lock on negative -> +1.0)
        # Revolute 86 (FR): axis z=1, forward=negative, backward=positive (lock on positive -> -1.0)
        # Revolute 87 (RR): axis z=1, forward=negative, backward=positive (lock on positive -> -1.0)
        # Revolute 88 (RL): axis z=1, forward=negative, backward=positive (lock on positive -> -1.0)
        self.lock_signs = [1.0, -1.0, -1.0, -1.0]

        # ---- State ----
        self.joint_velocities = {name: 0.0 for name in self.RATCHET_JOINTS}

        # ---- ROS interfaces ----
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/ratchet_effort_controller/commands', 10)

        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_loop)

        # Periodic log
        self.log_timer = self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f'Ratchet Controller started [brake={self.braking_effort} N·m, '
            f'deadzone={self.velocity_threshold} rad/s]')

    def _joint_state_cb(self, msg: JointState):
        """Cache the latest velocities for the ratchet joints."""
        for i, name in enumerate(msg.name):
            if name in self.joint_velocities:
                if i < len(msg.velocity):
                    self.joint_velocities[name] = msg.velocity[i]

    def _control_loop(self):
        """Apply braking effort when a wheel tries to spin backward."""
        efforts = []
        for i, joint_name in enumerate(self.RATCHET_JOINTS):
            vel = self.joint_velocities[joint_name]
            sign = self.lock_signs[i]

            # If velocity is in the "locked" direction, apply braking
            if vel * sign < -self.velocity_threshold:
                # Apply effort opposing the backward velocity
                brake = self.braking_effort * sign
                efforts.append(brake)
            else:
                # Free rolling — no effort applied
                efforts.append(0.0)

        cmd = Float64MultiArray()
        cmd.data = efforts
        self.effort_pub.publish(cmd)

    def _log_status(self):
        """Periodically log ratchet wheel velocities."""
        vels = [f'{self.joint_velocities[j]:+.2f}' for j in self.RATCHET_JOINTS]
        self.get_logger().info(f'Wheel velocities (rad/s): [{", ".join(vels)}]')


def main(args=None):
    rclpy.init(args=args)
    node = RatchetControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
