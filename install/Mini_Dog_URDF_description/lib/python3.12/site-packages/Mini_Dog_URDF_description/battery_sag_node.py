#!/usr/bin/env python3
"""
Battery Voltage Sag Simulation Node for Mini Robot Dog (DAZZY)

Simulates real-time battery voltage droop under servo load for a
Tattu 300mAh 7.6V 75C 2S1P LiHV battery powering 4× PTK-7465 servos.

Electrical Model:
  1. Read |effort| from all 4 joints via /joint_states
  2. Map torque → current via PTK-7465 torque-current curve
  3. Compute voltage sag: V_drop = I_total × R_internal
  4. Publish degradation multiplier: V_actual / V_max

Topics:
  Subscribes: /joint_states (sensor_msgs/JointState)
  Publishes:  /battery_state (sensor_msgs/BatteryState)
              /servo_degradation (std_msgs/Float64)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, BatteryState
from std_msgs.msg import Float64


class BatterySagNode(Node):
    """Simulates battery voltage sag based on real-time servo torque demand."""

    # Joint names for the 4 actuated hip servos
    JOINT_NAMES = [
        'Revolute 13',   # Front-Left Hip
        'Revolute 14',   # Rear-Left Hip
        'Revolute 16',   # Front-Right Hip
        'Revolute 32',   # Rear-Right Hip
    ]

    def __init__(self):
        super().__init__('battery_sag_node')

        # ============================================================
        # Declare ROS parameters with defaults for 2S LiPo
        # ============================================================
        self.declare_parameter('v_max', 8.4)           # LiPo fully charged (4.2V × 2 cells)
        self.declare_parameter('v_nominal', 7.4)        # Nominal voltage
        self.declare_parameter('v_cutoff', 6.4)         # Low-voltage cutoff (3.2V × 2 cells)
        self.declare_parameter('r_internal', 0.10)      # Pack internal resistance (Ω)
        self.declare_parameter('capacity_ah', 0.30)     # Battery capacity (Ah)
        self.declare_parameter('tau_stall', 0.5394)     # PTK-7465 stall torque (N·m)
        self.declare_parameter('i_stall', 1.6)          # PTK-7465 stall current (A)
        self.declare_parameter('i_noload', 0.08)        # PTK-7465 no-load current (A)
        self.declare_parameter('max_discharge_c', 75.0)  # Max C-rating
        self.declare_parameter('publish_rate', 50.0)     # Publish rate (Hz) for battery state

        # Read parameters
        self.v_max = self.get_parameter('v_max').value
        self.v_nominal = self.get_parameter('v_nominal').value
        self.v_cutoff = self.get_parameter('v_cutoff').value
        self.r_internal = self.get_parameter('r_internal').value
        self.capacity_ah = self.get_parameter('capacity_ah').value
        self.tau_stall = self.get_parameter('tau_stall').value
        self.i_stall = self.get_parameter('i_stall').value
        self.i_noload = self.get_parameter('i_noload').value
        self.max_discharge_c = self.get_parameter('max_discharge_c').value
        publish_rate = self.get_parameter('publish_rate').value

        # Derived constants
        self.max_continuous_current = self.max_discharge_c * self.capacity_ah  # 75 × 0.3 = 22.5A
        self.torque_to_current_slope = (self.i_stall - self.i_noload) / self.tau_stall

        # ============================================================
        # State variables
        # ============================================================
        self.current_efforts = {name: 0.0 for name in self.JOINT_NAMES}
        self.total_current = 0.0
        self.v_actual = self.v_max
        self.degradation = 1.0
        self.energy_consumed_wh = 0.0  # Track cumulative energy consumption
        self.remaining_capacity = 1.0  # SOC: 0.0 to 1.0
        self.last_update_time = None

        # ============================================================
        # Subscribers
        # ============================================================
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/true_joint_states',
            self._joint_state_callback,
            10
        )

        # ============================================================
        # Publishers
        # ============================================================
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.degradation_pub = self.create_publisher(Float64, '/servo_degradation', 10)

        # Timer for publishing at a controlled rate
        timer_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(timer_period, self._publish_battery_state)

        # ============================================================
        # Startup log
        # ============================================================
        self.get_logger().info(
            f'Battery Sag Node started:\n'
            f'  Battery: Tattu 300mAh 7.6V 75C 2S1P LiHV\n'
            f'  V_max={self.v_max}V, V_cutoff={self.v_cutoff}V\n'
            f'  R_internal={self.r_internal * 1000:.0f}mΩ\n'
            f'  Max continuous: {self.max_continuous_current:.1f}A\n'
            f'  Servo: PTK-7465, τ_stall={self.tau_stall}N·m, I_stall={self.i_stall}A\n'
            f'  Torque→Current slope: {self.torque_to_current_slope:.4f} A/(N·m)'
        )

    def _joint_state_callback(self, msg: JointState):
        """
        Process joint state messages to extract effort (torque) values.
        Called at ~333 Hz (matching controller update rate).
        """
        if not msg.effort:
            return

        # Map joint names to effort values
        for i, name in enumerate(msg.name):
            if name in self.current_efforts and i < len(msg.effort):
                self.current_efforts[name] = msg.effort[i]

        # ============================================================
        # Step 1: Compute per-servo current from torque
        # ============================================================
        # Linear model: I = I_noload + (|τ| / τ_stall) × (I_stall - I_noload)
        # This models the DC motor characteristic where current is
        # proportional to torque load.
        per_servo_currents = []
        for name in self.JOINT_NAMES:
            abs_torque = abs(self.current_efforts.get(name, 0.0))
            # Clamp torque to stall value (can't exceed stall)
            abs_torque = min(abs_torque, self.tau_stall)
            current = self.i_noload + self.torque_to_current_slope * abs_torque
            per_servo_currents.append(current)

        # ============================================================
        # Step 2: Total current draw
        # ============================================================
        self.total_current = sum(per_servo_currents)

        # Clamp to max continuous discharge (safety)
        self.total_current = min(self.total_current, self.max_continuous_current)

        # ============================================================
        # Step 3: Voltage sag calculation
        # V_actual = V_max - (I_total × R_internal)
        # ============================================================
        v_drop = self.total_current * self.r_internal
        self.v_actual = self.v_max - v_drop

        # Clamp to cutoff voltage
        self.v_actual = max(self.v_actual, self.v_cutoff)

        # ============================================================
        # Step 4: Degradation multiplier
        # Represents how much the torque limit should be scaled down
        # ============================================================
        self.degradation = self.v_actual / self.v_max

        # ============================================================
        # Step 5: Track energy consumption (simple coulomb counting)
        # ============================================================
        current_time = self.get_clock().now()
        if self.last_update_time is not None:
            dt = (current_time - self.last_update_time).nanoseconds * 1e-9
            if dt > 0 and dt < 1.0:  # Sanity check
                # Energy in Wh: P × dt = V × I × dt(hours)
                energy_increment = self.v_actual * self.total_current * (dt / 3600.0)
                self.energy_consumed_wh += energy_increment
                # SOC based on simple energy counting
                total_energy_wh = self.v_nominal * self.capacity_ah  # ~2.28 Wh
                self.remaining_capacity = max(
                    0.0,
                    1.0 - (self.energy_consumed_wh / total_energy_wh)
                )
        self.last_update_time = current_time

    def _publish_battery_state(self):
        """Publish battery state and degradation multiplier."""
        # ============================================================
        # Publish BatteryState (ROS standard message)
        # ============================================================
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'base_link'
        battery_msg.voltage = float(self.v_actual)
        battery_msg.current = float(-self.total_current)  # Negative = discharging
        battery_msg.capacity = float(self.capacity_ah)
        battery_msg.design_capacity = float(self.capacity_ah)
        battery_msg.percentage = float(self.remaining_capacity)
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_msg.present = True

        # Per-cell voltages (2S)
        cell_voltage = self.v_actual / 2.0
        battery_msg.cell_voltage = [float(cell_voltage), float(cell_voltage)]
        battery_msg.cell_temperature = []  # Not modeled

        self.battery_pub.publish(battery_msg)

        # ============================================================
        # Publish degradation multiplier
        # ============================================================
        deg_msg = Float64()
        deg_msg.data = float(self.degradation)
        self.degradation_pub.publish(deg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
