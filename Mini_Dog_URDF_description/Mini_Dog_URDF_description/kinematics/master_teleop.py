#!/usr/bin/env python3
"""
Master Teleop Node — Unified Keyboard Controller for DAZZY
==========================================================

Integrates all 3 robot tricks into a single interactive keyboard controller:
  - [W / w] : Normal Trot Walk Gait (with IMU pitch balancing & stance bias)
  - [C / c] : Cute Crawl Gait (4-beat creep gait with yaw drift compensation)
  - [S / s] : Skateboard Gait (Synchronous 4-leg kick & glide, sim/hw auto-adapted)
  - [R / r] : Pivot Turn Right (Skid Steer)
  - [L / l] : Pivot Turn Left (Skid Steer)
  - [U / u] : Wheelie (Sit & Wave)
  - [SPACE / X / x] : Stand / Stop (Hold 0° posture)
  - [Q / Ctrl+C]    : Shutdown Teleop

Supports both Gazebo Simulation (`use_sim_mode:=True`) and Real Hardware (`use_sim_mode:=False`).

Joint Mapping (Empirically Verified):
  Index 0 (FL): -1.0
  Index 1 (RL): -1.0
  Index 2 (FR):  1.0
  Index 3 (RR):  1.0
"""

import sys
import os
import math
import select
import termios
import tty
import threading
import time

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


def get_key(settings, timeout=0.02):
    """Read a single raw keypress non-blockingly from stdin."""
    if sys.platform == 'win32':
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        return ''
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class MasterTeleopNode(Node):

    JOINT_LIMIT = 0.785398  # ±45 deg limit
    TRANSITION_TIME = 0.15   # Blending time when switching modes (seconds)

    def __init__(self, term_settings):
        super().__init__('master_teleop_node')
        self.term_settings = term_settings

        # ---- Parameters ----
        self.declare_parameter('use_sim_mode', True)
        self.declare_parameter('control_rate', 50.0)

        # Walk parameters
        self.declare_parameter('walk_amplitude', 0.15)
        self.declare_parameter('walk_frequency', 2.0)
        self.declare_parameter('walk_duty_factor', 0.65)
        self.declare_parameter('walk_stance_bias', 0.03)
        self.declare_parameter('kp_pitch', 0.3)

        # Crawl parameters
        self.declare_parameter('crawl_cycle_duration', 3.0)
        self.declare_parameter('crawl_reach_angle', -0.20)
        self.declare_parameter('crawl_push_angle', 0.10)
        self.declare_parameter('crawl_yaw_comp', 0.08)

        # Skateboard parameters
        self.declare_parameter('skate_stroke_amplitude', 0.18)
        self.declare_parameter('skate_front_offset', -0.15)
        self.declare_parameter('skate_rear_offset', 0.15)

        self.use_sim_mode = self.get_parameter('use_sim_mode').value
        self.control_rate = self.get_parameter('control_rate').value

        # Mode State Machine
        self.current_mode = 'STAND'  # 'STAND', 'WALK', 'CRAWL', 'SKATE'
        self.previous_mode = 'STAND'
        self.mode_start_time = None
        self.running = True

        # Joint Position Memory (for smooth transitions)
        self.current_cmd = [0.0, 0.0, 0.0, 0.0]
        self.blend_start_cmd = [0.0, 0.0, 0.0, 0.0]
        self.blend_start_time = 0.0

        # IMU & Motion State
        self.filtered_pitch = 0.0
        self.motion_level = 1.0
        self.imu_received = False

        # Skateboard sub-state machine
        self.skate_state = 'REACH'
        self.skate_state_start_time = 0.0
        
        # Wheelie sub-state machine
        self.wheelie_state = 'SIT'
        self.wheelie_state_start_time = 0.0

        # ROS Interfaces
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)

        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)

        # Start background keyboard thread
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()

        self._print_hud()

    def _imu_cb(self, msg: Imu):
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        if not self.imu_received:
            self.filtered_pitch = pitch
            self.imu_received = True
        else:
            self.filtered_pitch = 0.95 * self.filtered_pitch + 0.05 * pitch

        g_proj = 9.81 * math.sin(pitch)
        ax_net = abs(msg.linear_acceleration.x - g_proj)
        wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        gyro_mag = math.sqrt(wx*wx + wy*wy + wz*wz)
        self.motion_level = 0.85 * self.motion_level + 0.15 * (ax_net + gyro_mag)

    def _clamp(self, val):
        return max(-self.JOINT_LIMIT, min(self.JOINT_LIMIT, val))

    def _quintic_smooth(self, p):
        p = max(0.0, min(1.0, p))
        return 6 * (p ** 5) - 15 * (p ** 4) + 10 * (p ** 3)

    def _print_hud(self):
        mode_colors = {
            'STAND': '\033[1;33m[STAND / IDLE]\033[0m',
            'WALK': '\033[1;32m[WALK FORWARD]\033[0m',
            'CRAWL': '\033[1;36m[CRAWL GAIT]\033[0m',
            'SKATE': '\033[1;35m[SKATEBOARD GAIT]\033[0m',
            'PIVOT_RIGHT': '\033[1;31m[PIVOT RIGHT]\033[0m',
            'PIVOT_LEFT': '\033[1;31m[PIVOT LEFT]\033[0m',
            'WHEELIE': '\033[1;34m[SIT & WAVE]\033[0m'
        }
        sys.stdout.write('\033[2J\033[H')  # Clear screen
        sys.stdout.write(
            "============================================================\n"
            "   MINI ROBOT DOG (DAZZY) — UNIFIED TELEOP CONTROLLER       \n"
            "============================================================\n"
            "  [W / w]         : Trot Walk Gait (IMU Balanced)\n"
            "  [C / c]         : Crawl Gait (Cute 4-Beat Creep)\n"
            "  [S / s]         : Skateboard Gait (Kick & Glide)\n"
            "  [R / r]         : Pivot Turn Right (Skid Steer)\n"
            "  [L / l]         : Pivot Turn Left (Skid Steer)\n"
            "  [U / u]         : Wheelie (Sit & Wave)\n"
            "  [SPACE / X / x] : Stand / Stop (Hold Posture)\n"
            "  [Q / Ctrl+C]    : Shutdown Teleop\n"
            "============================================================\n"
            f"  MODE: {mode_colors.get(self.current_mode, self.current_mode)}\n"
            f"  Sim Mode: {self.use_sim_mode}\n"
            "============================================================\n"
            "  Press key to change trick/gait instantly...\n"
        )
        sys.stdout.flush()

    def set_mode(self, new_mode):
        if new_mode != self.current_mode:
            now = self.get_clock().now().nanoseconds / 1e9
            self.previous_mode = self.current_mode
            self.current_mode = new_mode
            self.mode_start_time = now
            self.skate_state = 'REACH'
            self.skate_state_start_time = now
            self.wheelie_state = 'SIT'
            self.wheelie_state_start_time = now

            # Snapshot current positions for smooth blending
            self.blend_start_cmd = list(self.current_cmd)
            self.blend_start_time = now
            self._print_hud()

    def _keyboard_loop(self):
        while self.running and rclpy.ok():
            key = get_key(self.term_settings, timeout=0.05)
            if key in ['w', 'W']:
                self.set_mode('WALK')
            elif key in ['c', 'C']:
                self.set_mode('CRAWL')
            elif key in ['s', 'S']:
                self.set_mode('SKATE')
            elif key in ['r', 'R']:
                self.set_mode('PIVOT_RIGHT')
            elif key in ['l', 'L']:
                self.set_mode('PIVOT_LEFT')
            elif key in ['u', 'U']:
                self.set_mode('WHEELIE')
            elif key in [' ', 'x', 'X']:
                self.set_mode('STAND')
            elif key in ['q', 'Q', '\x03']:
                self.running = False
                self.get_logger().info('Shutting down Master Teleop...')
                break

    # ------------------------------------------------------------------
    # Kinematics Controllers for the 3 Gaits
    # ------------------------------------------------------------------
    def _compute_walk_targets(self, elapsed):
        amp = self.get_parameter('walk_amplitude').value
        freq = self.get_parameter('walk_frequency').value
        duty = self.get_parameter('walk_duty_factor').value
        down_bias = self.get_parameter('walk_stance_bias').value
        kp_pitch = self.get_parameter('kp_pitch').value

        t_cycle = elapsed % (1.0 / freq)
        cf = t_cycle * freq

        def leg_angle(phase):
            swing_t = 1.0 - duty
            if phase < swing_t:
                p = phase / swing_t
                return amp - (2.0 * amp) * p, False
            else:
                p = (phase - swing_t) / duty
                return -amp + (2.0 * amp) * p, True

        ang1, stance1 = leg_angle(cf)
        ang2, stance2 = leg_angle((cf + 0.5) % 1.0)

        bias1 = down_bias if stance1 else 0.0
        bias2 = down_bias if stance2 else 0.0

        pitch_bias = max(-0.15, min(0.15, kp_pitch * self.filtered_pitch))

        fl = ang1 + bias1 + pitch_bias
        rr = ang1 + bias1 + pitch_bias
        fr = ang2 + bias2 + pitch_bias
        rl = ang2 + bias2 + pitch_bias

        return [fl, rl, fr, rr]

    def _compute_crawl_targets(self, elapsed):
        cycle_dur = self.get_parameter('crawl_cycle_duration').value
        reach = self.get_parameter('crawl_reach_angle').value
        push = self.get_parameter('crawl_push_angle').value
        yaw_comp = self.get_parameter('crawl_yaw_comp').value

        master_phase = (elapsed % cycle_dur) / cycle_dur

        def crawl_angle(phase):
            if phase < 0.25:
                p = phase / 0.25
                smooth = (1.0 - math.cos(math.pi * p)) / 2.0
                return push + (reach - push) * smooth
            else:
                p = (phase - 0.25) / 0.75
                smooth = (1.0 - math.cos(math.pi * p)) / 2.0
                return reach + (push - reach) * smooth

        phase_fl = (master_phase - 0.0) % 1.0
        phase_rr = (master_phase - 0.25) % 1.0
        phase_fr = (master_phase - 0.50) % 1.0
        phase_rl = (master_phase - 0.75) % 1.0

        target_fl = crawl_angle(phase_fl) * (1.0 - yaw_comp)
        target_rl = crawl_angle(phase_rl) * (1.0 - yaw_comp)
        target_fr = crawl_angle(phase_fr) * (1.0 + yaw_comp)
        target_rr = crawl_angle(phase_rr) * (1.0 + yaw_comp)

        return [target_fl, target_rl, target_fr, target_rr]

    def _compute_skate_targets(self, now, elapsed):
        amp = self.get_parameter('skate_stroke_amplitude').value
        front_off = self.get_parameter('skate_front_offset').value
        rear_off = self.get_parameter('skate_rear_offset').value

        if self.use_sim_mode:
            reach_dur, push_dur = 1.20, 0.12
            dir_mult = -1.0
        else:
            reach_dur, push_dur = 0.15, 0.45
            dir_mult = 1.0

        elapsed_in_state = now - self.skate_state_start_time

        if self.skate_state == 'REACH':
            p = min(1.0, elapsed_in_state / reach_dur)
            smooth = self._quintic_smooth(p)
            stroke = amp + (-amp - amp) * smooth
            if elapsed_in_state >= reach_dur:
                self.skate_state = 'PUSH'
                self.skate_state_start_time = now
        elif self.skate_state == 'PUSH':
            p = min(1.0, elapsed_in_state / push_dur)
            smooth = self._quintic_smooth(p)
            stroke = -amp + (amp - (-amp)) * smooth
            if elapsed_in_state >= push_dur:
                self.skate_state = 'SKATE'
                self.skate_state_start_time = now
        elif self.skate_state == 'SKATE':
            stroke = amp
            if elapsed_in_state >= 0.25:
                if self.motion_level < 0.5 or elapsed_in_state >= 1.5:
                    self.skate_state = 'REACH'
                    self.skate_state_start_time = now

        eff_stroke = stroke * dir_mult
        front_angle = eff_stroke + front_off
        rear_angle = eff_stroke + rear_off

        return [front_angle, rear_angle, front_angle, rear_angle]

    def _compute_pivot_targets(self, now, elapsed, turn_direction='right'):
        amp = self.get_parameter('skate_stroke_amplitude').value
        front_off = self.get_parameter('skate_front_offset').value
        rear_off = self.get_parameter('skate_rear_offset').value

        if self.use_sim_mode:
            reach_dur, push_dur = 1.20, 0.12
            dir_mult = -1.0
        else:
            reach_dur, push_dur = 0.15, 0.45
            dir_mult = 1.0

        elapsed_in_state = now - self.skate_state_start_time

        if self.skate_state == 'REACH':
            p = min(1.0, elapsed_in_state / reach_dur)
            smooth = self._quintic_smooth(p)
            stroke = amp + (-amp - amp) * smooth
            if elapsed_in_state >= reach_dur:
                self.skate_state = 'PUSH'
                self.skate_state_start_time = now
        elif self.skate_state == 'PUSH':
            p = min(1.0, elapsed_in_state / push_dur)
            smooth = self._quintic_smooth(p)
            stroke = -amp + (amp - (-amp)) * smooth
            if elapsed_in_state >= push_dur:
                self.skate_state = 'SKATE'
                self.skate_state_start_time = now
        elif self.skate_state == 'SKATE':
            stroke = amp
            if elapsed_in_state >= 0.25:
                if self.motion_level < 0.5 or elapsed_in_state >= 1.5:
                    self.skate_state = 'REACH'
                    self.skate_state_start_time = now

        eff_stroke = stroke * dir_mult
        active_front_angle = eff_stroke + front_off
        active_rear_angle = eff_stroke + rear_off

        passive_stroke = amp * dir_mult
        passive_front_angle = passive_stroke + front_off
        passive_rear_angle = passive_stroke + rear_off

        if turn_direction == 'right':
            # Drive LEFT side, passive RIGHT side
            return [active_front_angle, active_rear_angle, passive_front_angle, passive_rear_angle]
        else:
            # Drive RIGHT side, passive LEFT side
            return [passive_front_angle, passive_rear_angle, active_front_angle, active_rear_angle]

    def _compute_wheelie_targets(self, now, elapsed):
        sit_rear_angle = -0.70
        wave_base_angle = -0.20
        wave_amplitude = 0.40
        wave_frequency = 1.5
        sit_duration = 1.5
        
        elapsed_in_state = now - self.wheelie_state_start_time
        
        front_left_angle = 0.0
        front_right_angle = 0.0
        rear_angle = 0.0
        
        if self.wheelie_state == 'SIT':
            progress = min(1.0, elapsed_in_state / sit_duration)
            smooth = self._quintic_smooth(progress)
            
            rear_angle = 0.0 + (sit_rear_angle - 0.0) * smooth
            front_left_angle = 0.0 + (wave_base_angle - 0.0) * smooth
            front_right_angle = 0.0 + (wave_base_angle - 0.0) * smooth
            
            if elapsed_in_state >= sit_duration:
                self.wheelie_state = 'WAVE'
                self.wheelie_state_start_time = now
                
        elif self.wheelie_state == 'WAVE':
            rear_angle = sit_rear_angle
            phase = elapsed_in_state * wave_frequency * 2.0 * math.pi
            front_left_angle = wave_base_angle + wave_amplitude * math.sin(phase)
            front_right_angle = wave_base_angle + wave_amplitude * math.sin(phase + math.pi)
            
        return [front_left_angle, rear_angle, front_right_angle, rear_angle]

    # ------------------------------------------------------------------
    # Control Loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if not self.running:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.mode_start_time is None:
            self.mode_start_time = now

        elapsed = now - self.mode_start_time

        # Calculate raw un-mapped leg target angles
        if self.current_mode == 'WALK':
            targets = self._compute_walk_targets(elapsed)
        elif self.current_mode == 'CRAWL':
            targets = self._compute_crawl_targets(elapsed)
        elif self.current_mode == 'SKATE':
            targets = self._compute_skate_targets(now, elapsed)
        elif self.current_mode == 'PIVOT_RIGHT':
            targets = self._compute_pivot_targets(now, elapsed, 'right')
        elif self.current_mode == 'PIVOT_LEFT':
            targets = self._compute_pivot_targets(now, elapsed, 'left')
        elif self.current_mode == 'WHEELIE':
            targets = self._compute_wheelie_targets(now, elapsed)
        else:  # 'STAND'
            targets = [0.0, 0.0, 0.0, 0.0]

        # Apply Empirically Verified Hardware Multipliers
        # Index 0 (FL) -> -1.0
        # Index 1 (RL) -> -1.0
        # Index 2 (FR) ->  1.0
        # Index 3 (RR) ->  1.0
        raw_cmd = [
            targets[0] * -1.0,
            targets[1] * -1.0,
            targets[2] *  1.0,
            targets[3] *  1.0
        ]

        # Smooth blending transition when changing modes
        blend_elapsed = now - self.blend_start_time
        
        # Prevent negative elapsed time from causing wild alpha values (due to Gazebo resets or /clock flutters)
        if blend_elapsed < 0.0:
            blend_elapsed = 0.0
            
        if blend_elapsed < self.TRANSITION_TIME:
            alpha = blend_elapsed / self.TRANSITION_TIME
            cmd = [
                self.blend_start_cmd[i] + (raw_cmd[i] - self.blend_start_cmd[i]) * alpha
                for i in range(4)
            ]
        else:
            cmd = raw_cmd

        # Clamp and publish
        final_cmd = [self._clamp(c) for c in cmd]
        self.current_cmd = final_cmd

        msg = Float64MultiArray()
        msg.data = final_cmd
        self.cmd_pub.publish(msg)


def main(args=None):
    term_settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = MasterTeleopNode(term_settings)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero joint commands on exit
        zero = Float64MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        node.cmd_pub.publish(zero)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
