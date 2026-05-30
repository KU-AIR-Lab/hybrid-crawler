#!/usr/bin/env python3
# maestro_bridge.py — Pololu Mini Maestro bridge for ROS 2
# - J1/J2: linear carriages in meters (position or speed mode)
# - J3: lateral slider commanded in meters, converted to motor angle via pulley radius
# - J4..J7: revolute joints in radians
# - In J1/J2 position mode: speed[i] <= 0.0 means HARD HOLD (no motion), ignoring position[i]
# - Publishes /joint_estimate:
#       J1,J2: carriage positions [m]
#       J3:    slider position    [m]
#       J4..J7: (currently unused in estimate)

from typing import Dict, Any, List
import math
import serial   # pyserial

import rclpy
from rclpy.node import Node

from crawler_msgs.msg import JointSetpoint
from sensor_msgs.msg import JointState


class MaestroSerial:
    """Minimal Pololu Maestro serial helper (Compact protocol)."""
    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.02):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)

    def set_target_14bit(self, channel: int, quarter_us: int):
        q = max(0, min(16383, int(quarter_us)))
        self.ser.write(bytes([0x84, channel & 0xFF, q & 0x7F, (q >> 7) & 0x7F]))

    def set_digital(self, channel: int, high: bool):
        self.set_target_14bit(channel, 7000 if high else 0)


class MaestroBridge(Node):
    def __init__(self):
        super().__init__('maestro_bridge')

        # ---------- helpers ----------
        def d_param(name, default):
            return self.declare_parameter(name, default).value

        def d_list_i(name, default):
            v = d_param(name, default)
            return [int(x) for x in (v if isinstance(v, (list, tuple)) else [v])]

        def d_list_f(name, default):
            v = d_param(name, default)
            return [float(x) for x in (v if isinstance(v, (list, tuple)) else [v])]

        def d_int(name, default):   return int(d_param(name, default))
        def d_float(name, default): return float(d_param(name, default))
        def d_bool(name, default):  return bool(d_param(name, default))
        def d_str(name, default):   return str(d_param(name, default))

        # ---------- basics ----------
        self.port     = d_str('port', '/dev/ttyACM0')
        self.baud     = d_int('baud', 9600)
        self.rate_hz  = d_float('rate_hz', 50.0)
        self.debug    = d_bool('debug_log', False)
        self.stale_ms = d_int('stale_ms', 500)
        self.neutral_on_startup = d_bool('neutral_on_startup', True)

        # ---------- channels ----------
        self.ch_servo       = d_list_i('ch_servo',       [0, 1, 2, 3, 4, 5, 6])
        self.ch_sol_front   = d_int('ch_sol_front', 10)
        self.ch_pump_front  = d_int('ch_pump_front', 11)
        self.ch_sol_back    = d_int('ch_sol_back',  12)
        self.ch_pump_back   = d_int('ch_pump_back', 13)

        # ---------- J1/J2 (linear axes, meters) ----------
        self.mode_j12           = d_str('j12_mode', 'position').strip().lower()
        self.j12_center_us      = d_list_i('j12_center_us',      [1500, 1500])
        self.j12_deadband_us    = d_list_i('j12_deadband_us',    [60, 60])
        self.j12_k_us_per_mps   = d_list_f('j12_k_us_per_mps',   [6000.0, 6000.0])
        self.j12_vmax_mps_limit = d_list_f('j12_vmax_mps_limit', [0.10, 0.10])
        self.j12_amax_mps2      = d_list_f('j12_amax_mps2',      [0.80, 0.80])
        self.j12_pos_tol_m      = d_list_f('j12_pos_tol_m',      [0.001, 0.001])

        self.j12_pos_cmd_scale  = d_list_f('j12_pos_cmd_scale', [1.0, 1.0])
        self.j12_use_limits     = d_bool('j12_use_limits', True)

        # Joint limits J1/J2 if declared
        self.j12_min_m = [float('-inf'), float('-inf')]
        self.j12_max_m = [float('inf'),  float('inf')]
        jl = self.get_parameters_by_prefix('joint_limits')
        for k, p in jl.items():
            parts = k.split('.')
            if len(parts) == 2:
                j, b = parts
                if j in ('J1', 'J2'):
                    i = 0 if j == 'J1' else 1
                    try:
                        val = float(p.value)
                        if b == 'min':
                            self.j12_min_m[i] = val
                        if b == 'max':
                            self.j12_max_m[i] = val
                    except Exception:
                        pass

        # ---------- J3..J7 servo mapping (angles → PWM) ----------
        # J3 will be commanded in meters, but still mapped to PWM via angle using j37_k_us_per_rad.
        self.j37_center_us      = d_list_i('j37_center_us',      [1500, 1500, 1500, 1500, 1500])
        self.j37_k_us_per_rad   = d_list_f('j37_k_us_per_rad',   [600, 600, 600, 900, 900])
        self.j37_min_us         = d_list_i('j37_min_us',         [890, 900, 900, 700, 700])
        self.j37_max_us         = d_list_i('j37_max_us',         [2130, 2100, 2100, 2300, 2300])

        # ---------- J3: slider transfer function and limits ----------
        # Command x_d [m] → motor angle θ_cmd [rad] via:
        #   x = x_home + r * θ  ⇒  θ = (x - x_home)/r
        self.j3_pulley_radius_m = d_float('j3_pulley_radius_m', 0.01)  # default 1 cm
        self.j3_home_x_m        = d_float('j3_home_x_m', 0.0)
        # Optional slider limits in meters (independent of joint_limits.J3)
        self.j3_min_x_m         = d_float('j3_min_x_m', float('-inf'))
        self.j3_max_x_m         = d_float('j3_max_x_m', float('inf'))

        self.j3_x_m             = self.j3_home_x_m
        self.j3_angle_rad       = 0.0

        # ---------- Maestro hardware ----------
        try:
            self.m = MaestroSerial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f'Maestro connected on {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to open {self.port}: {e}')
            raise

        # ---------- internal state ----------
        self._have_time = False
        self._last_us   = 0
        self.j12_x_m    = [0.0, 0.0]
        self.j12_v_mps  = [0.0, 0.0]
        for i in range(2):
            if math.isfinite(self.j12_min_m[i]) and math.isfinite(self.j12_max_m[i]):
                self.j12_x_m[i] = 0.5 * (self.j12_min_m[i] + self.j12_max_m[i])

        if self.neutral_on_startup:
            self.drive_neutral()

        # ROS I/O
        self.sub_cmd = self.create_subscription(JointSetpoint, '/joint_setpoint', self.on_setpoint, 10)
        self.pub_est = self.create_publisher(JointState, '/joint_estimate', 10)

        self.last_msg = None
        self.last_ms  = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_tick)

    # ---------- J3 transfer helpers ----------
    def j3_angle_to_x(self, angle_rad: float) -> float:
        """
        Motor angle (rad) → lateral slider travel (m).
        x = x_home + r * θ
        """
        return self.j3_home_x_m + self.j3_pulley_radius_m * angle_rad

    def j3_x_to_angle(self, x_m: float) -> float:
        """
        Lateral slider travel (m) → motor angle (rad).
        N = x / (2πr), θ = 2πN = x / r.
        """
        r = max(1e-9, self.j3_pulley_radius_m)
        return (x_m - self.j3_home_x_m) / r

    # ---------- core bridge ----------
    def drive_neutral(self):
        # J1/J2 to neutral
        for i in range(2):
            self.m.set_target_14bit(self.ch_servo[i], int(self.j12_center_us[i]) * 4)
        # J3..J7 to neutral centers
        for j in range(2, 7):
            idx = j - 2
            self.m.set_target_14bit(self.ch_servo[j], int(self.j37_center_us[idx]) * 4)
        # Actuators off
        self.m.set_digital(self.ch_sol_front, False)
        self.m.set_digital(self.ch_pump_front, False)
        self.m.set_digital(self.ch_sol_back,  False)
        self.m.set_digital(self.ch_pump_back, False)

    def on_setpoint(self, msg: JointSetpoint):
        self.last_msg = msg
        self.last_ms  = int(self.get_clock().now().nanoseconds // 1_000_000)

    def on_tick(self):
        now_us = int(self.get_clock().now().nanoseconds // 1000)
        dt = 0.0
        if self._have_time:
            dt = max(0.0, (now_us - self._last_us) * 1e-6)
        self._last_us = now_us
        self._have_time = True

        # If no fresh command, relax J1/J2 and actuators, keep last J3..J7 PWM
        if self.last_msg is None or ((now_us // 1000) - self.last_ms) > self.stale_ms:
            for i in range(2):
                self.m.set_target_14bit(self.ch_servo[i], int(self.j12_center_us[i]) * 4)
                self.j12_v_mps[i] *= 0.8
            self.m.set_digital(self.ch_sol_front, False)
            self.m.set_digital(self.ch_pump_front, False)
            self.m.set_digital(self.ch_sol_back,  False)
            self.m.set_digital(self.ch_pump_back, False)
            self._publish_estimate()
            return

        pos = list(self.last_msg.position)
        spd = list(self.last_msg.speed)

        # ===== J1/J2 (linear axes) =====
        if self.mode_j12 == 'speed':
            for i in range(2):
                v_req = float(spd[i])
                vmax  = max(1e-9, float(self.j12_vmax_mps_limit[i]))
                if abs(v_req) < 1e-9:
                    us = int(self.j12_center_us[i])
                    self.j12_v_mps[i] *= 0.6
                else:
                    sign = 1.0 if v_req > 0 else -1.0
                    mag  = min(abs(v_req) / vmax, 1.0)
                    max_off = int(self.j12_k_us_per_mps[i] * vmax)
                    offset  = int(self.j12_deadband_us[i] + max(0, max_off - self.j12_deadband_us[i]) * mag)
                    us = int(self.j12_center_us[i] + sign * offset)
                    self.j12_v_mps[i] = sign * mag * vmax
                    self.j12_x_m[i] += self.j12_v_mps[i] * dt
                self.m.set_target_14bit(self.ch_servo[i], us * 4)
        else:
            # POSITION mode with HARD HOLD behavior for speed<=0
            for i in range(2):
                x = float(self.j12_x_m[i])
                v = float(self.j12_v_mps[i])

                vmax_in = float(spd[i]) if i < len(spd) else 0.0
                hold = vmax_in <= 0.0

                if hold:
                    # HARD HOLD: neutral PWM, freeze position, zero velocity
                    self.m.set_target_14bit(self.ch_servo[i], int(self.j12_center_us[i]) * 4)
                    self.j12_v_mps[i] *= 0.5
                    if abs(self.j12_v_mps[i]) < 1e-4:
                        self.j12_v_mps[i] = 0.0
                else:
                    # scaled position command (compensation)
                    xt = float(pos[i]) * float(self.j12_pos_cmd_scale[i])

                    if self.j12_use_limits:
                        xt = max(self.j12_min_m[i], min(self.j12_max_m[i], xt))

                    vmax = min(abs(vmax_in), self.j12_vmax_mps_limit[i])
                    vmax = max(1e-6, vmax)
                    amax = max(1e-6, self.j12_amax_mps2[i])

                    err  = xt - x
                    dirn = 1.0 if err >= 0.0 else -1.0
                    ds   = (v * v) / (2.0 * amax)
                    need_decel = abs(err) <= ds + 1e-9
                    a = (-dirn * amax) if need_decel else (dirn * amax)

                    v = v + a * dt
                    if dirn > 0:
                        v = max(0.0, min(v,  vmax))
                    else:
                        v = min(0.0, max(v, -vmax))
                    step = v * dt
                    if abs(step) > abs(err):
                        step = err
                        v = step / dt if dt > 0.0 else 0.0
                    x = x + step

                    if abs(xt - x) <= self.j12_pos_tol_m[i] and abs(v) <= 0.5 * self.j12_pos_tol_m[i]:
                        x = xt
                        v = 0.0

                    sign = 1.0 if v > 0.0 else (-1.0 if v < 0.0 else 0.0)
                    if sign == 0.0:
                        us = int(self.j12_center_us[i])
                    else:
                        mag = min(abs(v) / vmax, 1.0)
                        max_off = int(self.j12_k_us_per_mps[i] * vmax)
                        offset  = int(self.j12_deadband_us[i] + max(0, max_off - self.j12_deadband_us[i]) * mag)
                        us = int(self.j12_center_us[i] + sign * offset)
                    self.m.set_target_14bit(self.ch_servo[i], us * 4)
                    self.j12_x_m[i]   = x
                    self.j12_v_mps[i] = v

        # ===== J3..J7 =====
        def clamp(v, a, b):
            return a if v < a else (b if v > b else v)

        for j in range(2, 7):
            idx = j - 2

            if j == 2:
                # J3: command is slider position in meters
                if len(pos) > 2:
                    x_cmd = float(pos[2])
                else:
                    x_cmd = self.j3_x_m  # hold last

                # Apply slider soft-limits if configured
                if math.isfinite(self.j3_min_x_m):
                    x_cmd = max(self.j3_min_x_m, x_cmd)
                if math.isfinite(self.j3_max_x_m):
                    x_cmd = min(self.j3_max_x_m, x_cmd)

                theta_cmd = self.j3_x_to_angle(x_cmd)
                self.j3_angle_rad = theta_cmd
                self.j3_x_m       = self.j3_angle_to_x(theta_cmd)  # should == clamped x_cmd

                ang = theta_cmd
            else:
                # J4..J7 still commanded in radians
                ang = float(pos[j]) if j < len(pos) else 0.0

            us = int(self.j37_center_us[idx] + self.j37_k_us_per_rad[idx] * ang)
            us = clamp(us, self.j37_min_us[idx], self.j37_max_us[idx])
            self.m.set_target_14bit(self.ch_servo[j], us * 4)

        # Actuators
        self.m.set_digital(self.ch_sol_front, bool(self.last_msg.solenoids_front_on))
        self.m.set_digital(self.ch_pump_front, bool(self.last_msg.pumps_front_on))
        self.m.set_digital(self.ch_sol_back,  bool(self.last_msg.solenoids_back_on))
        self.m.set_digital(self.ch_pump_back, bool(self.last_msg.pumps_back_on))

        self._publish_estimate()

    def _publish_estimate(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.position = [math.nan] * 7
        js.velocity = [math.nan] * 7

        # J1/J2: carriage positions and velocities [m, m/s]
        js.position[0], js.position[1] = self.j12_x_m[0], self.j12_x_m[1]
        js.velocity[0], js.velocity[1] = self.j12_v_mps[0], self.j12_v_mps[1]

        # J3: slider position in meters
        js.position[2] = self.j3_x_m
        # (velocity left as NaN for now)

        self.pub_est.publish(js)


def main():
    rclpy.init()
    node = MaestroBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()