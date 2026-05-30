#!/usr/bin/env python3
import time
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from crawler_msgs.msg import JointSetpoint

def clamp(x,a,b): return a if x<a else (b if x>b else x)

class ReachedEstimator(Node):
    """
    Very simple 'reached' estimator from commanded JointSetpoint.
    - A joint is 'reached' if no new command has arrived for dwell_s,
      OR if its commanded speed is ~0 (<= speed_eps),
      OR if the last command is older than max_age_s (failsafe).
    """
    def __init__(self):
        super().__init__('reached_from_commands', automatically_declare_parameters_from_overrides=True)

        def p_float(name, default):
            try:
                return float(self.get_parameter(name).value)
            except Exception:
                self.get_logger().debug(f'Param "{name}" not set; using default {default}')
                return default

        def p_str(name, default):
            try:
                return str(self.get_parameter(name).value)
            except Exception:
                self.get_logger().debug(f'Param "{name}" not set; using default {default}')
                return default

        self.in_topic   = p_str('in_topic',  '/joint_setpoint')
        self.out_topic  = p_str('out_topic', '/maestro/reached')
        self.rate_hz    = p_float('rate_hz', 50.0)
        self.dwell_s    = p_float('dwell_s', 0.15)
        self.speed_eps  = p_float('speed_eps', 1e-3)
        self.max_age_s  = p_float('max_age_s', 2.0)

        self.pub = self.create_publisher(UInt8MultiArray, self.out_topic, 10)
        self.last: JointSetpoint | None = None
        self.last_t: float = 0.0

        self.sub = self.create_subscription(JointSetpoint, self.in_topic, self._on_cmd, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self._tick)

        self.get_logger().info(f'Publishing synthetic reached flags on {self.out_topic} (listen on {self.in_topic})')

    def _on_cmd(self, msg: JointSetpoint):
        self.last = msg
        self.last_t = time.time()

    def _tick(self):
        now = time.time()
        flags: List[int] = [0]*7

        if self.last is None:
            flags = [1]*7
        else:
            age = now - self.last_t
            quiet = age >= self.dwell_s

            # Safely turn speed into a python list; handle numpy arrays too.
            try:
                n = len(self.last.speed)
            except Exception:
                n = 0
            spd_list = list(self.last.speed) if n > 0 else []
            # normalize to length 7 (pad with zeros or clip)
            if len(spd_list) < 7:
                spd_list = spd_list + [0.0]*(7 - len(spd_list))
            elif len(spd_list) > 7:
                spd_list = spd_list[:7]

            for i in range(7):
                v = float(spd_list[i])
                reached = 1 if (quiet or abs(v) <= self.speed_eps or age > self.max_age_s) else 0
                flags[i] = reached

        self.pub.publish(UInt8MultiArray(data=flags))

def main():
    rclpy.init()
    node = ReachedEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

