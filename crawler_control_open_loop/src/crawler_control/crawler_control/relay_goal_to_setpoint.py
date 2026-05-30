#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crawler_msgs.msg import JointSetpoint

class Relay(Node):
    def __init__(self):
        # let ROS auto-declare any params from the launch overrides
        super().__init__('relay_goal_to_setpoint', automatically_declare_parameters_from_overrides=True)

        # helpers to read-or-default without redeclaring
        def p_str(name, default):
            try:
                return str(self.get_parameter(name).value)
            except Exception:
                self.get_logger().debug(f'Param "{name}" not set; using default {default}')
                return default

        def p_float(name, default):
            try:
                return float(self.get_parameter(name).value)
            except Exception:
                self.get_logger().debug(f'Param "{name}" not set; using default {default}')
                return default

        self.in_topic  = p_str('in_topic',  '/joint_goal')
        self.out_topic = p_str('out_topic', '/joint_setpoint')
        self.rate_hz   = p_float('rate_hz', 50.0)

        self.pub = self.create_publisher(JointSetpoint, self.out_topic, 10)
        self.last_msg = None

        self.sub = self.create_subscription(JointSetpoint, self.in_topic, self._on_msg, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self._tick)

        self.get_logger().info(f'Relaying {self.in_topic}  →  {self.out_topic} @ {self.rate_hz} Hz')

    def _on_msg(self, msg: JointSetpoint):
        # pass through immediately and also cache for periodic re-publish
        self.last_msg = msg
        self.pub.publish(msg)

    def _tick(self):
        if self.last_msg is not None:
            self.pub.publish(self.last_msg)

def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

