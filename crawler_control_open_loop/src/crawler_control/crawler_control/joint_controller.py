#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crawler_msgs.msg import JointSetpoint

class JointController(Node):
    """
    Subscribes /joint_goal and republishes /joint_setpoint at a fixed rate.
    """
    def __init__(self):
        super().__init__('joint_controller')
        self.pub = self.create_publisher(JointSetpoint, '/joint_setpoint', 10)
        self.sub = self.create_subscription(JointSetpoint, '/joint_goal', self.on_goal, 10)

        self.rate_hz = float(self.declare_parameter('rate_hz', 100.0).value)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_tick)

        self.last = JointSetpoint()
        self.last.position = [0.0]*7
        self.last.speed    = [0.0]*7
        self.last.pumps_front_on = False
        self.last.solenoids_front_on = False
        self.last.pumps_back_on = False
        self.last.solenoids_back_on = False

    def on_goal(self, msg: JointSetpoint):
        if len(msg.position) != 7 or len(msg.speed) != 7:
            self.get_logger().warn('Ignoring /joint_goal with wrong sizes.')
            return
        self.last = msg

    def on_tick(self):
        out = JointSetpoint()
        out.header.stamp = self.get_clock().now().to_msg()
        out.position = list(self.last.position)
        out.speed    = list(self.last.speed)
        out.pumps_front_on     = self.last.pumps_front_on
        out.solenoids_front_on = self.last.solenoids_front_on
        out.pumps_back_on      = self.last.pumps_back_on
        out.solenoids_back_on  = self.last.solenoids_back_on
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
