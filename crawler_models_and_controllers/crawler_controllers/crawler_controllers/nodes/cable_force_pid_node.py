import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from crawler_controllers.pid import PID

class CableForcePIDNode(Node):
    def __init__(self):
        super().__init__('cable_force_pid')
        self.declare_parameters('', [
            ('kp', 1.0), ('ki', 0.0), ('kd', 0.0),
            ('rate_hz', 50.0),
            ('u_min', -1.0), ('u_max', 1.0),
            ('integral_limit', 10.0),
            ('topic_Fc_d', '/cable/Fc_d'),
            ('topic_Fc_act', '/cable/Fc_act'),
            ('topic_cmd', '/cable/contact_force_cmd'),
        ])
        self.pid = PID(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            u_min=self.get_parameter('u_min').value,
            u_max=self.get_parameter('u_max').value,
            integral_limit=self.get_parameter('integral_limit').value,
        )
        self.Fc_d = 0.0
        self.Fc_act = 0.0
        self.create_subscription(Float32, self.get_parameter('topic_Fc_d').value, self._on_d, 10)
        self.create_subscription(Float32, self.get_parameter('topic_Fc_act').value, self._on_a, 10)
        self.pub = self.create_publisher(Float32, self.get_parameter('topic_cmd').value, 10)
        self.last = time.monotonic()
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_d(self, msg):
        self.Fc_d = float(msg.data)

    def _on_a(self, msg):
        self.Fc_act = float(msg.data)

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        u = self.pid.step(self.Fc_d - self.Fc_act, dt)
        self.pub.publish(Float32(data=float(u)))

def main():
    rclpy.init()
    node = CableForcePIDNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
