import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

class PumpBangBangController(Node):
    def __init__(self):
        super().__init__('pump_bangbang_controller')
        self.declare_parameters('', [
            ('deadband', 0.02),
            ('topic_p_d', '/soft/p_d'),
            ('topic_p_act', '/soft/p_act'),
            ('topic_on', '/soft/pump_on')
        ])
        self.p_d = 0.0
        self.p_act = 0.0
        self.deadband = float(self.get_parameter('deadband').value)
        self.sub_d = self.create_subscription(Float64, self.get_parameter('topic_p_d').value, self._on_d, 10)
        self.sub_a = self.create_subscription(Float64, self.get_parameter('topic_p_act').value, self._on_a, 10)
        self.pub = self.create_publisher(Bool, self.get_parameter('topic_on').value, 10)
        self.timer = self.create_timer(0.02, self._tick)
    def _on_d(self, msg): self.p_d = float(msg.data)
    def _on_a(self, msg): self.p_act = float(msg.data)
    def _tick(self):
        e = self.p_d - self.p_act
        on = e > self.deadband
        self.pub.publish(Bool(data=on))

def main():
    rclpy.init()
    n = PumpBangBangController()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
