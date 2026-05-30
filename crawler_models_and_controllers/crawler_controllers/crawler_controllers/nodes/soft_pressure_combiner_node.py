import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SoftPressureCombinerNode(Node):
    """Publishes p_d = p_nom_d + del_p for the soft arm pressure loop."""
    def __init__(self):
        super().__init__('soft_pressure_combiner')
        self.declare_parameters('', [
            ('rate_hz', 50.0),
            ('topic_p_nom_d', '/soft/p_nom_d'),
            ('topic_del_p', '/soft/del_p'),
            ('topic_p_d', '/soft/p_d'),
            ('p_min', -1.0e9),
            ('p_max',  1.0e9),
        ])
        self.p_nom_d = 0.0
        self.del_p = 0.0
        self.p_min = float(self.get_parameter('p_min').value)
        self.p_max = float(self.get_parameter('p_max').value)
        self.create_subscription(Float32, self.get_parameter('topic_p_nom_d').value, self._on_nom, 10)
        self.create_subscription(Float32, self.get_parameter('topic_del_p').value, self._on_del, 10)
        self.pub = self.create_publisher(Float32, self.get_parameter('topic_p_d').value, 10)
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_nom(self, msg):
        self.p_nom_d = float(msg.data)

    def _on_del(self, msg):
        self.del_p = float(msg.data)

    def _tick(self):
        p = self.p_nom_d + self.del_p
        p = max(self.p_min, min(self.p_max, p))
        self.pub.publish(Float32(data=float(p)))

def main():
    rclpy.init()
    node = SoftPressureCombinerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
