
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SoftForwardStaticsNode(Node):
    def __init__(self):
        super().__init__('soft_forward_statics_node')
        self.sub = self.create_subscription(Float32, '/soft/alp_nom_d', self.on_alp, 10)
        self.pub = self.create_publisher(Float32, '/soft/p_nom_d', 10)

    def user_mapping(self, alp_nom: float) -> float:
        k = 1.0
        return k * alp_nom

    def on_alp(self, msg: Float32):
        p_nom = self.user_mapping(float(msg.data))
        self.pub.publish(Float32(data=float(p_nom)))

def main():
    rclpy.init()
    node = SoftForwardStaticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
