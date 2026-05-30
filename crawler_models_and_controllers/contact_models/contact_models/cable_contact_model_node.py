
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CableContactModelNode(Node):
    def __init__(self):
        super().__init__('cable_contact_model_node')
        self.sub = self.create_subscription(Float32, '/cable/contact_force_cmd', self.on_cmd, 10)
        self.pub = self.create_publisher(Float32, '/cable/del_alp', 10)

    def user_mapping(self, fc_cmd: float) -> float:
        k = 0.01
        return k * fc_cmd

    def on_cmd(self, msg: Float32):
        del_alp = self.user_mapping(float(msg.data))
        self.pub.publish(Float32(data=float(del_alp)))

def main():
    rclpy.init()
    node = CableContactModelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
