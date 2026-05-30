#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SoftForwardStaticsNode(Node):
    """Reusable soft statics node for front or rear soft arm pair."""
    def __init__(self):
        super().__init__('soft_forward_statics_node')
        self.declare_parameters('', [
            ('topic_alp_nom_d', '/soft/front/alp_nom_d'),
            ('topic_p_nom_d', '/soft/front/p_nom_d'),
        ])
        self.sub = self.create_subscription(Float32, self.get_parameter('topic_alp_nom_d').value, self.on_alp, 10)
        self.pub = self.create_publisher(Float32, self.get_parameter('topic_p_nom_d').value, 10)

    def user_mapping(self, alp_nom: float) -> float:
        # User-editable dummy mapping: nominal soft-arm angle -> nominal pressure.
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
