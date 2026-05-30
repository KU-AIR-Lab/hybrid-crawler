#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SoftContactModelNode(Node):
    """Soft contact model. Reusable for front and rear instances."""
    def __init__(self):
        super().__init__('soft_contact_model')
        self.declare_parameters('', [
            ('topic_cmd', '/soft/front/contact_force_cmd'),
            ('topic_del_p', '/soft/front/del_p'),
        ])
        self.sub = self.create_subscription(Float32, self.get_parameter('topic_cmd').value, self.on_cmd, 10)
        self.pub = self.create_publisher(Float32, self.get_parameter('topic_del_p').value, 10)

    def user_mapping(self, fc_cmd: float) -> float:
        # User-editable dummy mapping: contact-force command -> pressure increment.
        k = 5.0
        return k * fc_cmd

    def on_cmd(self, msg: Float32):
        del_p = self.user_mapping(float(msg.data))
        self.pub.publish(Float32(data=float(del_p)))


def main():
    rclpy.init()
    node = SoftContactModelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
