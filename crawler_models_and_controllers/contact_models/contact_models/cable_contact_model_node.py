#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CableContactModelNode(Node):
    """Unified cable contact model: /cable/contact_force_cmd -> /cable/del_alp."""
    def __init__(self):
        super().__init__('cable_contact_model')
        self.declare_parameters('', [
            ('topic_cmd', '/cable/contact_force_cmd'),
            ('topic_del_alp', '/cable/del_alp'),
        ])
        self.sub = self.create_subscription(Float32, self.get_parameter('topic_cmd').value, self.on_cmd, 10)
        self.pub = self.create_publisher(Float32, self.get_parameter('topic_del_alp').value, 10)

    def user_mapping(self, fc_cmd: float) -> float:
        # User-editable dummy mapping: desired force-controller command -> angle increment.
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
