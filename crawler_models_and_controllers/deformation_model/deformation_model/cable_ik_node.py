#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class CableIKNode(Node):
    """Unified cable IK: /cable/alp_nom_d + /cable/del_alp -> /cable/joint_cmd."""
    def __init__(self):
        super().__init__('cable_ik_node')
        self.declare_parameters('', [
            ('rate_hz', 100.0),
            ('topic_alp_nom_d', '/cable/alp_nom_d'),
            ('topic_del_alp', '/cable/del_alp'),
            ('topic_joint_cmd', '/cable/joint_cmd'),
        ])
        self.alp_nom = 0.0
        self.del_alp = 0.0
        self.sub_nom = self.create_subscription(Float32, self.get_parameter('topic_alp_nom_d').value, self.on_nom, 10)
        self.sub_del = self.create_subscription(Float32, self.get_parameter('topic_del_alp').value, self.on_del, 10)
        self.pub_joint = self.create_publisher(JointState, self.get_parameter('topic_joint_cmd').value, 10)
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_nom(self, msg):
        self.alp_nom = float(msg.data)

    def on_del(self, msg):
        self.del_alp = float(msg.data)

    def user_mapping(self, alp: float):
        # User-editable dummy mapping. Symmetric J4/J5 with opposite signs.
        j4 = alp
        j5 = -alp
        return j4, j5

    def tick(self):
        alp_cmd = self.alp_nom + self.del_alp
        j4, j5 = self.user_mapping(alp_cmd)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['J4', 'J5']
        js.position = [float(j4), float(j5)]
        self.pub_joint.publish(js)


def main():
    rclpy.init()
    node = CableIKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
