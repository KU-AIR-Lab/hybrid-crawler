
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class CableIKNode(Node):
    def __init__(self):
        super().__init__('cable_ik_node')
        self.alp_nom = 0.0
        self.del_alp = 0.0
        self.sub_nom = self.create_subscription(Float32, '/cable/alp_nom_d', self.on_nom, 10)
        self.sub_del = self.create_subscription(Float32, '/cable/del_alp', self.on_del, 10)
        self.pub_joint = self.create_publisher(JointState, '/cable/joint_cmd', 10)
        self.timer = self.create_timer(0.01, self.tick)

    def on_nom(self, msg): self.alp_nom = float(msg.data)
    def on_del(self, msg): self.del_alp = float(msg.data)

    def user_mapping(self, alp: float):
        j4 = alp
        j5 = -alp
        return j4, j5

    def tick(self):
        alp_cmd = self.alp_nom + self.del_alp
        j4, j5 = self.user_mapping(alp_cmd)
        from sensor_msgs.msg import JointState
        js = JointState()
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
