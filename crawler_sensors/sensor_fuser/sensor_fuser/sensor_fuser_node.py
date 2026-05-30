#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


def reduce_vals(vals, mode):
    vals = [float(v) for v in vals]
    if not vals:
        return 0.0
    mode = str(mode).lower().strip()
    if mode == 'max':
        return max(vals)
    if mode == 'min':
        return min(vals)
    if mode == 'sum':
        return sum(vals)
    # default: mean
    return sum(vals) / len(vals)


class SensorFuser(Node):
    """
    Raw sensing is left/right, but control is unified:

      Cable: all 10 cable contact sensors -> /cable/Fc_act
      Soft front: [left,right] contact -> /soft/front/Fc_act
                  [left,right] pressure -> /soft/front/p_act
      Soft rear:  [left,right] contact -> /soft/rear/Fc_act
                  [left,right] pressure -> /soft/rear/p_act

    Default reductions:
      cable force: max(all 10)
      soft force: max(left,right)
      soft pressure: mean(left,right)
    """
    def __init__(self):
        super().__init__('sensor_fuser')

        self.cable_reduce = self.declare_parameter('cable_reduce', 'max').value
        self.soft_front_force_reduce = self.declare_parameter('soft_front_force_reduce', 'max').value
        self.soft_rear_force_reduce = self.declare_parameter('soft_rear_force_reduce', 'max').value
        self.soft_front_pressure_reduce = self.declare_parameter('soft_front_pressure_reduce', 'mean').value
        self.soft_rear_pressure_reduce = self.declare_parameter('soft_rear_pressure_reduce', 'mean').value
        self.alpha_force = float(self.declare_parameter('alpha_force', 0.2).value)
        self.alpha_pressure = float(self.declare_parameter('alpha_pressure', 0.2).value)

        self._y = {}
        self._cable_left = []
        self._cable_right = []

        # Unified controller feedback topics
        self.pub_cable_fc = self.create_publisher(Float32, '/cable/Fc_act', 10)
        self.pub_soft_front_fc = self.create_publisher(Float32, '/soft/front/Fc_act', 10)
        self.pub_soft_rear_fc = self.create_publisher(Float32, '/soft/rear/Fc_act', 10)
        self.pub_soft_front_p = self.create_publisher(Float32, '/soft/front/p_act', 10)
        self.pub_soft_rear_p = self.create_publisher(Float32, '/soft/rear/p_act', 10)

        # Raw sensor array subscriptions from Teensy bridge
        self.create_subscription(Float32MultiArray, '/sensors/cable/left/contact_array', self._cb_cable_left, 10)
        self.create_subscription(Float32MultiArray, '/sensors/cable/right/contact_array', self._cb_cable_right, 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/front/contact_array', self._cb_soft_front_force, 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/rear/contact_array', self._cb_soft_rear_force, 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/front/pressure_array', self._cb_soft_front_pressure, 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/rear/pressure_array', self._cb_soft_rear_pressure, 10)

        self.get_logger().info('sensor_fuser ready: unified topics /cable/Fc_act, /soft/front/*, /soft/rear/*')

    def _smooth(self, key, x, alpha):
        alpha = max(0.0, min(1.0, float(alpha)))
        y_prev = self._y.get(key, x)
        y = alpha * x + (1.0 - alpha) * y_prev
        self._y[key] = y
        return y

    def _publish(self, pub, value):
        pub.publish(Float32(data=float(value)))

    def _publish_cable(self):
        vals = list(self._cable_left) + list(self._cable_right)
        if not vals:
            return
        x = reduce_vals(vals, self.cable_reduce)
        y = self._smooth('cable_fc', x, self.alpha_force)
        self._publish(self.pub_cable_fc, y)

    def _cb_cable_left(self, msg):
        self._cable_left = list(msg.data)
        self._publish_cable()

    def _cb_cable_right(self, msg):
        self._cable_right = list(msg.data)
        self._publish_cable()

    def _cb_soft_front_force(self, msg):
        x = reduce_vals(list(msg.data), self.soft_front_force_reduce)
        y = self._smooth('soft_front_fc', x, self.alpha_force)
        self._publish(self.pub_soft_front_fc, y)

    def _cb_soft_rear_force(self, msg):
        x = reduce_vals(list(msg.data), self.soft_rear_force_reduce)
        y = self._smooth('soft_rear_fc', x, self.alpha_force)
        self._publish(self.pub_soft_rear_fc, y)

    def _cb_soft_front_pressure(self, msg):
        x = reduce_vals(list(msg.data), self.soft_front_pressure_reduce)
        y = self._smooth('soft_front_p', x, self.alpha_pressure)
        self._publish(self.pub_soft_front_p, y)

    def _cb_soft_rear_pressure(self, msg):
        x = reduce_vals(list(msg.data), self.soft_rear_pressure_reduce)
        y = self._smooth('soft_rear_p', x, self.alpha_pressure)
        self._publish(self.pub_soft_rear_p, y)


def main():
    rclpy.init()
    node = SensorFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
