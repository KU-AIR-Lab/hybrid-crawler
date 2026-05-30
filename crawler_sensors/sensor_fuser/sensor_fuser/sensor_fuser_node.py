#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

def reduce_vals(vals, mode):
    vals = [float(v) for v in vals]
    if not vals:
        return 0.0
    if mode == 'max':
        return max(vals)
    if mode == 'mean':
        return sum(vals)/len(vals)
    return sum(vals)/len(vals)

class SensorFuser(Node):
    def __init__(self):
        super().__init__('sensor_fuser')
        self.c_left_mode  = self.declare_parameter('cable_left_reduce',  'max').value
        self.c_right_mode = self.declare_parameter('cable_right_reduce', 'max').value
        self.soft_f_mode  = self.declare_parameter('soft_force_reduce',  'identity').value
        self.soft_p_mode  = self.declare_parameter('soft_pressure_reduce','identity').value
        self.alphaF = float(self.declare_parameter('alpha_force', 0.2).value)
        self.alphaP = float(self.declare_parameter('alpha_pressure', 0.2).value)
        self._y = {}
        self.pub_fc_left  = self.create_publisher(Float32, '/cable/left/Fc_act', 10)
        self.pub_fc_right = self.create_publisher(Float32, '/cable/right/Fc_act',10)
        self.pub_fc_sf_fl = self.create_publisher(Float32, '/soft/front/left/Fc_act', 10)
        self.pub_fc_sf_fr = self.create_publisher(Float32, '/soft/front/right/Fc_act',10)
        self.pub_fc_sr_fl = self.create_publisher(Float32, '/soft/rear/left/Fc_act', 10)
        self.pub_fc_sr_fr = self.create_publisher(Float32, '/soft/rear/right/Fc_act',10)
        self.pub_p_sf_fl  = self.create_publisher(Float32, '/soft/front/left/p_act', 10)
        self.pub_p_sf_fr  = self.create_publisher(Float32, '/soft/front/right/p_act',10)
        self.pub_p_sr_fl  = self.create_publisher(Float32, '/soft/rear/left/p_act', 10)
        self.pub_p_sr_fr  = self.create_publisher(Float32, '/soft/rear/right/p_act',10)
        self.create_subscription(Float32MultiArray, '/sensors/cable/left/contact_array', self._cb_cable_left, 10)
        self.create_subscription(Float32MultiArray, '/sensors/cable/right/contact_array', self._cb_cable_right, 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/front/contact_array',
                                 lambda m: self._cb_soft_pair(m, 'sf_fc', self.soft_f_mode,
                                                              (self.pub_fc_sf_fl, self.pub_fc_sf_fr), self.alphaF), 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/rear/contact_array',
                                 lambda m: self._cb_soft_pair(m, 'sr_fc', self.soft_f_mode,
                                                              (self.pub_fc_sr_fl, self.pub_fc_sr_fr), self.alphaF), 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/front/pressure_array',
                                 lambda m: self._cb_soft_pair(m, 'sf_p', self.soft_p_mode,
                                                              (self.pub_p_sf_fl, self.pub_p_sf_fr), self.alphaP), 10)
        self.create_subscription(Float32MultiArray, '/sensors/soft/rear/pressure_array',
                                 lambda m: self._cb_soft_pair(m, 'sr_p', self.soft_p_mode,
                                                              (self.pub_p_sr_fl, self.pub_p_sr_fr), self.alphaP), 10)

    def _smooth(self, key, x, alpha):
        y_prev = self._y.get(key, x)
        y = alpha * x + (1.0 - alpha) * y_prev
        self._y[key] = y
        return y

    def _cb_cable_left(self, msg):
        vals = list(msg.data)
        x = reduce_vals(vals, self.c_left_mode)
        y = self._smooth('c_left', x, self.alphaF)
        out = Float32(); out.data = float(y)
        self.pub_fc_left.publish(out)

    def _cb_cable_right(self, msg):
        vals = list(msg.data)
        x = reduce_vals(vals, self.c_right_mode)
        y = self._smooth('c_right', x, self.alphaF)
        out = Float32(); out.data = float(y)
        self.pub_fc_right.publish(out)

    def _cb_soft_pair(self, msg, key_prefix, mode, pubs, alpha):
        vals = list(msg.data)
        if mode == 'identity' and len(vals) >= 2:
            L, R = float(vals[0]), float(vals[1])
        else:
            v = reduce_vals(vals, mode)
            L, R = v, v
        yL = self._smooth(key_prefix+'_L', L, alpha)
        yR = self._smooth(key_prefix+'_R', R, alpha)
        mL = Float32(); mL.data = float(yL)
        mR = Float32(); mR.data = float(yR)
        pubs[0].publish(mL)
        pubs[1].publish(mR)

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
