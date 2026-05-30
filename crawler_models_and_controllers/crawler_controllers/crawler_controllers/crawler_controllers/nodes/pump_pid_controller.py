import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from crawler_controllers.pid import PID
import time

class PumpPIDController(Node):
    def __init__(self):
        super().__init__('pump_pid_controller')
        self.declare_parameters('', [
            ('kp', 1.0), ('ki', 0.0), ('kd', 0.0),
            ('rate_hz', 50.0),
            ('u_min', 0.0), ('u_max', 1.0),
            ('topic_p_d', '/soft/p_d'),
            ('topic_p_act', '/soft/p_act'),
            ('topic_pwm', '/soft/pump_pwm')
        ])
        self.pid = PID(
            kp=float(self.get_parameter('kp').value),
            ki=float(self.get_parameter('ki').value),
            kd=float(self.get_parameter('kd').value),
            u_min=float(self.get_parameter('u_min').value),
            u_max=float(self.get_parameter('u_max').value),
        )
        self.p_d = 0.0
        self.p_act = 0.0
        self.sub_d = self.create_subscription(Float64, self.get_parameter('topic_p_d').value, self._on_d, 10)
        self.sub_a = self.create_subscription(Float64, self.get_parameter('topic_p_act').value, self._on_a, 10)
        self.pub = self.create_publisher(Float64, self.get_parameter('topic_pwm').value, 10)
        self.last = time.time()
        dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(dt, self._tick)
    def _on_d(self, msg): self.p_d = float(msg.data)
    def _on_a(self, msg): self.p_act = float(msg.data)
    def _tick(self):
        now = time.time(); dt = now - self.last; self.last = now
        e = self.p_d - self.p_act
        u = self.pid.step(e, dt)
        self.pub.publish(Float64(data=float(u)))

def main():
    rclpy.init()
    n = PumpPIDController()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
