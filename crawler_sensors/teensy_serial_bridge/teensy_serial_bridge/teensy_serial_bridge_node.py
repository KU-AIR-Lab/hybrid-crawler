#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

try:
    import serial, struct
except Exception:
    serial = None
    struct = None

HEADER = b'\xAA\x55'

class TeensySerialBridge(Node):
    def __init__(self):
        super().__init__('teensy_serial_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyACM1').value
        self.baud = int(self.declare_parameter('baud', 2000000).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 250.0).value)
        self.mock = bool(self.declare_parameter('mock', False).value)
        self.mapping = {
            'cable_left':  (0,5),
            'cable_right': (5,10),
            'soft_front_contact': (10,12),
            'soft_rear_contact':  (12,14),
            'soft_front_pressure':(14,16),
            'soft_rear_pressure': (16,18),
        }
        self.pub_cable_left  = self.create_publisher(Float32MultiArray, '/sensors/cable/left/contact_array', 10)
        self.pub_cable_right = self.create_publisher(Float32MultiArray, '/sensors/cable/right/contact_array',10)
        self.pub_sf_fc = self.create_publisher(Float32MultiArray, '/sensors/soft/front/contact_array', 10)
        self.pub_sr_fc = self.create_publisher(Float32MultiArray, '/sensors/soft/rear/contact_array', 10)
        self.pub_sf_p  = self.create_publisher(Float32MultiArray, '/sensors/soft/front/pressure_array', 10)
        self.pub_sr_p  = self.create_publisher(Float32MultiArray, '/sensors/soft/rear/pressure_array', 10)
        self.dt = 1.0/max(1.0, self.rate_hz)
        self._buf = bytearray()

        if not self.mock and serial is not None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.001)
                self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial: {e}; switching to mock mode')
                self.mock = True
                self.ser = None
        else:
            self.ser = None

        self.timer = self.create_timer(self.dt, self.on_tick)

    def _read_frame(self):
        needed = 2 + 4 + 18*4 + 2
        start = time.time()
        while True:
            if self.ser is None or serial is None or struct is None:
                return None
            chunk = self.ser.read(1024)
            if chunk:
                self._buf.extend(chunk)
            idx = self._buf.find(HEADER)
            if idx >= 0 and len(self._buf) >= idx + needed:
                frame = self._buf[idx:idx+needed]
                del self._buf[:idx+needed]
                try:
                    ts = struct.unpack_from('<I', frame, 2)[0]
                    vals = list(struct.unpack_from('<18f', frame, 6))
                    return (ts, vals)
                except Exception:
                    continue
            if time.time() - start > 0.002:
                return None

    def _pub_slice(self, pub, vals, sl):
        msg = Float32MultiArray()
        msg.data = vals[sl[0]:sl[1]]
        pub.publish(msg)

    def on_tick(self):
        if self.mock:
            vals = [0.5,0.6,0.7,0.8,0.9,
                    0.4,0.3,0.2,0.1,0.0,
                    1.0,1.1, 1.2,1.3,
                    2.0,2.1, 2.2,2.3]
        else:
            fr = self._read_frame()
            if fr is None:
                return
            _, vals = fr

        self._pub_slice(self.pub_cable_left,  vals, self.mapping['cable_left'])
        self._pub_slice(self.pub_cable_right, vals, self.mapping['cable_right'])
        self._pub_slice(self.pub_sf_fc,       vals, self.mapping['soft_front_contact'])
        self._pub_slice(self.pub_sr_fc,       vals, self.mapping['soft_rear_contact'])
        self._pub_slice(self.pub_sf_p,        vals, self.mapping['soft_front_pressure'])
        self._pub_slice(self.pub_sr_p,        vals, self.mapping['soft_rear_pressure'])

def main():
    rclpy.init()
    node = TeensySerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
