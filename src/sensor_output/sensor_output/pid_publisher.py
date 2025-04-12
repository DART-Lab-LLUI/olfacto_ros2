import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import serial
import struct

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.publisher = self.create_publisher(UInt16, '/pid_output', 10)

        # Serial connection
        self.ser = serial.Serial('/dev/ttyACM1', 500000, timeout=0.01)
        self.buffer = []
        self.samples_per_publish = 10

        # Timers
        self.create_timer(0.0002, self.read_serial)  # fast polling
        self.create_timer(0.001, self.publish_average)  # try to publish at 1 kHz

    def read_serial(self):
        while self.ser.in_waiting >= 2:
            raw = self.ser.read(2)
            if len(raw) == 2:
                sample = struct.unpack('<H', raw)[0]
                self.buffer.append(sample)

    def publish_average(self):
        if len(self.buffer) >= self.samples_per_publish:
            avg_val = int(sum(self.buffer[:self.samples_per_publish]) / self.samples_per_publish)
            del self.buffer[:self.samples_per_publish]

            msg = UInt16()
            msg.data = avg_val
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
