import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import serial
import struct

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.publisher = self.create_publisher(UInt16, '/pid_output', 10)

        # Connect to Teensy via USB serial
        self.ser = serial.Serial('/dev/ttyACM1', 500000, timeout=0.01)

        # Buffer for downsampling
        self.buffer = []

        # Read serial frequently (every 0.2 ms)
        self.create_timer(0.0002, self.read_serial)

        # Publish every 1 ms (1 kHz)
        self.create_timer(0.001, self.publish_average)

    def read_serial(self):
        while self.ser.in_waiting >= 2:
            raw = self.ser.read(2)
            if len(raw) == 2:
                sample = struct.unpack('<H', raw)[0]
                self.buffer.append(sample)

    def publish_average(self):
        if self.buffer:
            avg_val = int(sum(self.buffer) / len(self.buffer))
            self.buffer.clear()

            msg = UInt16()
            msg.data = avg_val
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
