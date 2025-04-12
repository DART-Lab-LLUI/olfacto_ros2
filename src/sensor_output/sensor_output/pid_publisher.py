import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import serial
import struct

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.publisher = self.create_publisher(UInt16, '/pid_output', 10)

        # Setup serial connection to Teensy
        self.ser = serial.Serial('/dev/ttyACM1', 500000, timeout=0.01)

        # Downsampling parameters
        self.buffer = []
        self.samples_per_publish = 10  # Average 10 samples for 1 kHz publish

        # Call self.read_serial() every 0.5 ms
        self.create_timer(0.001, self.publish_average)

    # Add this method:
    def publish_average(self):
        if self.buffer:
            avg_val = int(sum(self.buffer) / len(self.buffer))
            self.buffer.clear()
            msg = UInt16()
            msg.data = avg_val
            self.publisher.publish(msg)


#    def read_serial(self):
#        while self.ser.in_waiting >= 2:
#            raw = self.ser.read(2)
#            if len(raw) == 2:
#                sample = struct.unpack('<H', raw)[0]
#                self.buffer.append(sample)

#                if len(self.buffer) >= self.samples_per_publish:
#                    avg_val = int(sum(self.buffer) / len(self.buffer))
#                    self.buffer.clear()

#                    msg = UInt16()
#                    msg.data = avg_val
#                    self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
