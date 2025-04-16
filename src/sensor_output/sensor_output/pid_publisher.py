import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Float64MultiArray
import serial
import struct

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/pid_output', 10)

        # Setup serial connection to Teensy
        self.ser = serial.Serial('/dev/ttyACM1', 500000, timeout=0.01)

        # Downsampling parameters
        self.buffer = []
        self.samples_per_publish = 20

        # Call self.read_serial() every 0.5 ms
        self.create_timer(0.0005, self.read_serial)

    def read_serial(self):
        while self.ser.in_waiting >= 2:
            raw = self.ser.read(2)
            if len(raw) == 2:
                sample = struct.unpack('<H', raw)[0]
                self.buffer.append(sample)

#                if len(self.buffer) >= self.samples_per_publish:
#                    avg_val = int(sum(self.buffer) / len(self.buffer))
#                    self.buffer.clear()

#                    msg = UInt16()
#                    msg.data = avg_val
#                    self.publisher.publish(msg)

                if len(self.buffer) >= self.samples_per_publish:
                    avg_val = int(sum(self.buffer) / len(self.buffer))
                    self.buffer.clear()

                    timestamp = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds (float)
                    msg = Float64MultiArray()
                    msg.data = [timestamp, float(avg_val)]  # use float for consistency
                    self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = PIDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
