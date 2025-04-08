import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.ser = serial.Serial('/dev/arduino_sensor', 115200, timeout=0.01)

        self.pub_hot_junction_1 = self.create_publisher(Float32, 'hot_junction_1', 10)
        self.pub_cold_junction_1 = self.create_publisher(Float32, 'cold_junction_1', 10)
        self.pub_hot_junction_2 = self.create_publisher(Float32, 'hot_junction_2', 10)
        self.pub_cold_junction_2 = self.create_publisher(Float32, 'cold_junction_2', 10)
        self.pub_humidity = self.create_publisher(Float32, 'humidity', 10)

    def try_read_and_publish(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            data = line.split(",")
            if len(data) == 6:
                _, hot1, cold1, hot2, cold2, humidity = map(float, data)
                self.pub_hot_junction_1.publish(Float32(data=hot1))
                self.pub_cold_junction_1.publish(Float32(data=cold1))
                self.pub_hot_junction_2.publish(Float32(data=hot2))
                self.pub_cold_junction_2.publish(Float32(data=cold2))
                self.pub_humidity.publish(Float32(data=humidity))
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)  # Process ROS events
            node.try_read_and_publish()               # Check serial line every loop
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

