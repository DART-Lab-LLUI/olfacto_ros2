import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/arduino_sensor', 115200, timeout=0.1)  # Change if needed

        # Publishers for each sensor
        self.pub_hot_junction_1 = self.create_publisher(Float32, 'hot_junction_1', 10)
        self.pub_cold_junction_1 = self.create_publisher(Float32, 'cold_junction_1', 10)
        self.pub_hot_junction_2 = self.create_publisher(Float32, 'hot_junction_2', 10)
        self.pub_cold_junction_2 = self.create_publisher(Float32, 'cold_junction_2', 10)
        self.pub_humidity = self.create_publisher(Float32, 'humidity', 10)

        # Independent timers for each sensor type
        self.timer_thermocouples = self.create_timer(0.032, self.publish_thermocouples)  # ~31.2 Hz
        self.timer_humidity = self.create_timer(0.001, self.publish_humidity)  # ~1000 Hz

    def read_serial_data(self):
        """ Reads and parses serial data from Arduino """
        try:
            line = self.ser.readline().decode().strip()
            if line:
                data = line.split(",")  # Expected: timestamp,hot1,cold1,hot2,cold2,humidity
                if len(data) == 6:
                    return [float(x) for x in data]  # Convert to floats
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
        return None

    def publish_thermocouples(self):
        """ Reads and publishes thermocouple sensor data """
        data = self.read_serial_data()
        if data:
            _, hot1, cold1, hot2, cold2, _ = data  # Ignore timestamp and humidity

            self.pub_hot_junction_1.publish(Float32(data=hot1))
            self.pub_cold_junction_1.publish(Float32(data=cold1))
            self.pub_hot_junction_2.publish(Float32(data=hot2))
            self.pub_cold_junction_2.publish(Float32(data=cold2))

            #self.get_logger().info(f'Published: Hot1={hot1} Cold1={cold1} Hot2={hot2} Cold2={cold2}')

    def publish_humidity(self):
        """ Reads and publishes humidity sensor data """
        data = self.read_serial_data()
        if data:
            _, _, _, _, _, humidity = data  # Only take humidity

            self.pub_humidity.publish(Float32(data=humidity))
            #self.get_logger().info(f'Published: Humidity={humidity} %RH')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
