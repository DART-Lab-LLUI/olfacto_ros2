import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import threading

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/pid_output', 10)

        # Serial setup: match your Teensy port
        self.ser = serial.Serial('/dev/ttyACM1', 500000, timeout=0)

        # Start a thread to continuously read serial data
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    def read_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue

                parts = line.split(",")
                if len(parts) != 2:
                    continue  # Skip malformed lines

                teensy_time_us = int(parts[0])
                value = int(parts[1])

                ros_time = self.get_clock().now().nanoseconds * 1e-9  # seconds as float
                teensy_time = teensy_time_us * 1e-6  # convert µs to seconds

                msg = Float64MultiArray()
                msg.data = [ros_time, teensy_time, float(value)]
                self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().warn(f"Error reading serial: {e}")

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PIDPublisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
