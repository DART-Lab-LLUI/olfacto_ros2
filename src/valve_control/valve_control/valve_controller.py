import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

class ValveController(Node):
    def __init__(self):
        super().__init__('valve_controller')
        
        self.get_logger().info('Initializing Valve Controller Node')
        
        # Setup serial connection to Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Change this to the correct port if necessary
            time.sleep(2)  # Wait for the connection to initialize
            self.get_logger().info('Connected to Arduino on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise e
        
        # Create subscription to ROS topic for controlling valves
        self.subscription = self.create_subscription(
            String,
            'control_valve',
            self.control_valve_callback,
            10
        )

    def control_valve(self, valve_number: int, state: str):
        """
        Control a specific valve by sending a command to the Arduino.
        Args:
            valve_number (int): The valve number to control (1-12)
            state (str): "ON" or "OFF"
        """
        if 1 <= valve_number <= 12:
            command = f'VALVE_{valve_number}_{state}\n'
            try:
                self.arduino.write(command.encode())
                self.get_logger().info(f'Sent: {command.strip()}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command to Arduino: {e}')
        else:
            self.get_logger().warn("Invalid valve number. Must be between 1 and 12.")

    def control_valve_callback(self, msg):
        """
        Callback function to handle incoming ROS topic messages.
        Expects the message to be in the format "valve_number:state" (e.g., "1:ON" or "3:OFF").
        """
        try:
            valve_number, state = msg.data.split(':')
            valve_number = int(valve_number)
            state = state.strip().upper()
            if state in ['ON', 'OFF']:
                self.control_valve(valve_number, state)
            else:
                self.get_logger().warn(f'Invalid state: {state}. Must be "ON" or "OFF".')
        except ValueError:
            self.get_logger().warn(f'Invalid message format: {msg.data}. Expected format is "valve_number:state" (e.g., "1:ON").')

    def destroy_node(self):
        """
        Ensure the serial connection is closed when the node is destroyed.
        """
        if self.arduino.is_open:
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ValveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down valve controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
