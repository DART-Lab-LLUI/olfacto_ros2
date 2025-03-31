import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import termios
import tty
import sys

class KeyboardValveController(Node):
    def __init__(self):
        super().__init__('keyboard_valve_controller')
        
        self.get_logger().info('Initializing Keyboard Valve Controller Node')
        
        # Define key mapping for valve numbers
        self.key_map = {
            '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '0': 10,
            'q': 11, 'w': 12, 'e': 13, 'r': 14, 't': 15, 'y': 16, 'u' : 17, 'i' : 18, 'o' : 19, 'p' : 20 
        }
        
        self.current_valve = None  # Track currently open valve
        
        # Create publisher
        self.control_publisher = self.create_publisher(String, 'control_valve', 10)

        self.get_logger().info('Press keys 1-0 and qwertyuiop to control valves. Press Ctrl+C to exit.')
        self.listen_for_keys()

    def listen_for_keys(self):
        """ Continuously listen for key presses """
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)  # Read single character
                if key == '\x03':  # Handle Ctrl+C
                    raise KeyboardInterrupt
                if key in self.key_map:
                    self.toggle_valve(self.key_map[key])
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down keyboard valve controller')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            rclpy.shutdown()

    def toggle_valve(self, valve_number):
        """ Toggle the specified valve, ensuring only one is open at a time. """
        msg = String()
        
        # Close the currently open valve (if any)
        if self.current_valve is not None:
            msg.data = f'{self.current_valve}:OFF'
            self.control_publisher.publish(msg)
            self.get_logger().info(f'Closed valve {self.current_valve}')
        
        # Open the new valve only if it's different from the currently open one
        if self.current_valve != valve_number:
            msg.data = f'{valve_number}:ON'
            self.control_publisher.publish(msg)
            self.current_valve = valve_number
            self.get_logger().info(f'Opened valve {valve_number}')
        else:
            self.current_valve = None  # If the same valve is pressed again, it turns off


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardValveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down keyboard valve controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
