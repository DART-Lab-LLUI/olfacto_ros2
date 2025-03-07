import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Float32

class OlfactometerController(Node):
    def __init__(self):
        super().__init__('olfactometer_controller')
        self.valve_publisher = self.create_publisher(String, 'control_valve', 10)
        self.mfc0_publisher = self.create_publisher(Float32, 'mfc0/set_flow_rate', 10)
        self.mfc1_publisher = self.create_publisher(Float32, 'mfc1/set_flow_rate', 10)
        self.current_valve = None  # Track the currently open valve
        self.delay_time = 0.2  # Delay before closing the previous valve (in seconds)

        self.get_logger().info("Olfactometer Controller initialized. Type 'valve_number:ratio' (e.g., '1:0.5') to control.")

    def process_input(self, user_input):
        """ Process the user input and send ROS messages. """
        try:
            valve_number, ratio = user_input.split(':')
            valve_number = int(valve_number)
            ratio = float(ratio)

            if valve_number < 1 or valve_number > 16:
                self.get_logger().error("Invalid valve number. Must be between 1 and 16.")
                return
            if not (0.0 <= ratio <= 1.0):
                self.get_logger().error("Invalid ratio. Must be between 0 and 1.")
                return

            # Calculate flow rates
            total_flow = 8.0  # Total flow in LPM
            flow_mfc0 = total_flow * ratio
            flow_mfc1 = total_flow * (1 - ratio)

            # Open new valve
            self.open_valve(valve_number)
            # If another valve is open, close it after
            if self.current_valve is not None and self.current_valve != valve_number:
                time.sleep(0.2)
                self.close_valve(self.current_valve)
            # Update the currently open valve
            self.current_valve = valve_number  

            # Publish flow rates
            self.mfc0_publisher.publish(Float32(data=flow_mfc0))
            self.mfc1_publisher.publish(Float32(data=flow_mfc1))
            self.get_logger().info(f"Set MFC0 to {flow_mfc0} LPM, MFC1 to {flow_mfc1} LPM")

        except ValueError:
            self.get_logger().error("Invalid input format. Use 'valve_number:ratio' (e.g., '1:0.5').")

    def close_valve(self, valve_number):
        """ Send close valve command to arduino """
        valve_msg = String()
        valve_msg.data = f"{valve_number}:OFF"
        self.valve_publisher.publish(valve_msg)
        self.get_logger().info(f"Closed valve {valve_number}")

    def open_valve(self, valve_number):
        """ Send open valve command to arduino """
        valve_msg = String()
        valve_msg.data = f"{valve_number}:ON"
        self.valve_publisher.publish(valve_msg)
        self.get_logger().info(f"Opened valve {valve_number}")


    def reset_mfcs(self):
        """ Sets both MFCs to 0 LPM. """
        self.mfc0_publisher.publish(Float32(data=0.0))
        self.mfc1_publisher.publish(Float32(data=0.0))
        self.get_logger().info("Set both MFCs to 0 LPM")

def main(args=None):
    rclpy.init(args=args)
    controller = OlfactometerController()

    try:
        while rclpy.ok():
            user_input = input("Enter command (valve_number:ratio) or 'exit' to quit: ")
            if user_input.lower() == 'exit':
                break
            controller.process_input(user_input)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down Olfactometer Controller.")
    finally:
        if controller.current_valve is not None:
            controller.close_valve(controller.current_valve)  # Close last valve before exit
        controller.reset_mfcs()  # Set MFCs to 0 LPM
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
