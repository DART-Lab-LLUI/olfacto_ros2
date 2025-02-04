import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensirion_shdlc_driver import ShdlcSerialPort
from sensirion_shdlc_driver.errors import ShdlcDeviceError
from sensirion_driver_adapters.shdlc_adapter.shdlc_channel import ShdlcChannel
from sensirion_uart_sfx6xxx.device import Sfx6xxxDevice
from sensirion_uart_sfx6xxx.commands import StatusCode

class MassFlowControllerNode(Node):
    def __init__(self):
        super().__init__('mass_flow_controller_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Update with your port
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.overheating_flag = False  # Flag to track overheating

        # Initialize the mass flow controller
        self.initialize_controller()

        # Subscriber for setting the flow rate
        self.subscription = self.create_subscription(
            Float32,
            'set_flow_rate',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher for outputting measured flow rate
        self.flow_rate_publisher = self.create_publisher(Float32, 'measured_flow_rate', 10)

        # Timer for publishing measured flow rate
        self.timer = self.create_timer(0.5, self.publish_measured_flow_rate)

        self.get_logger().info("Mass flow controller node initialized and ready to operate.")

    def initialize_controller(self):
        """Initialize or reset the mass flow controller."""
        try:
            self.port = ShdlcSerialPort(port=self.serial_port, baudrate=115200)
            self.channel = ShdlcChannel(self.port)
            self.sensor = Sfx6xxxDevice(self.channel)
            self.sensor.device_reset()
            self.get_logger().info("Mass flow controller initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize the mass flow controller: {e}")
            raise

    def listener_callback(self, msg):
        flow_rate = msg.data
        self.get_logger().info(f"Received flow rate command: {flow_rate} L/min")

        if 0.0 <= flow_rate <= 20.0:
            try:
                self.sensor.set_setpoint(flow_rate)
                self.overheating_flag = False  # Reset flag if successful
                self.get_logger().info(f"Set flow rate to {flow_rate} L/min")
            except ShdlcDeviceError as e:
                self.handle_overheating_error(e)
            except Exception as e:
                self.get_logger().error(f"Failed to set flow rate: {e}")
        else:
            self.get_logger().error("Flow rate out of range (0-20 L/min). Command ignored.")

    def publish_measured_flow_rate(self):
        try:
            measured_value = self.sensor.read_averaged_measured_value(50)
            self.flow_rate_publisher.publish(Float32(data=measured_value))
        except ShdlcDeviceError as e:
            self.handle_overheating_error(e)
        except Exception as e:
            self.get_logger().error(f"Failed to read measured flow rate: {e}")

    def handle_overheating_error(self, error):
        if error.error_code == StatusCode.SENSOR_MEASURE_LOOP_NOT_RUNNING_ERROR.value:
            if not self.overheating_flag:  # Handle overheating only once
                self.get_logger().error("Valve was closed due to overheating protection. Resetting the controller.")
                self.reset_controller()
        else:
            self.get_logger().error(f"Device error: {error}")

    def reset_controller(self):
        """Reset the mass flow controller after an overheating error."""
        try:
            self.sensor.close_valve()
            self.port.close()
            self.get_logger().info("Closed valve and communication port. Reinitializing the controller...")
            self.initialize_controller()
            self.overheating_flag = True  # Prevent repeated resets
        except Exception as e:
            self.get_logger().error(f"Failed to reset the controller: {e}")

    def destroy_node(self):
        try:
            self.sensor.close_valve()
            self.port.close()
            self.get_logger().info("Mass flow controller shut down successfully.")
        except Exception as e:
            self.get_logger().error(f"Error shutting down mass flow controller: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MassFlowControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
