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
        self.declare_parameter('serial_port', '/dev/mfc')  # Set correct port
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.overheating_flags = {0: False, 1: False, 2:False}  # Track overheating per MFC

        # Initialize Mass Flow Controllers
        self.initialize_controllers()

        # Create subscribers for each MFC
        for i in range(3):
            topic_name = f'mfc{i}/set_flow_rate'
            self.create_subscription(Float32, topic_name, lambda msg, index=i: self.listener_callback(msg, index), 10)

        # Create publishers for each MFC
        self.flow_rate_publishers = {
            i: self.create_publisher(Float32, f'mfc{i}/measured_flow_rate', 10) for i in range(3)
        }

        # Timer to periodically read and publish flow rates
        self.timer = self.create_timer(0.01, self.publish_measured_flow_rates)

        self.get_logger().info("Mass flow controller node initialized and ready to operate.")

    def initialize_controllers(self):
        """Initialize the mass flow controllers."""
        try:
            self.port = ShdlcSerialPort(port=self.serial_port, baudrate=115200)
            self.sensors = {
                i: Sfx6xxxDevice(ShdlcChannel(self.port, shdlc_address=i)) for i in range(3)
            }
            for i in range(3):
                self.sensors[i].device_reset()
            self.get_logger().info("All Mass Flow Controllers initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Mass Flow Controllers: {e}")
            raise

    def listener_callback(self, msg, mfc_index):
        """Handles incoming flow rate commands for each MFC."""
        flow_rate = msg.data
        self.get_logger().info(f"Received flow rate command for MFC {mfc_index}: {flow_rate} L/min")

        if 0.0 <= flow_rate <= 20.0:
            try:
                self.sensors[mfc_index].set_setpoint(flow_rate)
                self.overheating_flags[mfc_index] = False  # Reset overheating flag if successful
                self.get_logger().info(f"Set flow rate for MFC {mfc_index} to {flow_rate} L/min")
            except ShdlcDeviceError as e:
                self.handle_overheating_error(e, mfc_index)
            except Exception as e:
                self.get_logger().error(f"Failed to set flow rate for MFC {mfc_index}: {e}")
        else:
            self.get_logger().error(f"Flow rate out of range (0-20 L/min) for MFC {mfc_index}. Command ignored.")

    def publish_measured_flow_rates(self):
        """Publishes the measured flow rates of all MFCs."""
        for i in range(3):
            try:
                #measured_value = self.sensors[i].read_averaged_measured_value(50)
                measured_value = self.sensors[i].read_measured_value()
                self.flow_rate_publishers[i].publish(Float32(data=measured_value))
            except ShdlcDeviceError as e:
                self.handle_overheating_error(e, i)
            except Exception as e:
                self.get_logger().error(f"Failed to read measured flow rate for MFC {i}: {e}")

    def handle_overheating_error(self, error, mfc_index):
        """Handles overheating errors for a specific MFC."""
        if error.error_code == StatusCode.SENSOR_MEASURE_LOOP_NOT_RUNNING_ERROR.value:
            if not self.overheating_flags[mfc_index]:  # Handle overheating only once
                self.get_logger().error(f"MFC {mfc_index}: Valve closed due to overheating. Resetting controller.")
                self.reset_controller(mfc_index)
        else:
            self.get_logger().error(f"Device error for MFC {mfc_index}: {error}")

    def reset_controller(self, mfc_index):
        """Resets a specific MFC after an overheating error."""
        try:
            self.sensors[mfc_index].close_valve()
            self.get_logger().info(f"Closed valve for MFC {mfc_index}. Reinitializing...")
            self.sensors[mfc_index] = Sfx6xxxDevice(ShdlcChannel(self.port, shdlc_address=mfc_index))
            self.sensors[mfc_index].device_reset()
            self.overheating_flags[mfc_index] = True  # Prevent repeated resets
            self.get_logger().info(f"MFC {mfc_index} reinitialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset MFC {mfc_index}: {e}")

    def destroy_node(self):
        """Shutdown sequence for all MFCs."""
        try:
            for i in range(3):
                self.sensors[i].close_valve()
            self.port.close()
            self.get_logger().info("All Mass Flow Controllers shut down successfully.")
        except Exception as e:
            self.get_logger().error(f"Error shutting down controllers: {e}")
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
