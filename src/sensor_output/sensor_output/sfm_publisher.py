import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
from sensirion_i2c_driver import I2cConnection, CrcCalculator
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_sensorbridge import (
    SensorBridgePort, SensorBridgeShdlcDevice, SensorBridgeI2cProxy
)
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sfm_sf06.device import SfmSf06Device


class SFMPublisher(Node):
    def __init__(self):
        super().__init__('sfm_publisher')

        self.publisher_flow = self.create_publisher(Float32, 'sfm_flow_output', 10)
        self.publisher_temp = self.create_publisher(Float32, 'sfm_temp_output', 10)

        self.get_logger().info("Connecting to SensorBridge and SFM4300...")

        # Setup SensorBridge connection
        self.serial_port = ShdlcSerialPort(port='/dev/ttyUSB1', baudrate=460800)
        self.shdlc_connection = ShdlcConnection(self.serial_port)
        self.bridge = SensorBridgeShdlcDevice(self.shdlc_connection, slave_address=0)

        self.bridge.set_i2c_frequency(SensorBridgePort.ONE, frequency=100e3)
        self.bridge.set_supply_voltage(SensorBridgePort.ONE, voltage=3.3)
        self.bridge.switch_supply_on(SensorBridgePort.ONE)

        i2c_proxy = SensorBridgeI2cProxy(self.bridge, port=SensorBridgePort.ONE)
        channel = I2cChannel(
            I2cConnection(i2c_proxy),
            slave_address=0x2A,
            crc=CrcCalculator(8, 0x31, 0xFF, 0x00)
        )
        self.sensor = SfmSf06Device(channel)

        # Stop any running measurements first
        try:
            self.sensor.stop_continuous_measurement()
            time.sleep(0.1)
        except Exception:
            pass

        # Start air-calibrated measurement
        self.sensor.start_air_continuous_measurement()
        self.get_logger().info("Started air continuous measurement.")

        # Create timer with very fast repeat rate (almost free-running)
        self.timer = self.create_timer(0.0, self.publish_sensor_data)

    def publish_sensor_data(self):
        try:
            flow, temp, status = self.sensor.read_measurement_data()

            msg_flow = Float32()
            msg_flow.data = flow.value
            self.publisher_flow.publish(msg_flow)

            msg_temp = Float32()
            msg_temp.data = temp.value
            self.publisher_temp.publish(msg_temp)

        except Exception as e:
            self.get_logger().warn(f"Sensor read failed: {e}")

    def destroy_node(self):
        # Cleanup before shutting down
        self.get_logger().info("Stopping sensor measurement...")
        try:
            self.sensor.stop_continuous_measurement()
        except Exception:
            pass
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SFMPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

