from sensirion_shdlc_driver import ShdlcSerialPort
from sensirion_driver_adapters.shdlc_adapter.shdlc_channel import ShdlcChannel
from sensirion_uart_sfx6xxx.device import Sfx6xxxDevice

with ShdlcSerialPort(port='/dev/ttyUSB0', baudrate=115200) as port:
    sensor1 = Sfx6xxxDevice(ShdlcChannel(port, shdlc_address=1))
    sensor2 = Sfx6xxxDevice(ShdlcChannel(port, shdlc_address=2))
    print(f"serial_number: {int(sensor1.get_serial_number(), 16)}; ")
    print(f"serial_number: {int(sensor2.get_serial_number(), 16)}; ")
    print(f"measured_value: {sensor1.read_measured_value()}; ")
    print(f"measured_value: {sensor2.read_measured_value()}; ")
