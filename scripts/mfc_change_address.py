from sensirion_shdlc_driver import ShdlcSerialPort
from sensirion_driver_adapters.shdlc_adapter.shdlc_channel import ShdlcChannel
from sensirion_uart_sfx6xxx.device import Sfx6xxxDevice

initial_address = 2
slave_address = 0

with ShdlcSerialPort(port='/dev/ttyUSB0', baudrate=115200) as port:
    channel = ShdlcChannel(port, shdlc_address=initial_address)
    sensor = Sfx6xxxDevice(channel)
    print(f"Initial slave adress: {sensor.get_slave_address()}; ")
    sensor.set_slave_address(slave_address)

    channel = ShdlcChannel(port, shdlc_address=slave_address)
    sensor = Sfx6xxxDevice(channel)
    print(f"New slave adress: {sensor.get_slave_address()}; ")
