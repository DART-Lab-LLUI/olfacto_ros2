from sensirion_shdlc_driver import ShdlcSerialPort
from sensirion_driver_adapters.shdlc_adapter.shdlc_channel import ShdlcChannel
from sensirion_uart_sfx6xxx.device import Sfx6xxxDevice

valid_address = []
counter = 0

with ShdlcSerialPort(port='/dev/mfc', baudrate=115200, additional_response_time=0) as port:
    for test_address in range(0, 255):
        channel = ShdlcChannel(port, shdlc_address = test_address, channel_delay = 0.02)
        sensor = Sfx6xxxDevice(channel)
        counter += 1
        try:
            valid_address.append(sensor.get_slave_address())
        except: 
            temp = 1
            #print(e)

print(str(counter) + ' address tested')
print(str(len(valid_address)) + ' MFC found at the following adresses : ')
print(valid_address)

