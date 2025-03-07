#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# (c) Copyright 2023 Sensirion AG, Switzerland
#
#     THIS FILE IS AUTOMATICALLY GENERATED!
#
# Generator:     sensirion-driver-generator 0.23.0
# Product:       sfx6xxx
# Model-Version: 2.0.0
#

import time
import argparse
from sensirion_shdlc_driver import ShdlcSerialPort
from sensirion_shdlc_driver.errors import ShdlcDeviceError
from sensirion_driver_adapters.shdlc_adapter.shdlc_channel import ShdlcChannel
from sensirion_uart_sfx6xxx.device import Sfx6xxxDevice
from sensirion_uart_sfx6xxx.commands import StatusCode

parser = argparse.ArgumentParser()
parser.add_argument('--serial-port', '-p', default='/dev/ttyUSB0')
args = parser.parse_args()

with ShdlcSerialPort(port=args.serial_port, baudrate=115200) as port:
    sensor = Sfx6xxxDevice(ShdlcChannel(port, shdlc_address=1))
    sensor.device_reset()
    time.sleep(2.0)
    serial_number = sensor.get_serial_number()
    print(f"serial_number: {serial_number}; ")
    sensor.set_setpoint(0)
    for i in range(200):
        try:
            averaged_measured_value = sensor.read_averaged_measured_value(50)
            print(f"averaged_measured_value: {averaged_measured_value}; ")
        except ShdlcDeviceError as e:
            if e.error_code == StatusCode.SENSOR_MEASURE_LOOP_NOT_RUNNING_ERROR.value:
                print("Most likely the valve was closed due to overheating "
                      "protection.\nMake sure a flow is applied and start the "
                      "script again.")
                break
        except BaseException:
            continue
    sensor.close_valve()
