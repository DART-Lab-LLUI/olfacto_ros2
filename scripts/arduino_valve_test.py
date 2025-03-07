import serial
import time

# Setup serial connection to Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600)  # Change this to the correct port
time.sleep(2)  # Wait for the connection to initialize

def control_valve(valve_number, state):
    """
    Control a specific valve by sending a command to the Arduino.
    Args:
        valve_number (int): The valve number to control (1-12)
        state (str): "ON" or "OFF"
    """
    if 1 <= valve_number <= 12:
        command = f'VALVE_{valve_number}_{state}\n'
        arduino.write(command.encode())
        print(f'Sent: {command.strip()}')
    else:
        print("Invalid valve number. Must be between 1 and 12.")

# Example usage
while True:
	control_valve(1, 'ON')
	control_valve(2, 'OFF')
	time.sleep(8)
	control_valve(2, 'ON')
	control_valve(1, 'OFF')
	time.sleep(2)

arduino.close()
