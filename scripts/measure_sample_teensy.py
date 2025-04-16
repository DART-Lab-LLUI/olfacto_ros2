import serial
import time

ser = serial.Serial('/dev/ttyACM0', 500000)
count = 0

# For Teensy timestamp rate check
last_teensy_timestamp = None
teensy_deltas = []

# For Python receive rate check
last_python_time = time.time()
python_deltas = []

while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        parts = line.split(",")
        if len(parts) != 2:
            continue

        # Parse values
        teensy_timestamp = int(parts[0])  # in microseconds
        value = int(parts[1])

        # Rate based on Teensy timestamp
        if last_teensy_timestamp is not None:
            delta_teensy = (teensy_timestamp - last_teensy_timestamp) / 1e6  # convert to seconds
            teensy_deltas.append(delta_teensy)
        last_teensy_timestamp = teensy_timestamp

        # Rate based on receive time
        current_python_time = time.time()
        delta_python = current_python_time - last_python_time
        python_deltas.append(delta_python)
        last_python_time = current_python_time

        count += 1
        if count == 1000:
            avg_teensy = 1 / (sum(teensy_deltas) / len(teensy_deltas))
            avg_python = 1 / (sum(python_deltas) / len(python_deltas))

            print(f"Rate from Teensy timestamps: {avg_teensy:.2f} Hz")
            print(f"Rate from Python receive timing: {avg_python:.2f} Hz")
            print("----")

            count = 0
            teensy_deltas.clear()
            python_deltas.clear()

    except Exception as e:
        print(f"Error: {e}")
