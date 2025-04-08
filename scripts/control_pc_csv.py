import csv
import time
import requests

# Configuration
PI_ADDRESS = "http://olfactopi.local:8000/stimulus"
CSV_FILE = "programs/example_schedule.csv"

def run_schedule():
    with open(CSV_FILE, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            try:
                valve = int(row['Valve'])
                ratio = float(row['ratio'])
                duration = float(row['duration'])
                total_flow = float(row['total_flow'])

                payload = {
                    "valve": valve,
                    "ratio": ratio,
                    "duration": duration,
                    "total_flow": total_flow
                }

                print(f"Sending: {payload}")
                response = requests.post(PI_ADDRESS, json=payload)
                if response.status_code == 200:
                    print(f"Success: Valve {valve}, Ratio {ratio}, Duration {duration}s, Total Flow {total_flow}")
                else:
                    print(f"Failed to send stimulus: {response.status_code}, {response.text}")

                time.sleep(duration)

            except Exception as e:
                print(f"Error processing row {row}: {e}")

if __name__ == "__main__":
    run_schedule()
