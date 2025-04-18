import csv
import time
import requests

# Configuration
PI_ADDRESS = "http://olfactopi.local:8000/stimulus"
CSV_FILE = "programs/new.csv"

def run_schedule():
    with open(CSV_FILE, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            try:
                # Strip spaces in case of badly formatted CSV
                valve = row['Valve'].strip()
                ratio = row['ratio'].strip()
                duration = row['duration'].strip()
                total_flow = row['total_flow'].strip()

                # If duration is missing or empty, skip the row
                if not duration:
                    print(f"Skipping row with missing duration: {row}")
                    continue

                duration = float(duration)

                # If valve, ratio, and total_flow are blank, it's a wait-only row
                if valve == "" and ratio == "" and total_flow == "":
                    print(f"Waiting for {duration} seconds (no stimulus)...")
                    time.sleep(duration)
                    continue

                # Otherwise, send stimulus
                payload = {
                    "valve": int(valve),
                    "ratio": float(ratio),
                    "duration": duration,
                    "total_flow": float(total_flow)
                }

                print(f"Sending: {payload}")
                response = requests.post(PI_ADDRESS, json=payload)
                if response.status_code == 200:
                    print(f"Success: {payload}")
                else:
                    print(f"Failed to send stimulus: {response.status_code}, {response.text}")

                # Wait for the stimulus duration
                time.sleep(duration)

            except Exception as e:
                print(f"Error processing row {row}: {e}")

if __name__ == "__main__":
    run_schedule()
