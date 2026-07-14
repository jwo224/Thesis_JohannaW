import serial
import csv
from datetime import datetime

from pathlib import Path
import threading
import time

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

# Save CSV in the same folder as this Python file
SCRIPT_FOLDER = Path(__file__).parent
filename = SCRIPT_FOLDER / ("fsr_servo_measurement_link30_20deg_elevator"
"" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print(f"Connected to {SERIAL_PORT}")
print(f"Saving data to:")
print(filename)
print()
print("Press ENTER to start a new measurement.")
print("Press Ctrl+C to stop.")
print()

running = True


def keyboard_thread():
    global running

    while running:
        input()
        print("Starting new measurement...")
        ser.write(b"s")


thread = threading.Thread(target=keyboard_thread, daemon=True)
thread.start()

with open(filename, mode="w", newline="") as file:
    writer = csv.writer(file)

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print(line)

            # Ignore comment/status lines
            if line.startswith("#"):
                continue

            parts = line.split(",")

            if len(parts) == 4:
                writer.writerow(parts)
                file.flush()

    except KeyboardInterrupt:
        running = False
        print("\nStopped logging.")

ser.close()
