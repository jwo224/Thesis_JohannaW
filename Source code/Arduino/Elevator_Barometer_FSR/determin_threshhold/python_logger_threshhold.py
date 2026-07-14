import serial
import csv
from datetime import datetime
from pathlib import Path
import threading

# =========================
# SETTINGS
# =========================

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

script_folder = Path(__file__).parent

output_file = script_folder / (
    "adaptive_button_test_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
)

# =========================
# SERIAL SETUP
# =========================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print(f"Connected to {SERIAL_PORT}")
print(f"Saving CSV to:")
print(output_file)
print()
print("Press ENTER to start the test.")
print("When asked if the press was successful, type y or n and press ENTER.")
print("Press Ctrl+C to stop.")
print()

running = True


def keyboard_thread():
    global running

    while running:
        user_input = input()

        if user_input.strip().lower() in ["y", "n"]:
            ser.write(user_input.strip().lower().encode())
            print(f"> Sent answer: {user_input.strip().lower()}")
        else:
            ser.write(b"s")
            print("> Sent start command")


thread = threading.Thread(target=keyboard_thread, daemon=True)
thread.start()

with open(output_file, mode="w", newline="") as file:
    writer = csv.writer(file)

    # Write clean CSV header
    writer.writerow([
        "trial",
        "time_ms",
        "phase",
        "threshold",
        "angle",
        "fsr_raw",
        "fsr_voltage"
    ])

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print(line)

            # Ignore ESP32 comment/status lines and header lines
            if line.startswith("#"):
                continue

            if line.startswith("trial,time_ms"):
                continue

            parts = line.split(",")

            if len(parts) == 7:
                writer.writerow(parts)
                file.flush()

    except KeyboardInterrupt:
        running = False
        print("\nStopped logging.")

ser.close()