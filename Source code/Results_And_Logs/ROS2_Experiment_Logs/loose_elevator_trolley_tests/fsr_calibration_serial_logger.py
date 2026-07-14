#!/usr/bin/env python3
"""Serial logger/controller for the ESP32-C3 FSR calibration firmware."""

import csv
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


# ============================== SETTINGS ==============================
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
# ======================================================================

CSV_HEADER = [
    "trial",
    "time_ms",
    "phase",
    "threshold",
    "angle",
    "fsr_raw",
    "fsr_voltage",
]


def import_serial():
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial is not installed.")
        print("Install it with:")
        print("  python -m pip install pyserial")
        raise SystemExit(1)

    return serial


def open_serial(serial_module):
    try:
        return serial_module.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=0.1,
            write_timeout=2.0,
        )
    except serial_module.SerialException as error:
        print(f"ERROR: Could not open serial port {SERIAL_PORT}:")
        print(f"  {error}")
        print()
        print("Check that:")
        print("  - SERIAL_PORT at the top of this script is correct.")
        print("  - The ESP32-C3 is connected.")
        print("  - Arduino Serial Monitor or another program is not using the port.")
        raise SystemExit(1)


def valid_data_row(line):
    """Return a parsed seven-column firmware row, or None if invalid."""
    try:
        row = next(csv.reader([line]))
    except (csv.Error, StopIteration):
        return None

    if len(row) != 7 or not row[2].strip():
        return None

    try:
        int(row[0].strip())
        int(row[1].strip())
        float(row[3].strip())
        float(row[4].strip())
        float(row[5].strip())
        float(row[6].strip())
    except ValueError:
        return None

    return [value.strip() for value in row]


def input_worker(command_queue, stop_event):
    """Read terminal commands without blocking incoming serial processing."""
    while not stop_event.is_set():
        try:
            user_input = input().strip().lower()
        except EOFError:
            return
        except KeyboardInterrupt:
            stop_event.set()
            return

        if user_input == "":
            command_queue.put(b"s")
        elif user_input in {"y", "n"}:
            command_queue.put(user_input.encode("ascii"))
        else:
            print("Input ignored. Press ENTER to start, or type y or n.")


def send_pending_commands(serial_port, command_queue):
    while True:
        try:
            command = command_queue.get_nowait()
        except queue.Empty:
            return

        try:
            serial_port.write(command)
            serial_port.flush()
            if command == b"s":
                print("[PC -> ESP32] Start command sent")
            else:
                print(f"[PC -> ESP32] {command.decode('ascii')}")
        except Exception as error:
            print(f"ERROR: Could not send command to ESP32: {error}")


def main():
    serial_module = import_serial()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    script_directory = Path(__file__).resolve().parent
    csv_path = script_directory / f"fsr_calibration_{timestamp}.csv"

    serial_port = open_serial(serial_module)
    command_queue = queue.Queue()
    stop_event = threading.Event()

    print(f"Connected to: {SERIAL_PORT} at {BAUD_RATE} baud")
    print(f"CSV will be saved to: {csv_path}")
    print()
    print("Press ENTER to start the calibration test.")
    print("Type y or n and press ENTER when the ESP32 asks a question.")
    print("Press Ctrl+C to stop and close the logger.")
    print()

    input_thread = threading.Thread(
        target=input_worker,
        args=(command_queue, stop_event),
        daemon=True,
    )
    input_thread.start()

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADER)
            csv_file.flush()

            while not stop_event.is_set():
                send_pending_commands(serial_port, command_queue)

                try:
                    raw_line = serial_port.readline()
                except serial_module.SerialException as error:
                    print(f"ERROR: Serial connection failed: {error}")
                    break

                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Every incoming non-empty line is shown to the user.
                print(line)

                if line.startswith("#"):
                    continue

                if line.replace(" ", "") == ",".join(CSV_HEADER):
                    continue

                row = valid_data_row(line)
                if row is None:
                    print("[LOGGER] Skipped malformed/non-data line")
                    continue

                writer.writerow(row)
                csv_file.flush()

    except KeyboardInterrupt:
        print("\nStopping logger...")
    finally:
        stop_event.set()

        try:
            if serial_port.is_open:
                serial_port.close()
        except Exception:
            pass

        # Give terminal output a moment to finish cleanly on slower systems.
        time.sleep(0.05)
        print(f"Serial port closed. CSV saved to: {csv_path}")


if __name__ == "__main__":
    main()
