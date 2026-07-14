#!/usr/bin/env python3

import csv
import os
import queue
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


DEFAULT_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
DEFAULT_CALIBRATION_CSV = os.path.expanduser("~/gr4_ws/elevator_logs/calibration.csv")
DEFAULT_LOG_DIR = os.path.expanduser("~/gr4_ws/elevator_logs")

CSV_START = "========== CSV START =========="
CSV_END = "========== CSV END =========="


class ElevatorCSVCalibrationLogger(Node):
    def __init__(self):
        super().__init__("elevator_csv_calibration_logger")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", 115200)
        self.declare_parameter("calibration_csv", DEFAULT_CALIBRATION_CSV)
        self.declare_parameter("log_dir", DEFAULT_LOG_DIR)
        self.declare_parameter("auto_load", True)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.calibration_csv = self.get_parameter("calibration_csv").value
        self.log_dir = self.get_parameter("log_dir").value
        self.auto_load = bool(self.get_parameter("auto_load").value)

        os.makedirs(self.log_dir, exist_ok=True)

        self.ser = None
        self.running = False
        self.reader_thread = None
        self.q = queue.Queue()

        self.capturing_csv = False
        self.csv_lines = []

        self.rx_pub = self.create_publisher(String, "esp32_rx", 50)
        self.csv_saved_pub = self.create_publisher(String, "csv_saved", 10)
        self.status_pub = self.create_publisher(String, "elevator_logger_status", 10)

        self.command_sub = self.create_subscription(
            String,
            "esp32_command",
            self.command_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.process_lines)

        self.connect()

        if self.auto_load:
            self.load_calibration_sequence()

    # ------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------

    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1,
                write_timeout=1.0
            )

            # ESP32 often resets when serial opens
            time.sleep(2.0)
            self.ser.reset_input_buffer()

            self.running = True
            self.reader_thread = threading.Thread(
                target=self.reader_loop,
                daemon=True
            )
            self.reader_thread.start()

            self.get_logger().info(f"Connected to ESP32 on {self.port} at {self.baud}")
            self.publish_status("connected")

        except Exception as e:
            self.get_logger().error(f"Could not open ESP32 serial port: {e}")
            self.publish_status(f"serial_error:{e}")

    def reader_loop(self):
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.1)
                    continue

                raw = self.ser.readline()

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()

                if line:
                    self.q.put(line)

            except Exception as e:
                self.q.put(f"[SERIAL_ERROR] {e}")
                time.sleep(0.2)

    def send_command(self, cmd):
        cmd = str(cmd).strip()

        if not cmd:
            return

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial port not open.")
            return

        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.ser.flush()
            self.get_logger().info(f"sent: {cmd}")
        except Exception as e:
            self.get_logger().error(f"write failed: {e}")

    def command_callback(self, msg):
        cmd = msg.data.strip()

        if cmd == "load_csv_calibration":
            self.load_calibration_sequence()
        else:
            self.send_command(cmd)

    # ------------------------------------------------------------
    # Calibration CSV loading
    # ------------------------------------------------------------

    def read_calibration_csv(self):
        if not os.path.exists(self.calibration_csv):
            raise FileNotFoundError(f"Calibration CSV not found: {self.calibration_csv}")

        calibration = {}

        with open(self.calibration_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)

            required = ["type", "pressure_Pa", "calibrated_floor"]

            for col in required:
                if col not in reader.fieldnames:
                    raise ValueError(
                        f"CSV missing column '{col}'. "
                        f"Found columns: {reader.fieldnames}"
                    )

            for row in reader:
                if row.get("type", "").strip() != "calibration":
                    continue

                floor_text = row.get("calibrated_floor", "").strip()
                pressure_text = row.get("pressure_Pa", "").strip()

                if floor_text == "" or pressure_text == "":
                    continue

                floor = int(float(floor_text))
                pressure = float(pressure_text)

                # Last calibration row for each floor wins
                calibration[floor] = pressure

        if 0 not in calibration:
            raise ValueError("Calibration CSV does not contain floor0.")

        if len(calibration) < 2:
            raise ValueError("Calibration CSV must contain at least floor0 and one other floor.")

        return dict(sorted(calibration.items()))

    def load_calibration_sequence(self):
        try:
            calibration = self.read_calibration_csv()
        except Exception as e:
            self.get_logger().error(f"Could not load calibration CSV: {e}")
            self.publish_status(f"calibration_load_error:{e}")
            return

        self.get_logger().info("Loading calibration from CSV:")
        for floor, pressure in calibration.items():
            self.get_logger().info(f"  floor {floor}: {pressure:.2f} Pa")

        self.send_command("baro_on")
        time.sleep(1.0)

        self.send_command("resetcal")
        time.sleep(0.2)

        self.send_command("clearlog")
        time.sleep(0.2)

        for floor, pressure in calibration.items():
            self.send_command(f"setcal {floor} {pressure:.2f}")
            time.sleep(0.15)

        self.send_command("overview")
        time.sleep(0.2)

        self.send_command("end")

        self.publish_status("calibration_loaded_from_csv")

    # ------------------------------------------------------------
    # Receiving and CSV saving
    # ------------------------------------------------------------

    def process_lines(self):
        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break

            self.handle_line(line)

    def handle_line(self, line):
        self.get_logger().info(f"esp32: {line}")

        msg = String()
        msg.data = line
        self.rx_pub.publish(msg)

        if line == CSV_START:
            self.capturing_csv = True
            self.csv_lines = []
            self.get_logger().info("CSV capture started")
            return

        if line == CSV_END:
            self.capturing_csv = False
            self.save_csv()
            return

        if self.capturing_csv:
            if line and not line.startswith("="):
                self.csv_lines.append(line)

    def save_csv(self):
        if not self.csv_lines:
            self.get_logger().warn("CSV END received, but no CSV lines captured")
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.log_dir, f"loaded_calibration_test_{stamp}.csv")

        try:
            with open(path, "w", encoding="utf-8") as f:
                for line in self.csv_lines:
                    f.write(line + "\n")

            self.get_logger().info("")
            self.get_logger().info("========================================")
            self.get_logger().info(f"SAVED CSV: {path}")
            self.get_logger().info("========================================")
            self.get_logger().info("")

            msg = String()
            msg.data = path
            self.csv_saved_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Could not save CSV: {e}")

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorCSVCalibrationLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
