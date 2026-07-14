#!/usr/bin/env python3

import os
import queue
import re
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
CSV_START = "========== CSV START =========="
CSV_END = "========== CSV END =========="
FLOOR_COMMAND_PATTERN = re.compile(r"^floor\s*(-?\d+)$", re.IGNORECASE)


class ESP32SerialBridge(Node):
    def __init__(self):
        super().__init__("esp32_serial_bridge")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", 115200)
        self.declare_parameter(
            "csv_dir",
            os.path.expanduser("~/gr4_ws/elevator_logs"),
        )
        self.declare_parameter("repeat_floor_calibration", True)
        self.declare_parameter("calibration_settle_seconds", 3.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.csv_dir = self.get_parameter("csv_dir").value
        self.repeat_floor_calibration = bool(
            self.get_parameter("repeat_floor_calibration").value
        )
        self.calibration_settle_seconds = max(
            1.5,
            float(self.get_parameter("calibration_settle_seconds").value),
        )

        os.makedirs(self.csv_dir, exist_ok=True)

        self.ser = None
        self.running = False
        self.reader_thread = None
        self.q = queue.Queue()

        self.capturing_csv = False
        self.csv_lines = []
        self.pending_floor_command = None
        self.pending_floor_command_at = None

        self.rx_pub = self.create_publisher(String, "esp32_rx", 50)
        self.csv_saved_pub = self.create_publisher(String, "csv_saved", 10)

        self.command_sub = self.create_subscription(
            String,
            "esp32_command",
            self.command_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.process_lines)

        self.connect()

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
                daemon=True,
            )
            self.reader_thread.start()

            self.get_logger().info(
                f"Connected to ESP32 on {self.port} at {self.baud}"
            )

            self.send_command("mode")

        except Exception as e:
            self.get_logger().error(f"Could not open ESP32 serial port: {e}")

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

    def command_callback(self, msg):
        cmd = msg.data.strip()
        floor_match = FLOOR_COMMAND_PATTERN.fullmatch(cmd)

        if self.repeat_floor_calibration and floor_match:
            self.start_stable_floor_calibration(cmd)
            return

        self.send_command(cmd)

    def start_stable_floor_calibration(self, cmd):
        if self.pending_floor_command is not None:
            self.get_logger().warn(
                "A floor calibration is already settling. "
                "Wait for its replacement measurement."
            )
            return

        if not self.send_command(cmd):
            return

        self.pending_floor_command = cmd
        self.pending_floor_command_at = (
            time.monotonic() + self.calibration_settle_seconds
        )
        self.get_logger().info(
            f"{cmd}: warm-up measurement sent; replacement measurement "
            f"will run in {self.calibration_settle_seconds:.1f} seconds"
        )

    def send_command(self, cmd):
        cmd = str(cmd).strip()

        if not cmd:
            return False

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial port not open.")
            return False

        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.ser.flush()
            self.get_logger().info(f"sent: {cmd}")
            return True
        except Exception as e:
            self.get_logger().error(f"write failed: {e}")
            return False

    def process_lines(self):
        self.process_pending_floor_calibration()

        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break

            self.handle_line(line)

    def process_pending_floor_calibration(self):
        if (
            self.pending_floor_command is None
            or self.pending_floor_command_at is None
            or time.monotonic() < self.pending_floor_command_at
        ):
            return

        cmd = self.pending_floor_command
        self.pending_floor_command = None
        self.pending_floor_command_at = None

        self.get_logger().info(
            f"{cmd}: sending settled replacement measurement"
        )
        self.send_command(cmd)

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
            self.get_logger().warn(
                "CSV END received, but no CSV lines captured"
            )
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.csv_dir, f"floor_log_{stamp}.csv")

        with open(path, "w", encoding="utf-8") as f:
            for line in self.csv_lines:
                f.write(line + "\n")

        self.get_logger().info(f"Saved CSV: {path}")

        msg = String()
        msg.data = path
        self.csv_saved_pub.publish(msg)

    def destroy_node(self):
        self.running = False

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
