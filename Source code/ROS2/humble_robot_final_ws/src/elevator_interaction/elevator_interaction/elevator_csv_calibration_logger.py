#!/usr/bin/env python3

import csv
import os
import queue
import re
import statistics
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
DEFAULT_CALIBRATION_CSV = os.path.expanduser(
    "~/gr4_ws/elevator_logs/calibration.csv"
)
DEFAULT_LOG_DIR = os.path.expanduser("~/gr4_ws/elevator_logs")

CSV_START = "========== CSV START =========="
CSV_END = "========== CSV END =========="
RECALIBRATION_TIMEOUT_SECONDS = 30.0
RECALIBRATION_DISCARD_SAMPLES = 2
RECALIBRATION_SAMPLE_COUNT = 7
RECALIBRATION_MAX_SPREAD_PA = 8.0
MIN_REASONABLE_PRESSURE_PA = 20000.0
MAX_REASONABLE_PRESSURE_PA = 120000.0

PRESSURE_PATTERNS = (
    re.compile(
        r"\bpressure(?:_pa)?\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*(?:pa)?\b",
        re.IGNORECASE,
    ),
    re.compile(
        r"\bp\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*pa\b",
        re.IGNORECASE,
    ),
)
RECALIBRATE_COMMAND_PATTERN = re.compile(
    r"^recalibrate_known_floor(?:\s+(-?\d+))?$",
    re.IGNORECASE,
)


class ElevatorCSVCalibrationLogger(Node):
    def __init__(self):
        super().__init__("elevator_csv_calibration_logger")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", 115200)
        self.declare_parameter("calibration_csv", DEFAULT_CALIBRATION_CSV)
        self.declare_parameter("log_dir", DEFAULT_LOG_DIR)
        self.declare_parameter("auto_load", True)
        self.declare_parameter(
            "recalibration_discard_samples",
            RECALIBRATION_DISCARD_SAMPLES,
        )
        self.declare_parameter(
            "recalibration_sample_count",
            RECALIBRATION_SAMPLE_COUNT,
        )
        self.declare_parameter(
            "recalibration_max_spread_pa",
            RECALIBRATION_MAX_SPREAD_PA,
        )
        self.declare_parameter(
            "recalibration_timeout_seconds",
            RECALIBRATION_TIMEOUT_SECONDS,
        )

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.calibration_csv = self.get_parameter("calibration_csv").value
        self.log_dir = self.get_parameter("log_dir").value
        self.auto_load = bool(self.get_parameter("auto_load").value)
        self.recalibration_discard_samples = max(
            0,
            int(self.get_parameter("recalibration_discard_samples").value),
        )
        self.recalibration_sample_count = max(
            3,
            int(self.get_parameter("recalibration_sample_count").value),
        )
        self.recalibration_max_spread_pa = max(
            0.1,
            float(self.get_parameter("recalibration_max_spread_pa").value),
        )
        self.recalibration_timeout_seconds = max(
            5.0,
            float(self.get_parameter("recalibration_timeout_seconds").value),
        )

        os.makedirs(self.log_dir, exist_ok=True)

        self.ser = None
        self.running = False
        self.reader_thread = None
        self.q = queue.Queue()

        self.capturing_csv = False
        self.csv_lines = []

        # This remains the unshifted calibration loaded from calibration.csv.
        self.original_calibration = {}
        self.esp32_mode = None
        self.recalibration_pending = False
        self.recalibration_requested_at = None
        self.recalibration_mode_switch_sent = False
        self.recalibration_discarded = 0
        self.recalibration_samples = []
        self.recalibration_reference_floor = None

        self.rx_pub = self.create_publisher(String, "esp32_rx", 50)
        self.csv_saved_pub = self.create_publisher(String, "csv_saved", 10)
        self.status_pub = self.create_publisher(
            String,
            "elevator_logger_status",
            10,
        )

        self.command_sub = self.create_subscription(
            String,
            "esp32_command",
            self.command_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.process_lines)
        self.recalibration_timer = self.create_timer(
            0.2,
            self.check_recalibration_timeout,
        )

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

            self.get_logger().info(
                f"Connected to ESP32 on {self.port} at {self.baud}"
            )
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
                    self.q.put((time.monotonic(), line))

            except Exception as e:
                self.q.put((time.monotonic(), f"[SERIAL_ERROR] {e}"))
                time.sleep(0.2)

    def send_command(self, cmd):
        cmd = str(cmd).strip()

        if not cmd:
            return False

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial port not open.")
            self.publish_status("serial_error:port_not_open")
            return False

        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.ser.flush()
            self.get_logger().info(f"sent: {cmd}")
            return True
        except Exception as e:
            self.get_logger().error(f"write failed: {e}")
            self.publish_status(f"serial_write_error:{e}")
            return False

    def command_callback(self, msg):
        cmd = msg.data.strip()

        if cmd == "load_csv_calibration":
            self.load_calibration_sequence()
        else:
            recalibrate_match = RECALIBRATE_COMMAND_PATTERN.fullmatch(cmd)
            if recalibrate_match:
                floor_text = recalibrate_match.group(1)
                if floor_text is None:
                    self.get_logger().error(
                        "Missing known floor number. Use: "
                        "recalibrate_known_floor FLOOR"
                    )
                    self.publish_status(
                        "recalibration_error:missing_floor"
                    )
                    return

                self.start_known_floor_recalibration(int(floor_text))
            else:
                self.send_command(cmd)

    # ------------------------------------------------------------
    # Calibration CSV loading
    # ------------------------------------------------------------

    def read_calibration_csv(self):
        if not os.path.exists(self.calibration_csv):
            raise FileNotFoundError(
                f"Calibration CSV not found: {self.calibration_csv}"
            )

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
            raise ValueError(
                "Calibration CSV must contain at least floor0 "
                "and one other floor."
            )

        return dict(sorted(calibration.items()))

    def load_calibration_sequence(self):
        try:
            calibration = self.read_calibration_csv()
        except Exception as e:
            self.get_logger().error(f"Could not load calibration CSV: {e}")
            self.publish_status(f"calibration_load_error:{e}")
            return

        # Keep an immutable baseline in memory. Recalibration starts here.
        self.original_calibration = calibration.copy()

        self.get_logger().info("Loading calibration from CSV:")
        for floor, pressure in calibration.items():
            self.get_logger().info(f"  floor {floor}: {pressure:.2f} Pa")

        if not self.send_command("baro_on"):
            self.publish_status("calibration_load_error:serial_not_available")
            return
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
    # Known-floor recalibration
    # ------------------------------------------------------------

    def start_known_floor_recalibration(self, reference_floor):
        if self.recalibration_pending:
            self.get_logger().warn(
                "Known-floor recalibration is already waiting for pressure"
            )
            self.publish_status("recalibration_already_pending")
            return

        if not self.original_calibration:
            try:
                self.original_calibration = self.read_calibration_csv()
            except Exception as e:
                self.get_logger().error(
                    f"Could not prepare known-floor recalibration: {e}"
                )
                self.publish_status(f"recalibration_error:{e}")
                return

        if reference_floor not in self.original_calibration:
            self.get_logger().error(
                f"Cannot recalibrate at floor {reference_floor}: "
                "floor is not present in calibration.csv"
            )
            self.publish_status(
                "recalibration_error:"
                f"unknown_floor={reference_floor}"
            )
            return

        self.recalibration_pending = True
        self.recalibration_requested_at = time.monotonic()
        self.recalibration_mode_switch_sent = False
        self.recalibration_discarded = 0
        self.recalibration_samples = []
        self.recalibration_reference_floor = reference_floor

        old_reference_pressure = self.original_calibration[reference_floor]
        self.get_logger().info(
            f"recalibration reference floor: {reference_floor}"
        )
        self.get_logger().info(
            "recalibration original reference pressure: "
            f"{old_reference_pressure:.2f} Pa"
        )
        self.publish_status(
            "recalibration_waiting_for_pressure:"
            f"floor={reference_floor}"
        )

        if self.esp32_mode == "barometer":
            self.get_logger().info(
                "recalibration waiting for fresh stable pressure readings"
            )
        elif self.esp32_mode == "robot":
            self.enter_barometer_for_recalibration()
        elif not self.send_command("mode"):
            self.cancel_recalibration("serial_not_available")
        else:
            self.get_logger().info(
                "recalibration checking ESP32 mode before measuring pressure"
            )

    def enter_barometer_for_recalibration(self):
        if self.recalibration_mode_switch_sent:
            return

        self.recalibration_mode_switch_sent = True
        self.get_logger().info(
            "recalibration entering barometer mode for pressure readings"
        )

        if not self.send_command("baro_on"):
            self.cancel_recalibration("serial_not_available")

    def parse_live_pressure(self, line):
        try:
            fields = next(csv.reader([line]))
        except (csv.Error, StopIteration):
            fields = []

        # Normal live CSV-format rows use pressure_Pa as the fourth field.
        live_types = {"read", "live", "measurement"}
        if fields and fields[0].strip().lower() in live_types:
            if len(fields) > 3:
                pressure = self.valid_pressure(fields[3])
                if pressure is not None:
                    return pressure

        # Also accept human-readable firmware output such as
        # "pressure_Pa: 101325.0" or "P=101325.0 Pa".
        for pattern in PRESSURE_PATTERNS:
            match = pattern.search(line)
            if match:
                pressure = self.valid_pressure(match.group(1))
                if pressure is not None:
                    return pressure

        return None

    @staticmethod
    def valid_pressure(value):
        try:
            pressure = float(str(value).strip())
        except (TypeError, ValueError):
            return None

        if (
            MIN_REASONABLE_PRESSURE_PA
            <= pressure
            <= MAX_REASONABLE_PRESSURE_PA
        ):
            return pressure

        return None

    def collect_recalibration_pressure(self, pressure):
        if (
            self.recalibration_discarded
            < self.recalibration_discard_samples
        ):
            self.recalibration_discarded += 1
            self.get_logger().info(
                "recalibration discarded settling sample "
                f"{self.recalibration_discarded}/"
                f"{self.recalibration_discard_samples}: "
                f"{pressure:.2f} Pa"
            )
            self.publish_status(
                "recalibration_discarding_settling_samples:"
                f"{self.recalibration_discarded}/"
                f"{self.recalibration_discard_samples}"
            )
            return

        self.recalibration_samples.append(pressure)
        if (
            len(self.recalibration_samples)
            > self.recalibration_sample_count
        ):
            self.recalibration_samples.pop(0)

        sample_total = len(self.recalibration_samples)
        if sample_total < self.recalibration_sample_count:
            self.get_logger().info(
                "recalibration stable sample "
                f"{sample_total}/{self.recalibration_sample_count}: "
                f"{pressure:.2f} Pa"
            )
            self.publish_status(
                f"recalibration_sampling:{sample_total}/"
                f"{self.recalibration_sample_count}"
            )
            return

        pressure_spread = (
            max(self.recalibration_samples)
            - min(self.recalibration_samples)
        )
        if pressure_spread > self.recalibration_max_spread_pa:
            self.get_logger().warn(
                "recalibration pressure is not stable yet: "
                f"spread={pressure_spread:.2f} Pa, "
                f"limit={self.recalibration_max_spread_pa:.2f} Pa"
            )
            self.publish_status(
                "recalibration_waiting_for_stable_pressure:"
                f"spread_pa={pressure_spread:.2f}"
            )
            return

        current_reference_pressure = statistics.median(
            self.recalibration_samples
        )
        self.get_logger().info(
            "recalibration accepted median pressure "
            f"{current_reference_pressure:.2f} Pa from "
            f"{self.recalibration_sample_count} samples "
            f"(spread {pressure_spread:.2f} Pa)"
        )
        self.apply_known_floor_recalibration(current_reference_pressure)

    def apply_known_floor_recalibration(self, current_reference_pressure):
        reference_floor = self.recalibration_reference_floor
        if reference_floor not in self.original_calibration:
            self.cancel_recalibration("reference_floor_unavailable")
            return

        self.recalibration_pending = False
        self.recalibration_requested_at = None
        self.recalibration_mode_switch_sent = False
        self.recalibration_discarded = 0
        self.recalibration_samples = []

        old_reference_pressure = self.original_calibration[reference_floor]
        pressure_shift = (
            current_reference_pressure - old_reference_pressure
        )
        adjusted_calibration = {
            floor: pressure + pressure_shift
            for floor, pressure in self.original_calibration.items()
        }

        self.get_logger().info(
            f"recalibration reference floor: {reference_floor}"
        )
        self.get_logger().info(
            "recalibration original reference pressure: "
            f"{old_reference_pressure:.2f} Pa"
        )
        self.get_logger().info(
            "recalibration current reference pressure: "
            f"{current_reference_pressure:.2f} Pa"
        )
        self.get_logger().info(
            f"recalibration pressure shift: {pressure_shift:+.2f} Pa"
        )

        for floor, pressure in adjusted_calibration.items():
            self.get_logger().info(
                f"recalibration adjusted floor {floor}: "
                f"{pressure:.2f} Pa"
            )

        self.send_command("resetcal")
        time.sleep(0.2)

        for floor, pressure in adjusted_calibration.items():
            self.send_command(f"setcal {floor} {pressure:.2f}")
            time.sleep(0.15)

        self.send_command("overview")
        time.sleep(0.2)
        self.send_command("end")

        self.publish_status(
            "recalibration_applied:"
            f"reference_floor={reference_floor},"
            f"shift_pa={pressure_shift:+.2f}"
        )
        self.recalibration_reference_floor = None

    def check_recalibration_timeout(self):
        if (
            not self.recalibration_pending
            or self.recalibration_requested_at is None
        ):
            return

        elapsed = time.monotonic() - self.recalibration_requested_at
        if elapsed >= self.recalibration_timeout_seconds:
            self.cancel_recalibration("pressure_timeout")

    def cancel_recalibration(self, reason):
        self.recalibration_pending = False
        self.recalibration_requested_at = None
        self.recalibration_mode_switch_sent = False
        self.recalibration_discarded = 0
        self.recalibration_samples = []
        self.recalibration_reference_floor = None
        self.get_logger().error(f"Known-floor recalibration failed: {reason}")
        self.publish_status(f"recalibration_error:{reason}")

    # ------------------------------------------------------------
    # Receiving and CSV saving
    # ------------------------------------------------------------

    def process_lines(self):
        while True:
            try:
                received_at, line = self.q.get_nowait()
            except queue.Empty:
                break

            self.handle_line(line, received_at)

    def handle_line(self, line, received_at=None):
        self.get_logger().info(f"esp32: {line}")

        msg = String()
        msg.data = line
        self.rx_pub.publish(msg)

        normalized_line = line.strip().lower()

        if normalized_line == "mode robot":
            self.esp32_mode = "robot"
            if self.recalibration_pending:
                self.enter_barometer_for_recalibration()
        elif normalized_line == "mode barometer":
            self.esp32_mode = "barometer"
            if self.recalibration_pending:
                self.get_logger().info(
                    "recalibration waiting for fresh stable "
                    "pressure readings"
                )

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
            return

        if self.recalibration_pending:
            if (
                received_at is not None
                and self.recalibration_requested_at is not None
                and received_at < self.recalibration_requested_at
            ):
                return

            pressure = self.parse_live_pressure(line)
            if pressure is not None:
                self.collect_recalibration_pressure(pressure)

    def save_csv(self):
        if not self.csv_lines:
            self.get_logger().warn(
                "CSV END received, but no CSV lines captured"
            )
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"loaded_calibration_test_{stamp}.csv"
        path = os.path.join(self.log_dir, filename)

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
