import csv
import re
import serial
from datetime import datetime

PORT = "COM8"
BAUD = 115200

filename = "xiao_fsr_log_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"

pattern = re.compile(
    r"angle=(?P<angle>-?\d+),\s*fsr_raw=(?P<fsr_raw>\d+),\s*fsr_voltage=(?P<fsr_voltage>[0-9.]+)"
)

with serial.Serial(PORT, BAUD, timeout=1) as ser, open(
    filename, "w", newline="", encoding="utf-8"
) as f:
    writer = csv.writer(f)

    writer.writerow([
        "timestamp",
        "angle",
        "fsr_raw",
        "fsr_voltage",
        "raw_line",
    ])

    print(f"Logging serial from {PORT} to {filename}")

    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()

        if not line:
            continue

        print(line)

        timestamp = datetime.now().isoformat(timespec="milliseconds")
        match = pattern.search(line)

        if match:
            writer.writerow([
                timestamp,
                int(match.group("angle")),
                int(match.group("fsr_raw")),
                float(match.group("fsr_voltage")),
                line,
            ])
            f.flush()