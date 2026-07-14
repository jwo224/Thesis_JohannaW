#!/usr/bin/env python3
"""Capture labeled snapshots from every visible /dev/video* camera."""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import time
from pathlib import Path

import cv2


def symlink_targets(directory: str) -> dict[str, list[str]]:
    targets: dict[str, list[str]] = {}
    for link in glob.glob(os.path.join(directory, "*")):
        try:
            target = os.path.realpath(link)
        except OSError:
            continue
        targets.setdefault(target, []).append(link)
    return targets


def udev_name(device: str) -> str:
    try:
        output = subprocess.check_output(
            ["udevadm", "info", "--query=property", "--name", device],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        return ""

    useful_keys = (
        "ID_MODEL=",
        "ID_SERIAL_SHORT=",
        "ID_SERIAL=",
        "ID_PATH=",
        "ID_USB_INTERFACE_NUM=",
    )
    lines = [line for line in output.splitlines() if line.startswith(useful_keys)]
    return ", ".join(lines)


def video_devices() -> list[str]:
    return sorted(glob.glob("/dev/video*"), key=lambda path: int(path.removeprefix("/dev/video")))


def annotate(frame, lines: list[str]):
    y = 32
    for line in lines:
        cv2.putText(
            frame,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 32


def capture_snapshot(device: str, output_dir: Path, warmup: float) -> bool:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"{device}: could not open")
        return False

    start = time.time()
    frame = None
    while time.time() - start < warmup:
        ok, candidate = cap.read()
        if ok:
            frame = candidate
    ok, candidate = cap.read()
    cap.release()

    if ok:
        frame = candidate
    if frame is None:
        print(f"{device}: opened, but no frame received")
        return False

    basename = device.replace("/dev/", "")
    annotate(frame, [device, time.strftime("%Y-%m-%d %H:%M:%S")])
    path = output_dir / f"{basename}.jpg"
    cv2.imwrite(str(path), frame)
    print(f"{device}: wrote {path}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Grab one labeled image from every /dev/video* camera."
    )
    parser.add_argument(
        "--output",
        default="camera_identification",
        help="Directory where snapshots are written.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="Seconds to wait while each camera auto-exposure settles.",
    )
    args = parser.parse_args()

    devices = video_devices()
    if not devices:
        print("No /dev/video* devices found.")
        return 1

    by_id = symlink_targets("/dev/v4l/by-id")
    by_path = symlink_targets("/dev/v4l/by-path")

    print("Visible video devices:")
    for device in devices:
        real = os.path.realpath(device)
        aliases = by_id.get(real, []) + by_path.get(real, [])
        print(f"  {device}")
        for alias in aliases:
            print(f"    alias: {alias}")
        info = udev_name(device)
        if info:
            print(f"    udev: {info}")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("\nCapturing snapshots. Cover or wave at one physical camera, rerun, and compare images.")
    success = False
    for device in devices:
        success = capture_snapshot(device, output_dir, args.warmup) or success
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
