#!/usr/bin/env python3

import argparse
import csv
import math
import os
import subprocess
import time
from itertools import product
from pathlib import Path


BASE_SPEED_PARAMS = {
    "local_max_vx": 0.08,
    "local_max_vy": 0.10,
    "local_max_wz": 0.30,
    "direct_close_speed": 0.10,
    "attached_local_max_vx": 0.14,
    "attached_local_max_vy": 0.24,
    "attached_local_max_wz": 0.45,
    "return_local_max_vx": 0.18,
    "return_local_max_vy": 0.18,
    "return_local_max_wz": 0.45,
    "drive_under_max_vx": 0.05,
    "drive_under_max_vy": 0.12,
    "drive_under_max_wz": 0.16,
    "straight_under_speed": 0.16,
}

HOLONOMIC_BASE_SPEED_PARAMS = {
    "max_vx": 0.16,
    "max_vy": 0.16,
    "max_wz": 0.16,
}


def parse_float_list(text):
    values = []
    for item in text.replace(";", ",").split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    return values


def format_float(value):
    return f"{value:.4f}".rstrip("0").rstrip(".")


def make_override_string(params):
    return ";".join(f"{name}={format_float(value) if isinstance(value, float) else value}" for name, value in params.items())


def read_batch_rows(csv_path):
    if not csv_path.exists():
        return []
    with csv_path.open(newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def row_success(row):
    return str(row.get("success", "")).lower() == "true"


def average_success_duration(rows):
    durations = []
    for row in rows:
        if not row_success(row):
            continue
        value = row.get("command_to_success_sec") or row.get("duration_sec")
        try:
            durations.append(float(value))
        except (TypeError, ValueError):
            pass
    return sum(durations) / len(durations) if durations else math.inf


def summarize_failures(rows):
    reasons = {}
    for row in rows:
        if row_success(row):
            continue
        reason = row.get("failure_reason") or "unknown"
        reasons[reason] = reasons.get(reason, 0) + 1
    return ";".join(f"{reason}:{count}" for reason, count in sorted(reasons.items()))


def run_batch(args, mission_overrides, holonomic_overrides, runs, csv_filename):
    cmd = [
        "ros2",
        "launch",
        "my_robot_description",
        "mission_batch_test.launch.py",
        f"runs:={runs}",
        f"mission_step:={args.mission_step}",
        f"per_run_timeout:={args.per_run_timeout}",
        f"result_dir:={args.result_dir}",
        f"csv_filename:={csv_filename}",
        f"mission_param_overrides:={mission_overrides}",
        f"holonomic_param_overrides:={holonomic_overrides}",
    ]
    print("\n=== Running batch ===")
    print(" ".join(cmd))
    started = time.monotonic()
    completed = subprocess.run(cmd)
    return completed.returncode, time.monotonic() - started


def build_candidates(args):
    speed_scales = parse_float_list(args.speed_scales)
    pickup_aruco_clearances = parse_float_list(args.pickup_aruco_clearances)
    attached_lidar_ignore_radii = parse_float_list(args.attached_lidar_ignore_radii)
    straight_under_stop_ys = parse_float_list(args.straight_under_stop_ys)

    candidates = []
    for speed_scale, clearance, ignore_radius, stop_y in product(
        speed_scales,
        pickup_aruco_clearances,
        attached_lidar_ignore_radii,
        straight_under_stop_ys,
    ):
        params = {
            name: value * speed_scale
            for name, value in BASE_SPEED_PARAMS.items()
        }
        holonomic_params = {
            name: value * speed_scale
            for name, value in HOLONOMIC_BASE_SPEED_PARAMS.items()
        }
        params.update(
            {
                "pickup_aruco_clearance": clearance,
                "attached_lidar_ignore_radius": ignore_radius,
                "straight_under_stop_y": stop_y,
            }
        )
        candidates.append(
            {
                "speed_scale": speed_scale,
                "pickup_aruco_clearance": clearance,
                "attached_lidar_ignore_radius": ignore_radius,
                "straight_under_stop_y": stop_y,
                "mission_overrides": make_override_string(params),
                "holonomic_overrides": make_override_string(holonomic_params),
            }
        )
    return candidates


def write_summary(summary_path, rows):
    fieldnames = [
        "candidate",
        "phase",
        "runs",
        "success_count",
        "success_rate",
        "avg_success_duration_sec",
        "speed_scale",
        "pickup_aruco_clearance",
        "attached_lidar_ignore_radius",
        "straight_under_stop_y",
        "overrides",
        "holonomic_overrides",
        "failure_summary",
        "csv_filename",
        "process_returncode",
        "wall_duration_sec",
    ]
    with summary_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def choose_best(summary_rows):
    stable = [
        row for row in summary_rows
        if row["phase"] == "sweep" and int(row["success_count"]) == int(row["runs"]) and int(row["runs"]) > 0
    ]
    if not stable:
        stable = [row for row in summary_rows if row["phase"] == "sweep" and int(row["success_count"]) > 0]
    if not stable:
        return None

    def key(row):
        avg = float(row["avg_success_duration_sec"]) if row["avg_success_duration_sec"] != "inf" else math.inf
        return (
            float(row["success_rate"]),
            float(row["speed_scale"]),
            float(row["pickup_aruco_clearance"]),
            float(row["attached_lidar_ignore_radius"]),
            -avg,
        )

    return max(stable, key=key)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-per-candidate", type=int, default=1)
    parser.add_argument("--confirmation-runs", type=int, default=10)
    parser.add_argument("--mission-step", default="step0")
    parser.add_argument("--per-run-timeout", default="420.0")
    parser.add_argument("--result-dir", default="/home/group4/Nursing-Home-Robot/mission_parameter_sweep_results")
    parser.add_argument("--speed-scales", default="1.0,1.15,1.30,1.45")
    parser.add_argument("--pickup-aruco-clearances", default="0.85")
    parser.add_argument("--attached-lidar-ignore-radii", default="2.5,2.8,3.1")
    parser.add_argument("--straight-under-stop-ys", default="0.16")
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    result_dir.mkdir(parents=True, exist_ok=True)
    summary_path = result_dir / "sweep_summary.csv"
    best_path = result_dir / "best_overrides.txt"

    summary_rows = []
    candidates = build_candidates(args)
    print(f"Testing {len(candidates)} parameter candidates.")

    for index, candidate in enumerate(candidates, start=1):
        csv_filename = f"sweep_candidate_{index:03d}.csv"
        returncode, wall_duration = run_batch(
            args,
            candidate["mission_overrides"],
            candidate["holonomic_overrides"],
            args.runs_per_candidate,
            csv_filename,
        )
        batch_rows = read_batch_rows(result_dir / csv_filename)
        success_count = sum(1 for row in batch_rows if row_success(row))
        runs = len(batch_rows)
        avg_duration = average_success_duration(batch_rows)
        summary_rows.append(
            {
                "candidate": index,
                "phase": "sweep",
                "runs": runs,
                "success_count": success_count,
                "success_rate": round(success_count / runs, 4) if runs else 0.0,
                "avg_success_duration_sec": "inf" if math.isinf(avg_duration) else round(avg_duration, 3),
                "speed_scale": candidate["speed_scale"],
                "pickup_aruco_clearance": candidate["pickup_aruco_clearance"],
                "attached_lidar_ignore_radius": candidate["attached_lidar_ignore_radius"],
                "straight_under_stop_y": candidate["straight_under_stop_y"],
                "overrides": candidate["mission_overrides"],
                "holonomic_overrides": candidate["holonomic_overrides"],
                "failure_summary": summarize_failures(batch_rows),
                "csv_filename": csv_filename,
                "process_returncode": returncode,
                "wall_duration_sec": round(wall_duration, 3),
            }
        )
        write_summary(summary_path, summary_rows)

    best = choose_best(summary_rows)
    if best is None:
        print("No candidate succeeded. Summary written to", summary_path)
        return

    best_path.write_text(
        "mission_param_overrides:=" + best["overrides"] + "\n"
        "holonomic_param_overrides:=" + best["holonomic_overrides"] + "\n"
    )
    print("\n=== Best sweep candidate ===")
    print("mission:", best["overrides"])
    print("holonomic:", best["holonomic_overrides"])

    if args.confirmation_runs > 0:
        csv_filename = "best_confirmation.csv"
        returncode, wall_duration = run_batch(
            args,
            best["overrides"],
            best["holonomic_overrides"],
            args.confirmation_runs,
            csv_filename,
        )
        batch_rows = read_batch_rows(result_dir / csv_filename)
        success_count = sum(1 for row in batch_rows if row_success(row))
        runs = len(batch_rows)
        avg_duration = average_success_duration(batch_rows)
        summary_rows.append(
            {
                "candidate": best["candidate"],
                "phase": "confirmation",
                "runs": runs,
                "success_count": success_count,
                "success_rate": round(success_count / runs, 4) if runs else 0.0,
                "avg_success_duration_sec": "inf" if math.isinf(avg_duration) else round(avg_duration, 3),
                "speed_scale": best["speed_scale"],
                "pickup_aruco_clearance": best["pickup_aruco_clearance"],
                "attached_lidar_ignore_radius": best["attached_lidar_ignore_radius"],
                "straight_under_stop_y": best["straight_under_stop_y"],
                "overrides": best["overrides"],
                "holonomic_overrides": best["holonomic_overrides"],
                "failure_summary": summarize_failures(batch_rows),
                "csv_filename": csv_filename,
                "process_returncode": returncode,
                "wall_duration_sec": round(wall_duration, 3),
            }
        )
        write_summary(summary_path, summary_rows)

    print("Sweep complete. Summary:", summary_path)
    print("Best overrides:", best_path)


if __name__ == "__main__":
    main()
