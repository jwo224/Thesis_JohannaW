import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from io import StringIO

# =========================
# SETTINGS
# =========================

script_folder = Path(__file__).parent
plot_folder = script_folder / "plot"

output_pdf = script_folder / "fsr_plots_4panel.pdf"
output_svg = script_folder / "fsr_plots_4panel.svg"

HEADER = "time_ms,angle,fsr_raw,fsr_voltage"
SMOOTHING_WINDOW = 21   # larger = smoother

# =========================
# FIND CSV FILES
# =========================

csv_files = sorted(plot_folder.glob("*.csv"))

if not csv_files:
    raise FileNotFoundError(f"No CSV files found in: {plot_folder}")

print("Found CSV files:")
for file in csv_files:
    print(f"  {file.name}")

# =========================
# LOAD CSV AS SEPARATE RUNS
# =========================

def load_measurement_runs(csv_path):
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as file:
        lines = [line.strip() for line in file if line.strip()]

    runs = []
    current_run = []

    for line in lines:
        if line.startswith("#"):
            continue

        if line == HEADER:
            if current_run:
                runs.append(current_run)
                current_run = []
            continue

        current_run.append(line)

    if current_run:
        runs.append(current_run)

    dataframes = []

    for run in runs:
        csv_text = HEADER + "\n" + "\n".join(run)
        df = pd.read_csv(StringIO(csv_text))

        df["time_ms"] = pd.to_numeric(df["time_ms"], errors="coerce")
        df["angle"] = pd.to_numeric(df["angle"], errors="coerce")
        df["fsr_raw"] = pd.to_numeric(df["fsr_raw"], errors="coerce")
        df["fsr_voltage"] = pd.to_numeric(df["fsr_voltage"], errors="coerce")

        df = df.dropna()

        if len(df) == 0:
            continue

        # reset time so every run starts at 0
        df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0

        # smooth each run individually
        df["fsr_raw_smooth"] = df["fsr_raw"].rolling(
            window=SMOOTHING_WINDOW,
            center=True,
            min_periods=1
        ).mean()

        dataframes.append(df)

    return dataframes

# =========================
# COLLECT ALL RUNS
# =========================

all_runs = []

for csv_file in csv_files:
    runs = load_measurement_runs(csv_file)

    for run_index, df in enumerate(runs):
        if len(runs) == 1:
            label_name = csv_file.stem
        else:
            label_name = f"{csv_file.stem} run {run_index + 1}"

        all_runs.append((label_name, df))

if not all_runs:
    raise ValueError("No valid measurement runs found.")

# =========================
# BUILD COMMON TIME AXIS
# =========================

max_time = max(df["time_s"].max() for _, df in all_runs)
common_time = np.linspace(0, max_time, 500)

raw_interp_values = []
smooth_interp_values = []

for _, df in all_runs:
    t = df["time_s"].to_numpy()
    raw = df["fsr_raw"].to_numpy()
    smooth = df["fsr_raw_smooth"].to_numpy()

    raw_interp = np.full_like(common_time, np.nan, dtype=float)
    smooth_interp = np.full_like(common_time, np.nan, dtype=float)

    valid = (common_time >= t.min()) & (common_time <= t.max())

    raw_interp[valid] = np.interp(common_time[valid], t, raw)
    smooth_interp[valid] = np.interp(common_time[valid], t, smooth)

    raw_interp_values.append(raw_interp)
    smooth_interp_values.append(smooth_interp)

raw_interp_values = np.vstack(raw_interp_values)
smooth_interp_values = np.vstack(smooth_interp_values)

mean_fsr_raw = np.nanmean(raw_interp_values, axis=0)
mean_fsr_smooth = np.nanmean(smooth_interp_values, axis=0)

# Optional: smooth the averaged curve one more time
mean_fsr_smooth = pd.Series(mean_fsr_smooth).rolling(
    window=SMOOTHING_WINDOW,
    center=True,
    min_periods=1
).mean().to_numpy()

# =========================
# CREATE FIGURE
# =========================

fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# 1) Raw FSR vs time
for label_name, df in all_runs:
    axs[0, 0].plot(
        df["time_s"],
        df["fsr_raw"],
        linewidth=1.2,
        label=label_name
    )

axs[0, 0].set_title("FSR raw value vs time")
axs[0, 0].set_xlabel("Time [s]")
axs[0, 0].set_ylabel("FSR value [raw ADC]")
axs[0, 0].grid(True, alpha=0.3)
axs[0, 0].legend(fontsize=7)

# 2) Angle vs time
for label_name, df in all_runs:
    axs[0, 1].plot(
        df["time_s"],
        df["angle"],
        linewidth=1.2,
        label=label_name
    )

axs[0, 1].set_title("Servo angle vs time")
axs[0, 1].set_xlabel("Time [s]")
axs[0, 1].set_ylabel("Servo angle [deg]")
axs[0, 1].grid(True, alpha=0.3)
axs[0, 1].legend(fontsize=7)

# 3) Average raw FSR vs time
axs[1, 0].plot(
    common_time,
    mean_fsr_raw,
    linewidth=2.0,
    label="Average raw FSR"
)

axs[1, 0].set_title("Average FSR of all measurements vs time")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("Average FSR value [raw ADC]")
axs[1, 0].grid(True, alpha=0.3)
axs[1, 0].legend(fontsize=8)

# 4) Average smoothed FSR vs time
axs[1, 1].plot(
    common_time,
    mean_fsr_smooth,
    linewidth=2.5,
    label="Average smoothed FSR"
)

axs[1, 1].set_title("Average smoothed FSR vs time")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("Smoothed average FSR [raw ADC]")
axs[1, 1].grid(True, alpha=0.3)
axs[1, 1].legend(fontsize=8)

plt.tight_layout()

# =========================
# SAVE VECTOR OUTPUT
# =========================

plt.savefig(output_pdf, bbox_inches="tight")
plt.savefig(output_svg, bbox_inches="tight")

print()
print(f"Saved PDF: {output_pdf}")
print(f"Saved SVG: {output_svg}")

plt.show()