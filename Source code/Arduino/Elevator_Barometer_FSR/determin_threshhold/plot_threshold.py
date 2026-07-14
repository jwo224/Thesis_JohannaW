import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# =========================
# SETTINGS
# =========================

script_folder = Path(__file__).parent
plot_folder = script_folder / "threshold_plot"

output_pdf = script_folder / "adaptive_threshold_multiple.pdf"
output_svg = script_folder / "adaptive_threshold_multiple.svg"

SMOOTHING_WINDOW = 31      # larger = smoother
PRESS_SAMPLES = 500        # interpolation points for press curves

AVERAGE_COLOR = "black"

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
# FIXED COLORS PER FILE/RUN
# =========================

labels = [csv_file.stem for csv_file in csv_files]

default_colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

run_colors = {}
for i, label in enumerate(labels):
    run_colors[label] = default_colors[i % len(default_colors)]

# =========================
# LOAD ONE CSV
# =========================

def load_adaptive_csv(csv_path):
    df = pd.read_csv(csv_path)

    df["trial"] = pd.to_numeric(df["trial"], errors="coerce")
    df["time_ms"] = pd.to_numeric(df["time_ms"], errors="coerce")
    df["threshold"] = pd.to_numeric(df["threshold"], errors="coerce")
    df["angle"] = pd.to_numeric(df["angle"], errors="coerce")
    df["fsr_raw"] = pd.to_numeric(df["fsr_raw"], errors="coerce")
    df["fsr_voltage"] = pd.to_numeric(df["fsr_voltage"], errors="coerce")
    df["phase"] = df["phase"].astype(str).str.strip()

    df = df.dropna().reset_index(drop=True)

    return df

# =========================
# BUILD TRIAL SUMMARY
# =========================

def build_trial_summary(df):
    rows = []

    for trial_id in sorted(df["trial"].unique()):
        trial_df = df[df["trial"] == trial_id].copy()
        press_df = trial_df[trial_df["phase"] == "press"].copy()

        if len(trial_df) == 0:
            continue

        threshold = trial_df["threshold"].iloc[0]

        if len(press_df) == 0:
            rows.append({
                "trial": int(trial_id),
                "threshold": threshold,
                "trigger_angle": np.nan,
                "trigger_fsr": np.nan,
                "press_duration_s": np.nan
            })
            continue

        press_df["press_time_s"] = (
            press_df["time_ms"] - press_df["time_ms"].iloc[0]
        ) / 1000.0

        reached_df = press_df[press_df["fsr_raw"] >= threshold]

        if len(reached_df) > 0:
            trigger_row = reached_df.iloc[0]
        else:
            trigger_row = press_df.iloc[-1]

        rows.append({
            "trial": int(trial_id),
            "threshold": threshold,
            "trigger_angle": trigger_row["angle"],
            "trigger_fsr": trigger_row["fsr_raw"],
            "press_duration_s": press_df["press_time_s"].max()
        })

    return pd.DataFrame(rows)

# =========================
# BUILD PRESS AVERAGE FOR ONE FILE
# =========================

def build_press_average(df, common_time):
    press_trials = []

    for trial_id in sorted(df["trial"].unique()):
        press_df = df[
            (df["trial"] == trial_id) &
            (df["phase"] == "press")
        ].copy()

        if len(press_df) < 2:
            continue

        press_df["press_time_s"] = (
            press_df["time_ms"] - press_df["time_ms"].iloc[0]
        ) / 1000.0

        t = press_df["press_time_s"].to_numpy()
        y = press_df["fsr_raw"].to_numpy()

        interp = np.full_like(common_time, np.nan, dtype=float)

        valid = (common_time >= t.min()) & (common_time <= t.max())
        interp[valid] = np.interp(common_time[valid], t, y)

        press_trials.append(interp)

    if not press_trials:
        return None, None

    press_trials = np.vstack(press_trials)

    mean_raw = np.nanmean(press_trials, axis=0)

    mean_smooth = pd.Series(mean_raw).rolling(
        window=SMOOTHING_WINDOW,
        center=True,
        min_periods=1
    ).mean().to_numpy()

    return mean_raw, mean_smooth

# =========================
# LOAD ALL FILES
# =========================

all_data = []
max_press_time = 0.0

for csv_file in csv_files:
    df = load_adaptive_csv(csv_file)
    label = csv_file.stem

    for trial_id in sorted(df["trial"].unique()):
        press_df = df[
            (df["trial"] == trial_id) &
            (df["phase"] == "press")
        ].copy()

        if len(press_df) < 2:
            continue

        press_time_s = (
            press_df["time_ms"] - press_df["time_ms"].iloc[0]
        ) / 1000.0

        max_press_time = max(max_press_time, press_time_s.max())

    all_data.append((label, df))

if max_press_time <= 0:
    raise ValueError("No valid press-phase data found.")

common_time = np.linspace(0, max_press_time, PRESS_SAMPLES)

# =========================
# BUILD SUMMARIES AND CURVES
# =========================

all_summaries = []
all_mean_raw_curves = []
all_mean_smooth_curves = []

for label, df in all_data:
    summary_df = build_trial_summary(df)
    mean_raw, mean_smooth = build_press_average(df, common_time)

    all_summaries.append((label, summary_df))

    if mean_raw is not None:
        all_mean_raw_curves.append((label, mean_raw))

    if mean_smooth is not None:
        all_mean_smooth_curves.append((label, mean_smooth))

# =========================
# AVERAGES ACROSS ALL FILES
# =========================

max_trials = max(len(summary_df) for _, summary_df in all_summaries)

threshold_matrix = np.full((len(all_summaries), max_trials), np.nan)
trigger_angle_matrix = np.full((len(all_summaries), max_trials), np.nan)

for i, (_, summary_df) in enumerate(all_summaries):
    n = len(summary_df)

    threshold_matrix[i, :n] = summary_df["threshold"].to_numpy()
    trigger_angle_matrix[i, :n] = summary_df["trigger_angle"].to_numpy()

avg_threshold = np.nanmean(threshold_matrix, axis=0)
avg_trigger_angle = np.nanmean(trigger_angle_matrix, axis=0)

trial_axis = np.arange(1, max_trials + 1)

raw_curve_matrix = np.vstack([curve for _, curve in all_mean_raw_curves])
smooth_curve_matrix = np.vstack([curve for _, curve in all_mean_smooth_curves])

overall_mean_raw = np.nanmean(raw_curve_matrix, axis=0)
overall_mean_smooth = np.nanmean(smooth_curve_matrix, axis=0)

overall_mean_smooth = pd.Series(overall_mean_smooth).rolling(
    window=SMOOTHING_WINDOW,
    center=True,
    min_periods=1
).mean().to_numpy()

# =========================
# PLOTTING
# =========================

fig, axs = plt.subplots(2, 2, figsize=(13, 9))

# =========================
# 1) THRESHOLD PROGRESSION
# =========================

for label, summary_df in all_summaries:
    color = run_colors[label]

    axs[0, 0].plot(
        summary_df["trial"],
        summary_df["threshold"],
        marker="o",
        linewidth=1.5,
        label=label,
        color=color
    )

    # Last trial is assumed to be the successful one
    success_row = summary_df.iloc[-1]
    success_trial = success_row["trial"]

    axs[0, 0].axvspan(
        success_trial - 0.25,
        success_trial + 0.25,
        facecolor=color,
        alpha=0.15,
        zorder=0
    )

    axs[0, 0].plot(
        success_row["trial"],
        success_row["threshold"],
        marker="s",
        markersize=8,
        linestyle="None",
        color=color,
        zorder=3
    )

axs[0, 0].plot(
    trial_axis,
    avg_threshold,
    linewidth=3.0,
    label="Average",
    color=AVERAGE_COLOR,
    zorder=4
)

axs[0, 0].set_title("Threshold progression across trials")
axs[0, 0].set_xlabel("Trial")
axs[0, 0].set_ylabel("Threshold [raw ADC]")
axs[0, 0].grid(True, alpha=0.3)
axs[0, 0].legend(fontsize=7)

# =========================
# 2) TRIGGER ANGLE PROGRESSION
# =========================

for label, summary_df in all_summaries:
    color = run_colors[label]

    axs[0, 1].plot(
        summary_df["trial"],
        summary_df["trigger_angle"],
        marker="o",
        linewidth=1.5,
        label=label,
        color=color
    )

    success_row = summary_df.iloc[-1]
    success_trial = success_row["trial"]

    axs[0, 1].axvspan(
        success_trial - 0.25,
        success_trial + 0.25,
        facecolor=color,
        alpha=0.15,
        zorder=0
    )

    axs[0, 1].plot(
        success_row["trial"],
        success_row["trigger_angle"],
        marker="s",
        markersize=8,
        linestyle="None",
        color=color,
        zorder=3
    )

axs[0, 1].plot(
    trial_axis,
    avg_trigger_angle,
    linewidth=3.0,
    label="Average",
    color=AVERAGE_COLOR,
    zorder=4
)

axs[0, 1].set_title("Trigger angle across trials")
axs[0, 1].set_xlabel("Trial")
axs[0, 1].set_ylabel("Trigger angle [deg]")
axs[0, 1].grid(True, alpha=0.3)
axs[0, 1].legend(fontsize=7)

# =========================
# 3) AVERAGE RAW FSR DURING PRESS
# =========================

for label, mean_raw in all_mean_raw_curves:
    color = run_colors[label]

    axs[1, 0].plot(
        common_time,
        mean_raw,
        linewidth=1.4,
        label=label,
        color=color
    )

axs[1, 0].plot(
    common_time,
    overall_mean_raw,
    linewidth=3.0,
    label="Average",
    color=AVERAGE_COLOR,
    zorder=4
)

axs[1, 0].set_title("Average FSR during press phase vs time")
axs[1, 0].set_xlabel("Press time [s]")
axs[1, 0].set_ylabel("FSR value [raw ADC]")
axs[1, 0].grid(True, alpha=0.3)
axs[1, 0].legend(fontsize=7)

# =========================
# 4) AVERAGE SMOOTHED FSR DURING PRESS
# =========================

for label, mean_smooth in all_mean_smooth_curves:
    color = run_colors[label]

    axs[1, 1].plot(
        common_time,
        mean_smooth,
        linewidth=1.5,
        label=label,
        color=color
    )

axs[1, 1].plot(
    common_time,
    overall_mean_smooth,
    linewidth=3.2,
    label="Average",
    color=AVERAGE_COLOR,
    zorder=4
)

axs[1, 1].set_title("Average smoothed FSR during press phase vs time")
axs[1, 1].set_xlabel("Press time [s]")
axs[1, 1].set_ylabel("Smoothed FSR value [raw ADC]")
axs[1, 1].grid(True, alpha=0.3)
axs[1, 1].legend(fontsize=7)

# =========================
# FINAL LAYOUT AND SAVE
# =========================

plt.tight_layout()

plt.savefig(output_pdf, bbox_inches="tight")
plt.savefig(output_svg, bbox_inches="tight")

print()
print(f"Saved PDF: {output_pdf}")
print(f"Saved SVG: {output_svg}")

plt.show()