from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


# ------------------------------------------------------------
# Settings
# ------------------------------------------------------------

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = SCRIPT_DIR / "results.csv"
OUTPUT_DIR = SCRIPT_DIR / "yolo_vector_plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

MODEL_NAME = "yolov8s_no_augmentation"

# Larger fonts for thesis readability
plt.rcParams.update({
    "font.size": 12,
    "axes.titlesize": 14,
    "axes.labelsize": 12,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "legend.fontsize": 11,
    "figure.titlesize": 15,
})

# PGF settings for LaTeX / Overleaf
plt.rcParams.update({
    "pgf.texsystem": "pdflatex",
    "pgf.rcfonts": False,
})


# ------------------------------------------------------------
# Load data
# ------------------------------------------------------------

if not RESULTS_CSV.exists():
    raise FileNotFoundError(f"Could not find {RESULTS_CSV.resolve()}")

df = pd.read_csv(RESULTS_CSV)
df.columns = df.columns.str.strip()

required_columns = [
    "epoch",
    "train/box_loss",
    "train/cls_loss",
    "train/dfl_loss",
    "val/box_loss",
    "val/cls_loss",
    "val/dfl_loss",
    "metrics/precision(B)",
    "metrics/recall(B)",
    "metrics/mAP50(B)",
    "metrics/mAP50-95(B)",
]

missing = [col for col in required_columns if col not in df.columns]
if missing:
    raise ValueError(f"Missing required columns in results.csv: {missing}")

x = df["epoch"]


# ------------------------------------------------------------
# Helper function
# ------------------------------------------------------------

def save_figure(fig, filename_base: str) -> None:
    """Save one figure as PGF, PDF, and high-resolution PNG."""
    pgf_path = OUTPUT_DIR / f"{filename_base}.pgf"
    pdf_path = OUTPUT_DIR / f"{filename_base}.pdf"
    png_path = OUTPUT_DIR / f"{filename_base}.png"

    fig.savefig(pgf_path, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, dpi=300, bbox_inches="tight")

    print(f"Saved: {pgf_path}")
    print(f"Saved: {pdf_path}")
    print(f"Saved: {png_path}")


# ------------------------------------------------------------
# 1) Loss curves
# ------------------------------------------------------------

loss_plots = [
    ("train/box_loss", "Train box loss"),
    ("train/cls_loss", "Train classification loss"),
    ("train/dfl_loss", "Train DFL loss"),
    ("val/box_loss", "Validation box loss"),
    ("val/cls_loss", "Validation classification loss"),
    ("val/dfl_loss", "Validation DFL loss"),
]

fig, axes = plt.subplots(2, 3, figsize=(13, 6.8))

for ax, (column, title) in zip(axes.flatten(), loss_plots):
    ax.plot(x, df[column], marker="o", markersize=2.5, linewidth=1.3)
    ax.set_title(title)
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Loss")
    ax.grid(True, linewidth=0.3)

fig.suptitle(f"Training and validation losses: {MODEL_NAME}")
fig.tight_layout(rect=[0, 0, 1, 0.96])
save_figure(fig, "yolo_loss_curves")
plt.close(fig)


# ------------------------------------------------------------
# 2) Metric curves
# ------------------------------------------------------------

metric_plots = [
    ("metrics/precision(B)", "Precision"),
    ("metrics/recall(B)", "Recall"),
    ("metrics/mAP50(B)", "mAP50"),
    ("metrics/mAP50-95(B)", "mAP50--95"),
]

fig, axes = plt.subplots(2, 2, figsize=(10, 7))

for ax, (column, title) in zip(axes.flatten(), metric_plots):
    ax.plot(x, df[column], marker="o", markersize=2.5, linewidth=1.3)
    ax.set_title(title)
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Value")
    ax.set_ylim(0.0, 1.05)
    ax.grid(True, linewidth=0.3)

fig.suptitle(f"Validation metrics: {MODEL_NAME}")
fig.tight_layout(rect=[0, 0, 1, 0.95])
save_figure(fig, "yolo_metric_curves")
plt.close(fig)


# ------------------------------------------------------------
# 3) All curves in one figure
# ------------------------------------------------------------

all_plots = [
    ("train/box_loss", "Train box loss", "Loss"),
    ("train/cls_loss", "Train classification loss", "Loss"),
    ("train/dfl_loss", "Train DFL loss", "Loss"),
    ("metrics/precision(B)", "Precision", "Value"),
    ("metrics/recall(B)", "Recall", "Value"),
    ("val/box_loss", "Validation box loss", "Loss"),
    ("val/cls_loss", "Validation classification loss", "Loss"),
    ("val/dfl_loss", "Validation DFL loss", "Loss"),
    ("metrics/mAP50(B)", "mAP50", "Value"),
    ("metrics/mAP50-95(B)", "mAP50--95", "Value"),
]

fig, axes = plt.subplots(2, 5, figsize=(18, 7))

for ax, (column, title, ylabel) in zip(axes.flatten(), all_plots):
    ax.plot(x, df[column], marker="o", markersize=2.2, linewidth=1.2)
    ax.set_title(title)
    ax.set_xlabel("Epoch")
    ax.set_ylabel(ylabel)
    if ylabel == "Value":
        ax.set_ylim(0.0, 1.05)
    ax.grid(True, linewidth=0.3)

fig.suptitle(f"Training and validation curves: {MODEL_NAME}")
fig.tight_layout(rect=[0, 0, 1, 0.94])
save_figure(fig, "yolo_all_curves")
plt.close(fig)


print("\nDone.")
print(f"All plots saved in: {OUTPUT_DIR.resolve()}")
