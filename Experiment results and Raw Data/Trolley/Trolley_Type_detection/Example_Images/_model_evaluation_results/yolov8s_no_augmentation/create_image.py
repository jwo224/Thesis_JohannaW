from pathlib import Path
import textwrap

import pandas as pd
from PIL import Image, ImageDraw, ImageFont


# ------------------------------------------------------------
# Settings
# ------------------------------------------------------------

SCRIPT_DIR = Path(__file__).resolve().parent
PREDICTIONS_CSV = SCRIPT_DIR / "predictions.csv"
OUTPUT_DIR = SCRIPT_DIR / "annotated_prediction_examples"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Number of example images to save per group
N_CORRECT_PER_CLASS = 2
N_WRONG_TOTAL = 8

# Image output width for thesis figures
TARGET_WIDTH = 900


# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------

def load_font(size: int):
    """Try to load a common font. Fall back to default if unavailable."""
    possible_fonts = [
        "arial.ttf",
        "Arial.ttf",
        "DejaVuSans.ttf",
        "C:/Windows/Fonts/arial.ttf",
    ]

    for font_path in possible_fonts:
        try:
            return ImageFont.truetype(font_path, size)
        except OSError:
            continue

    return ImageFont.load_default()


def resize_keep_aspect(image: Image.Image, target_width: int) -> Image.Image:
    """Resize image to target width while keeping aspect ratio."""
    width, height = image.size
    if width == target_width:
        return image

    scale = target_width / width
    new_height = int(height * scale)
    return image.resize((target_width, new_height), Image.LANCZOS)


def draw_labelled_image(row, output_path: Path):
    """Draw true/predicted/confidence information onto an image."""
    image_path = Path(row["image"])
    if not image_path.is_absolute():
        image_path = SCRIPT_DIR / image_path

    if not image_path.exists():
        print(f"Missing image, skipped: {image_path}")
        return False

    image = Image.open(image_path).convert("RGB")
    image = resize_keep_aspect(image, TARGET_WIDTH)

    draw = ImageDraw.Draw(image)
    font = load_font(28)
    small_font = load_font(23)

    true_label = str(row["true_label"])
    predicted_label = str(row["predicted_label"])
    confidence = float(row["confidence"])
    correct = bool(row["correct"])

    status = "Correct" if correct else "Wrong"

    label_text = (
        f"True: {true_label} | Predicted: {predicted_label} | "
        f"Confidence: {confidence:.2f} | {status}"
    )

    # Wrap long text if needed
    wrapped_lines = textwrap.wrap(label_text, width=60)

    padding = 16
    line_height = 34
    bar_height = padding * 2 + line_height * len(wrapped_lines)

    # White background bar
    draw.rectangle(
        [(0, 0), (image.width, bar_height)],
        fill=(255, 255, 255),
    )

    # Border: green if correct, red if wrong
    border_color = (0, 140, 0) if correct else (180, 0, 0)
    border_width = 8
    draw.rectangle(
        [(0, 0), (image.width - 1, image.height - 1)],
        outline=border_color,
        width=border_width,
    )

    # Text
    y = padding
    for line in wrapped_lines:
        draw.text((padding, y), line, fill=(0, 0, 0), font=font)
        y += line_height

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path, quality=95)
    return True


def safe_name(text: str) -> str:
    return (
        str(text)
        .replace(" ", "_")
        .replace("/", "_")
        .replace("\\", "_")
        .replace(":", "_")
    )


# ------------------------------------------------------------
# Load CSV
# ------------------------------------------------------------

df = pd.read_csv(PREDICTIONS_CSV, keep_default_na=False)

required_columns = [
    "image",
    "file",
    "true_label",
    "predicted_label",
    "confidence",
    "correct",
]

missing = [col for col in required_columns if col not in df.columns]
if missing:
    raise ValueError(f"Missing columns in predictions.csv: {missing}")

# Convert correct column safely
df["correct"] = df["correct"].astype(str).str.lower().isin(["true", "1", "yes", "wahr"])


# ------------------------------------------------------------
# Save selected correct examples per class
# ------------------------------------------------------------

correct_df = df[df["correct"]].copy()
wrong_df = df[~df["correct"]].copy()

saved_count = 0

for true_label in sorted(correct_df["true_label"].unique()):
    class_df = correct_df[correct_df["true_label"] == true_label].copy()

    # Use highest confidence correct examples
    class_df = class_df.sort_values("confidence", ascending=False).head(N_CORRECT_PER_CLASS)

    for index, row in class_df.iterrows():
        filename = (
            f"correct_{safe_name(true_label)}_"
            f"{index:03d}.jpg"
        )
        output_path = OUTPUT_DIR / "correct_examples" / filename

        if draw_labelled_image(row, output_path):
            saved_count += 1


# ------------------------------------------------------------
# Save wrong examples
# ------------------------------------------------------------

wrong_df = wrong_df.sort_values("confidence", ascending=False).head(N_WRONG_TOTAL)

for index, row in wrong_df.iterrows():
    filename = (
        f"wrong_true_{safe_name(row['true_label'])}_"
        f"pred_{safe_name(row['predicted_label'])}_"
        f"{index:03d}.jpg"
    )
    output_path = OUTPUT_DIR / "wrong_examples" / filename

    if draw_labelled_image(row, output_path):
        saved_count += 1


# ------------------------------------------------------------
# Save all annotated images, optional
# ------------------------------------------------------------

for index, row in df.iterrows():
    status = "correct" if row["correct"] else "wrong"
    filename = (
        f"{index:03d}_{status}_true_{safe_name(row['true_label'])}_"
        f"pred_{safe_name(row['predicted_label'])}_"
        f"{index:03d}.jpg"
    )
    output_path = OUTPUT_DIR / "all_annotated" / filename
    draw_labelled_image(row, output_path)


print(f"Done. Saved annotated images to: {OUTPUT_DIR.resolve()}")
print(f"Selected example images saved: {saved_count}")
print("Use images from:")
print(f"  {OUTPUT_DIR / 'correct_examples'}")
print(f"  {OUTPUT_DIR / 'wrong_examples'}")
print(f"  {OUTPUT_DIR / 'all_annotated'}")
