from __future__ import annotations

from collections import Counter, defaultdict
from pathlib import Path
import csv
import json

import pandas as pd
from ultralytics import YOLO


MODELS_ROOT = Path(
    r"C:\Users\johan\Desktop\Master Thesis\Source code\YOLO Datasets\Models"
)

EXAMPLE_IMAGES_ROOT = Path(
    r"C:\Users\johan\Desktop\Master Thesis\Experiment results and Raw Data"
    r"\Trolley\Trolley_Type_detection\Example_Images"
)

OUTPUT_ROOT = EXAMPLE_IMAGES_ROOT / "_model_evaluation_results"

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
CONFIDENCE_THRESHOLD = 0.25


def normalize_label(label: str) -> str:
    text = label.strip().lower().replace("-", "_").replace(" ", "_")

    if "trash" in text:
        return "Trash"
    if "laundry" in text:
        return "Laundry"
    if "empty" in text:
        return "Empty"
    if text in {"none", "no_trolley", "background", "negative"}:
        return "None"

    return label.strip()


def discover_models(models_root: Path) -> dict[str, Path]:
    model_paths: dict[str, Path] = {}

    # Top-level .pt files, e.g. trolleys.pt
    for path in sorted(models_root.glob("*.pt")):
        model_paths[path.stem] = path

    # Ultralytics run folders with weights/best.pt
    for path in sorted(models_root.glob("*/weights/best.pt")):
        model_paths[path.parent.parent.name] = path

    if not model_paths:
        raise FileNotFoundError(f"No .pt models found under {models_root}")

    return model_paths


def discover_images(example_root: Path) -> list[tuple[Path, str]]:
    samples: list[tuple[Path, str]] = []

    for class_dir in sorted(path for path in example_root.iterdir() if path.is_dir()):
        if class_dir.name.startswith("_"):
            continue

        true_label = normalize_label(class_dir.name)

        for image_path in sorted(class_dir.iterdir()):
            if image_path.is_file() and image_path.suffix.lower() in IMAGE_EXTENSIONS:
                samples.append((image_path, true_label))

    if not samples:
        raise FileNotFoundError(f"No example images found under {example_root}")

    return samples


def predict_image(model: YOLO, image_path: Path, confidence_threshold: float) -> tuple[str, float, str]:
    result = model.predict(
        source=str(image_path),
        conf=confidence_threshold,
        verbose=False,
    )[0]

    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        return "None", 0.0, ""

    confidences = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)

    best_index = int(confidences.argmax())
    class_id = int(classes[best_index])
    confidence = float(confidences[best_index])

    raw_name = str(model.names.get(class_id, class_id))
    predicted_label = normalize_label(raw_name)

    return predicted_label, confidence, raw_name


def evaluate_model(
    model_name: str,
    model_path: Path,
    samples: list[tuple[Path, str]],
    output_root: Path,
) -> dict[str, object]:
    print(f"\nEvaluating {model_name}")
    print(f"  {model_path}")

    model = YOLO(str(model_path))
    rows = []

    for image_path, true_label in samples:
        predicted_label, confidence, raw_prediction = predict_image(
            model,
            image_path,
            CONFIDENCE_THRESHOLD,
        )

        correct = predicted_label == true_label

        rows.append(
            {
                "model": model_name,
                "image": str(image_path),
                "file": image_path.name,
                "true_label": true_label,
                "predicted_label": predicted_label,
                "raw_prediction": raw_prediction,
                "confidence": confidence,
                "correct": correct,
            }
        )

    df = pd.DataFrame(rows)
    model_output = output_root / model_name
    model_output.mkdir(parents=True, exist_ok=True)
    df.to_csv(model_output / "predictions.csv", index=False)

    labels = ["Empty", "Laundry", "Trash", "None"]
    confusion = pd.crosstab(
        df["true_label"],
        df["predicted_label"],
        rownames=["Actual"],
        colnames=["Predicted"],
        dropna=False,
    ).reindex(index=labels, columns=labels, fill_value=0)

    confusion.to_csv(model_output / "confusion_matrix.csv")

    overall_accuracy = float(df["correct"].mean())

    per_class_accuracy = {}
    for label in labels:
        class_df = df[df["true_label"] == label]
        if len(class_df) == 0:
            per_class_accuracy[label] = None
        else:
            per_class_accuracy[label] = float(class_df["correct"].mean())

    summary = {
        "model": model_name,
        "model_path": str(model_path),
        "n_images": int(len(df)),
        "accuracy": overall_accuracy,
        "per_class_accuracy": per_class_accuracy,
        "n_correct": int(df["correct"].sum()),
        "n_wrong": int((~df["correct"]).sum()),
    }

    with (model_output / "summary.json").open("w", encoding="utf-8") as file:
        json.dump(summary, file, indent=2)

    print(f"  accuracy: {overall_accuracy:.3f} ({summary['n_correct']}/{summary['n_images']})")

    return summary


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    models = discover_models(MODELS_ROOT)
    samples = discover_images(EXAMPLE_IMAGES_ROOT)

    print(f"Found {len(models)} models")
    print(f"Found {len(samples)} labelled example images")
    print("Images per folder/class:")

    counts = Counter(label for _, label in samples)
    for label, count in sorted(counts.items()):
        print(f"  {label}: {count}")

    summaries = []

    for model_name, model_path in models.items():
        summaries.append(
            evaluate_model(
                model_name=model_name,
                model_path=model_path,
                samples=samples,
                output_root=OUTPUT_ROOT,
            )
        )

    summary_df = pd.DataFrame(
        [
            {
                "model": item["model"],
                "n_images": item["n_images"],
                "accuracy": item["accuracy"],
                "n_correct": item["n_correct"],
                "n_wrong": item["n_wrong"],
                "empty_accuracy": item["per_class_accuracy"].get("Empty"),
                "laundry_accuracy": item["per_class_accuracy"].get("Laundry"),
                "trash_accuracy": item["per_class_accuracy"].get("Trash"),
                "none_accuracy": item["per_class_accuracy"].get("None"),
                "model_path": item["model_path"],
            }
            for item in summaries
        ]
    ).sort_values("accuracy", ascending=False)

    summary_df.to_csv(OUTPUT_ROOT / "model_comparison_summary.csv", index=False)

    print("\nModel comparison")
    print(summary_df[["model", "accuracy", "n_correct", "n_wrong"]].to_string(index=False))
    print(f"\nSaved results to: {OUTPUT_ROOT}")


if __name__ == "__main__":
    main()
