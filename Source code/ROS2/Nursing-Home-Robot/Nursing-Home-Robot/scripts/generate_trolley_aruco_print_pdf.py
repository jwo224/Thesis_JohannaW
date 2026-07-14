#!/usr/bin/env python3

from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


MM_PER_INCH = 25.4
A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0
DPI = 300
OUTPUT = Path("trolley_aruco_print_a4.pdf")


def mm_to_px(mm):
    return int(round(mm * DPI / MM_PER_INCH))


def load_font(size_px):
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        try:
            return ImageFont.truetype(path, size_px)
        except OSError:
            pass
    return ImageFont.load_default()


def draw_centered_text(draw, center, text, font, fill):
    cx, cy = center
    bbox = draw.textbbox((0, 0), text, font=font)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.text((cx - width / 2, cy - height / 2), text, font=font, fill=fill)


def marker_image(dictionary, marker_id, marker_size_mm):
    marker_px = mm_to_px(marker_size_mm)
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px)
    return Image.fromarray(image).convert("RGB")


def grid_slots(card_size_mm, cols, rows, used_count):
    page_w = mm_to_px(A4_WIDTH_MM)
    page_h = mm_to_px(A4_HEIGHT_MM)
    card = mm_to_px(card_size_mm)
    grid_w = cols * card
    grid_h = rows * card
    start_x = (page_w - grid_w) // 2
    start_y = (page_h - grid_h) // 2

    slots = []
    for row in range(rows):
        for col in range(cols):
            if len(slots) >= used_count:
                return slots
            x0 = start_x + col * card
            y0 = start_y + row * card
            slots.append((x0, y0, x0 + card, y0 + card))
    return slots


def draw_cut_grid(draw, slots):
    if not slots:
        return
    line = (150, 150, 150)
    for x0, y0, x1, y1 in slots:
        draw.rectangle((x0, y0, x1, y1), outline=line, width=2)


def draw_outer_crop_marks(draw, slots):
    if not slots:
        return
    x0 = min(slot[0] for slot in slots)
    y0 = min(slot[1] for slot in slots)
    x1 = max(slot[2] for slot in slots)
    y1 = max(slot[3] for slot in slots)
    mark = mm_to_px(7)
    gap = mm_to_px(1.2)
    width = 3
    black = (0, 0, 0)

    draw.line((x0 - gap - mark, y0, x0 - gap, y0), fill=black, width=width)
    draw.line((x0, y0 - gap - mark, x0, y0 - gap), fill=black, width=width)
    draw.line((x1 + gap, y0, x1 + gap + mark, y0), fill=black, width=width)
    draw.line((x1, y0 - gap - mark, x1, y0 - gap), fill=black, width=width)
    draw.line((x0 - gap - mark, y1, x0 - gap, y1), fill=black, width=width)
    draw.line((x0, y1 + gap, x0, y1 + gap + mark), fill=black, width=width)
    draw.line((x1 + gap, y1, x1 + gap + mark, y1), fill=black, width=width)
    draw.line((x1, y1 + gap, x1, y1 + gap + mark), fill=black, width=width)


def draw_scale_check(draw):
    x0 = mm_to_px(155)
    y = mm_to_px(286)
    x1 = x0 + mm_to_px(40)
    draw.line((x0, y, x1, y), fill=(0, 0, 0), width=3)
    draw.line((x0, y - mm_to_px(1.5), x0, y + mm_to_px(1.5)), fill=(0, 0, 0), width=3)
    draw.line((x1, y - mm_to_px(1.5), x1, y + mm_to_px(1.5)), fill=(0, 0, 0), width=3)
    draw.text((x0, y + mm_to_px(2)), "40 mm", font=load_font(mm_to_px(2.6)), fill=(0, 0, 0))


def make_front_page(dictionary, marker_ids, card_size_mm, marker_size_mm, cols, rows, title):
    page_size = (mm_to_px(A4_WIDTH_MM), mm_to_px(A4_HEIGHT_MM))
    page = Image.new("RGB", page_size, "white")
    draw = ImageDraw.Draw(page)
    slots = grid_slots(card_size_mm, cols, rows, len(marker_ids))

    card_px = mm_to_px(card_size_mm)
    marker_px = mm_to_px(marker_size_mm)
    offset = (card_px - marker_px) // 2
    for marker_id, (x0, y0, _x1, _y1) in zip(marker_ids, slots):
        page.paste(marker_image(dictionary, marker_id, marker_size_mm), (x0 + offset, y0 + offset))

    draw_cut_grid(draw, slots)
    draw_outer_crop_marks(draw, slots)
    draw.text(
        (mm_to_px(8), mm_to_px(4)),
        f"{title} | print at 100%, no fit-to-page",
        font=load_font(mm_to_px(3.0)),
        fill=(0, 0, 0),
    )
    draw_scale_check(draw)
    return page, slots


def make_back_page(marker_ids, slots, title):
    page_size = (mm_to_px(A4_WIDTH_MM), mm_to_px(A4_HEIGHT_MM))
    page = Image.new("RGB", page_size, "white")
    draw = ImageDraw.Draw(page)
    page_w = page_size[0]
    font = load_font(mm_to_px(9))
    small = load_font(mm_to_px(2.7))
    gray = (185, 185, 185)

    mirrored_slots = [(page_w - x1, y0, page_w - x0, y1) for x0, y0, x1, y1 in slots]
    draw_cut_grid(draw, mirrored_slots)
    draw_outer_crop_marks(draw, mirrored_slots)

    for marker_id, (x0, y0, x1, y1) in zip(marker_ids, mirrored_slots):
        draw_centered_text(draw, ((x0 + x1) / 2, (y0 + y1) / 2), f"ID {marker_id}", font, gray)

    draw.text(
        (mm_to_px(8), mm_to_px(4)),
        f"{title} back labels | duplex flip on long edge",
        font=small,
        fill=gray,
    )
    return page


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    page_specs = [
        {
            "marker_ids": list(range(0, 6)),
            "card_size_mm": 90.0,
            "marker_size_mm": 80.0,
            "cols": 2,
            "rows": 3,
            "title": "Underside markers 0-5, 80 mm marker, 90 mm card",
        },
        {
            "marker_ids": list(range(6, 12)),
            "card_size_mm": 90.0,
            "marker_size_mm": 80.0,
            "cols": 2,
            "rows": 3,
            "title": "Underside markers 6-11, 80 mm marker, 90 mm card",
        },
        {
            "marker_ids": list(range(12, 16)),
            "card_size_mm": 90.0,
            "marker_size_mm": 80.0,
            "cols": 2,
            "rows": 2,
            "title": "Underside markers 12-15, 80 mm marker, 90 mm card",
        },
        {
            "marker_ids": list(range(16, 28)),
            "card_size_mm": 43.0,
            "marker_size_mm": 38.0,
            "cols": 4,
            "rows": 3,
            "title": "Side markers 16-27, 38 mm marker, 43 mm card",
        },
    ]

    pages = []
    for spec in page_specs:
        front, slots = make_front_page(dictionary, **spec)
        pages.append(front)
        pages.append(make_back_page(spec["marker_ids"], slots, spec["title"]))

    pages[0].save(OUTPUT, "PDF", resolution=DPI, save_all=True, append_images=pages[1:])
    print(f"Wrote {OUTPUT.resolve()} with {len(pages)} A4 pages.")


if __name__ == "__main__":
    main()
