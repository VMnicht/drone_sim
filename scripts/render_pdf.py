#!/usr/bin/env python3

import argparse
from pathlib import Path

import pypdfium2 as pdfium
from PIL import Image, ImageDraw


def main():
    parser = argparse.ArgumentParser(description="Render a PDF for visual QA")
    parser.add_argument("pdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scale", type=float, default=1.6)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    for old_page in args.output.glob("report_page_*.png"):
        old_page.unlink()
    document = pdfium.PdfDocument(str(args.pdf))
    pages = []
    for index in range(len(document)):
        page = document[index]
        rendered = page.render(scale=args.scale).to_pil().convert("RGB")
        rendered.save(args.output / f"report_page_{index + 1:02d}.png")
        pages.append(rendered.copy())

    columns = 4
    thumbnail_width = 420
    thumbnail_height = 594
    cell_width = 440
    cell_height = 634
    rows = (len(pages) + columns - 1) // columns
    contact_sheet = Image.new(
        "RGB", (cell_width * columns, cell_height * rows), (225, 228, 234)
    )
    for index, page in enumerate(pages):
        thumbnail = page.copy()
        thumbnail.thumbnail((thumbnail_width, thumbnail_height))
        cell = Image.new("RGB", (cell_width, cell_height), "white")
        cell.paste(
            thumbnail,
            ((cell_width - thumbnail.width) // 2, 14),
        )
        ImageDraw.Draw(cell).text((12, 610), f"Page {index + 1}", fill="black")
        contact_sheet.paste(
            cell, ((index % columns) * cell_width, (index // columns) * cell_height)
        )
    contact_sheet.save(args.output / "report_contact_sheet.png")
    print(f"Rendered {len(pages)} pages into {args.output}")


if __name__ == "__main__":
    main()

