"""Generate print-ready AprilTag PDFs (exact metric sizes, stdlib-only PDF).

Implements the official apriltag_to_image() rendering from
AprilRobotics/apriltag (apriltag.c) for tagStandard41h12:
9x9 cells, black background, white fixed ring, 41 data bits
(bit i -> row bit_y[i]+2, col bit_x[i]+2, white iff code bit (40-i) set).

Print-scale compensation: pass --print-scale (measured/true, e.g. 0.96)
so the printed tag matches --size-mm after the printer's scaling.

Detection size ("Erkennungsgröße") = outer black edge, excl. quiet zone.

Example:
    python3 tools/make_apriltags.py --out ~/Downloads/apriltags.pdf
"""

from __future__ import annotations

import argparse
from pathlib import Path

# (code, tag ID, true detection size in mm)
CODES = {
    0: 0x000001BD8A64AD10,
    1: 0x000001BDC4F3B2D5,
}

BIT_XY = [
    (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2), (3, -2), (4, -2),
    (5, -2), (1, 1), (2, 1), (6, -2), (6, -1), (6, 0), (6, 1),
    (6, 2), (6, 3), (6, 4), (6, 5), (3, 1), (3, 2), (6, 6),
    (5, 6), (4, 6), (3, 6), (2, 6), (1, 6), (0, 6), (-1, 6),
    (3, 3), (2, 3), (-2, 6), (-2, 5), (-2, 4), (-2, 3), (-2, 2),
    (-2, 1), (-2, 0), (-2, -1), (1, 3), (1, 2), (2, 2),
]
assert len(BIT_XY) == 41

GRID = 9  # total_width


def white_ring() -> set[tuple[int, int]]:
    """Fixed white ring cells (row, col), transcribed from apriltag_to_image."""
    cells: set[tuple[int, int]] = set()
    for i in range(4):  # white_border_width(5) - 1, start offset 2
        cells.add((2, 2 + i))      # top row
        cells.add((2 + i, 6))      # right col
        cells.add((6, 3 + i))      # bottom row
        cells.add((3 + i, 2))      # left col
    return cells


def tag_cells(code: int) -> dict[tuple[int, int], bool]:
    """Map (row, col) -> True if cell must stay black."""
    data = {(y + 2, x + 2): i for i, (x, y) in enumerate(BIT_XY)}
    assert len(data) == 41, "data positions must be unique"
    ring = white_ring()
    assert not (set(data) & ring), "data must not overlap white ring"
    cells: dict[tuple[int, int], bool] = {}
    for r in range(GRID):
        for c in range(GRID):
            if (r, c) in ring:
                cells[(r, c)] = False
            elif (r, c) in data:
                i = data[(r, c)]
                bit_set = bool(code & (1 << (40 - i)))
                cells[(r, c)] = not bit_set
            else:
                cells[(r, c)] = True  # fixed black (separator rings)
    return cells


MM = 72.0 / 25.4
A4W, A4H = 210 * MM, 297 * MM


def escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def page(tag_id: int, size_pdf_mm: float, size_true_mm: float) -> str:
    code = CODES[tag_id]
    cells = tag_cells(code)
    cell = size_pdf_mm * MM / GRID
    tag_w = size_pdf_mm * MM
    ox = (A4W - tag_w) / 2
    oy = 80 * MM  # bottom of tag; labels live at the page bottom, quiet zone stays clear
    parts = ["0 g"]
    for (r, c), black in cells.items():
        if black:
            x = ox + c * cell
            y = oy + (GRID - 1 - r) * cell
            parts.append(f"{x:.2f} {y:.2f} {cell:.2f} {cell:.2f} re f")
    parts.append("BT /F1 13 Tf 0 0 0 rg")
    parts.append(f"36 {30 * MM:.2f} Td (tagStandard41h12  ID {tag_id}) Tj ET")
    parts.append("BT /F1 9 Tf 0 0 0 rg")
    lines = [
        f"Erkennungsgroesse (schwarze Aussenkante, echt): {size_true_mm:.0f} mm",
        f"Im PDF: {size_pdf_mm:.2f} mm (Druckskala 96 % bereits rausgerechnet)",
        "Drucken: A4, 100 % / Tatsaechliche Groesse, nicht skalieren.",
        "Kontrolle mit Lineal: schwarze Aussenkante messen.",
        "Ruhigzone: weisser Rand um das Tag freihalten.",
    ]
    y = 24 * MM
    for line in lines:
        parts.append(f"BT /F1 9 Tf 0 0 0 rg 36 {y:.2f} Td ({escape(line)}) Tj ET")
        y -= 5 * MM
    return "\n".join(parts) + "\n"


def build_pdf(pages: list[str]) -> bytes:
    objs: dict[int, str] = {
        1: "<< /Type /Catalog /Pages 2 0 R >>",
        2: f"<< /Type /Pages /Kids [{' '.join(f'{3 + 2 * k} 0 R' for k in range(len(pages)))}] /Count {len(pages)} >>",
    }
    order: list[int] = [1, 2]
    streams: dict[int, str] = {}
    obj_id = 3
    font_id = 3 + 2 * len(pages)
    for k, content in enumerate(pages):
        page_id, stream_id = 3 + 2 * k, 4 + 2 * k
        objs[page_id] = (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {A4W:.2f} {A4H:.2f}]"
            f" /Resources << /Font << /F1 {font_id} 0 R >> >>"
            f" /Contents {stream_id} 0 R >>"
        )
        streams[stream_id] = content
        order += [page_id, stream_id]
    objs[font_id] = "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"
    order.append(font_id)
    pdf = "%PDF-1.4\n"
    offs: dict[int, int] = {}
    for i in order:
        offs[i] = len(pdf.encode())
        if i in streams:
            s = streams[i]
            pdf += f"{i} 0 obj\n<< /Length {len(s.encode())} >>\nstream\n{s}endstream\nendobj\n"
        else:
            pdf += f"{i} 0 obj\n{objs[i]}\nendobj\n"
    xref = len(pdf.encode())
    n = max(order) + 1
    pdf += f"xref\n0 {n}\n0000000000 65535 f \n"
    for i in range(1, n):
        pdf += f"{offs.get(i, 0):010d} 00000 n \n"
    pdf += f"trailer\n<< /Size {n} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF"
    return pdf.encode()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--print-scale", type=float, default=0.96)
    ap.add_argument("--big-id", type=int, default=0)
    ap.add_argument("--big-mm", type=float, default=160.0)
    ap.add_argument("--small-id", type=int, default=1)
    ap.add_argument("--small-mm", type=float, default=48.0)
    args = ap.parse_args()

    pages = [
        page(args.big_id, args.big_mm / args.print_scale, args.big_mm),
        page(args.small_id, args.small_mm / args.print_scale, args.small_mm),
    ]
    out = Path(args.out).expanduser()
    out.write_bytes(build_pdf(pages))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
