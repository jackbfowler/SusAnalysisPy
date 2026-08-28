#!/usr/bin/env python3
"""Convert a hardpoint export (mm) into an io.py-compatible CSV (inches).

python tools/points_to_csv.py "pointsimport.txt" "data/2027BajaRear.csv"


Two input formats are supported:

1. Plain tab/space-separated rows, in this fixed order (20 rows):

       -421.700  298.400  62.800      #  1 Lower wishbone front pivot
       -1081.850 109.917  5.004       #  2 Lower wishbone rear pivot
       ...                            # ... wishbones, damper, track rod, spring
       -976.460  588.948  12.700      # 13 Wheel spindle point
       -976.460  670.621  12.700      # 14 Wheel centre point
       -0.000    0.302    -1.923      # 15 Part 1 C of G          (skipped)
       -0.091    0.578    -2.146      # 16 Part 2 C of G          (skipped)
       0.095     0.650    -0.695      # 17 Part 3 C of G          (skipped)
       -0.158    0.225    -0.412      # 18 Part 4 C of G          (skipped)
       -992.700  80.000   76.248      # 19 Inboard CV Centre -> Inner axle joint
       -992.700  40.000   76.248      # 20 Inner CV Axis Point    (skipped)

2. Labelled blocks (the "(N) Point N: <name> <x>" format), mapped by name:

       (1) Point 1: Lower Wishbone Front Pivot -421.700
       298.400
       62.800

Values are millimetres; the output CSV is in inches (divide by 25.4).

Non-pickup rows ("Part N C of G", "Inner CV Axis Point", ...) are skipped.
Config rows (steering rack, shock bump/droop, ...) are written as XXXX
placeholders, to be filled in before running the analyzer.

Usage:
    python tools/points_to_csv.py <input.txt> <output.csv>
    python tools/points_to_csv.py pointsimport.txt data/2027BajaRear.csv
"""
from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

MM_PER_IN = 25.4

# ---- Canonical CSV row order (matches every data/*.csv) ----
ROW_ORDER = [
    "Lower wishbone front pivot",
    "Lower wishbone rear pivot",
    "Lower wishbone outer ball joint",
    "Upper wishbone front pivot",
    "Upper wishbone rear pivot",
    "Upper wishbone outer ball joint",
    "Damper wishbone end",
    "Damper body end",
    "Outer track rod ball joint",
    "Inner track rod ball joint",
    "Upper spring pivot point",
    "Lower spring pivot point",
    "Wheel spindle point",
    "Wheel centre point",
    "Outer axle joint",
    "Inner axle joint",
]

# ---- Plain 20-row input: row index -> canonical CSV row name (None = skip) ----
POSITIONAL_ORDER = [
    "Lower wishbone front pivot",       #  1
    "Lower wishbone rear pivot",        #  2
    "Lower wishbone outer ball joint",  #  3
    "Upper wishbone front pivot",       #  4
    "Upper wishbone rear pivot",        #  5
    "Upper wishbone outer ball joint",  #  6
    "Damper wishbone end",              #  7
    "Damper body end",                  #  8
    "Outer track rod ball joint",       #  9
    "Inner track rod ball joint",       # 10
    "Upper spring pivot point",         # 11
    "Lower spring pivot point",         # 12
    "Wheel spindle point",              # 13
    "Wheel centre point",               # 14
    None,                               # 15 Part 1 C of G (non-pickup, skip)
    None,                               # 16 Part 2 C of G (non-pickup, skip)
    None,                               # 17 Part 3 C of G (non-pickup, skip)
    None,                               # 18 Part 4 C of G (non-pickup, skip)
    "Inner axle joint",                 # 19 Inboard CV Centre
    None,                               # 20 Inner CV Axis Point (means nothing, skip)
]

# ---- Labelled input: export name (lowercased) -> CSV row name (None = skip) ----
HEADER_RE = re.compile(
    r"^\s*\(\s*\d+\s*\)\s*Point\s*\d+\s*:\s*(?P<name>.+?)\s*"
    r"(?P<x>[-+]?\s*\d*\.?\d+)\s*$",
    re.IGNORECASE,
)
NUM_RE = re.compile(r"[-+]?\s*\d*\.?\d+")

NAME_MAP = {
    "lower wishbone front pivot": "Lower wishbone front pivot",
    "lower wishbone rear pivot": "Lower wishbone rear pivot",
    "lower wishbone outer ball joint": "Lower wishbone outer ball joint",
    "upper wishbone front pivot": "Upper wishbone front pivot",
    "upper wishbone rear pivot": "Upper wishbone rear pivot",
    "upper wishbone outer ball joint": "Upper wishbone outer ball joint",
    "damper wishbone end": "Damper wishbone end",
    "damper body end": "Damper body end",
    "outer track rod ball joint": "Outer track rod ball joint",
    "inner track rod ball joint": "Inner track rod ball joint",
    "upper spring pivot point": "Upper spring pivot point",
    "lower spring pivot point": "Lower spring pivot point",
    "wheel spindle point": "Wheel spindle point",
    "wheel centre point": "Wheel centre point",
    # Axle: the outer axle joint sits at the wheel spindle point, and the
    # inner axle joint is the inboard CV centre.
    "inboard cv centre": "Inner axle joint",
    # Non-pickup information (skipped):
    "part 1 c of g": None,
    "part 2 c of g": None,
    "part 3 c of g": None,
    "part 4 c of g": None,
    "inner cv axis point": None,
}

CONFIG_ROWS = [
    "Steering rack",
    "Shock bump from ride",
    "Shock droop from ride",
    "Wheel size",
    "Shock lower mount",
    "Wheelbase",
]


def _num(token: str) -> float:
    return float(token.replace(" ", ""))


def parse_plain(text: str) -> list[tuple[str, list[float]]]:
    """Parse plain rows of 3 numbers into [(row_name, [x, y, z]), ...]."""
    points: list[tuple[str, list[float]]] = []
    seen = 0  # position of every non-blank data row in the input
    for lineno, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        tokens = NUM_RE.findall(line)
        if len(tokens) < 3:
            raise ValueError(f"line {lineno}: expected 3 numbers, got {len(tokens)}")
        seen += 1
        if seen > len(POSITIONAL_ORDER):
            raise ValueError(
                f"line {lineno}: too many rows (expected at most "
                f"{len(POSITIONAL_ORDER)}); got at least {seen}"
            )
        name = POSITIONAL_ORDER[seen - 1]
        if name is None:
            continue  # non-pickup row (C of G, CV axis point) -> skip
        points.append((name, [_num(t) for t in tokens[:3]]))
    if seen < 20:
        raise ValueError(
            f"expected 20 rows (16 pickup points), found only {seen} rows"
        )
    return points


def parse_labeled(text: str) -> list[tuple[str, list[float]]]:
    """Parse the labelled mm format into [(name, [x, y, z]), ...]."""
    points: list[tuple[str, list[float]]] = []
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        m = HEADER_RE.match(line)
        if m is None:
            i += 1
            continue
        name = m.group("name").strip()
        coords = [_num(m.group("x"))]
        j = i + 1
        while len(coords) < 3 and j < len(lines):
            nums = NUM_RE.findall(lines[j].strip())
            coords.extend(_num(n) for n in nums)
            j += 1
        if len(coords) < 3:
            raise ValueError(f"Point '{name}' has only {len(coords)} coordinates")
        points.append((name, coords[:3]))
        i = j
    return points


def fmt(v: float) -> str:
    """Format inches with up to 5 decimals, trailing zeros trimmed."""
    s = f"{v:.5f}"
    return s.rstrip("0").rstrip(".") if "." in s else s


def build_rows(points: list[tuple[str, list[float]]]) -> dict[str, list[float]]:
    """Map parsed mm points -> canonical rows (inches)."""
    rows: dict[str, list[float]] = {}
    for name, coords_mm in points:
        target = name if name in ROW_ORDER else NAME_MAP.get(name.lower())
        if target is None:
            continue
        if target in rows:
            raise ValueError(f"Duplicate row '{target}' (from point '{name}')")
        rows[target] = [c / MM_PER_IN for c in coords_mm]

    # Outer axle joint sits at the wheel spindle point for this design.
    if "Wheel spindle point" in rows and "Outer axle joint" not in rows:
        rows["Outer axle joint"] = list(rows["Wheel spindle point"])

    missing = [r for r in ROW_ORDER if r not in rows]
    if missing:
        raise ValueError(
            "Missing hardpoints: " + ", ".join(missing)
            + ". Add them to NAME_MAP or provide them in the input."
        )
    return rows


def write_csv(rows: dict[str, list[float]], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["", "x", "y", "z"])
        for row in ROW_ORDER:
            w.writerow([row] + [fmt(v) for v in rows[row]])
        w.writerow(["", "", "", ""])
        for cfg in CONFIG_ROWS:
            w.writerow([cfg, "XXXX", "", ""])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="text file with points in mm")
    ap.add_argument("output", help="output CSV path (e.g. data/2027BajaRear.csv)")
    args = ap.parse_args()

    text = Path(args.input).read_text(encoding="utf-8")
    points = parse_labeled(text) if "Point" in text else parse_plain(text)
    if not points:
        raise SystemExit("error: no points found in input")
    rows = build_rows(points)
    write_csv(rows, Path(args.output))
    print(f"Wrote {Path(args.output)} with {len(rows)} hardpoints "
          f"(mm -> in, /{MM_PER_IN:g})")


if __name__ == "__main__":
    main()
