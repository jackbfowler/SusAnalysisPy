#!/usr/bin/env python3
"""Import ONE LOTUS mm hardpoint export into the shared SolidWorks/Python .txt.

    python tools/points_to_csv.py <input.txt> <output.txt> --axle front|rear
    python tools/points_to_csv.py pointsimport.txt datanew/2027Baja.txt --axle rear

The LOTUS export describes ONE corner; ``--axle`` names it. Point values are
millimetres in the legacy export frame (``+X`` forward, ``+Y`` left, ``+Z`` up)
and are written into the shared file in inches in the shared frame (``+X``
left, ``+Y`` up, ``+Z`` front): divide by 25.4, then
``(x, y, z)_shared = (y, z, x)_legacy``, with the shared point naming
(``LCA_*``/``UCA_*``/``shock_lower``/``shock_upper``/...). Keep this file's
naming tables in sync with ``src/sussyanal/io.py`` and
``tools/csv_to_shared.py``.

If ``output`` does not exist it is created with the imported corner filled in
and the other corner's variable lines left blank (to be filled by a later
import of the opposite axle). If it exists, only the imported corner's point
variables are (re)written; the other corner and any existing config values are
preserved. Config values (steering_rack, shock bump/droop, wheel size, shock
lower mount) cannot come from a LOTUS point export and are left blank for
manual fill.

Two input formats are supported:

1. Plain tab/space-separated rows, in this fixed order (20 rows):

       -421.700  298.400  62.800      #  1 Lower wishbone front pivot
       -1081.850 109.917  5.004       #  2 Lower wishbone rear pivot
       ...                            # ... wishbones, damper, track rod, spring
       -976.460  588.948  12.700      # 13 Wheel spindle point
       -976.460  670.621  12.700      # 14 Wheel centre point
       -0.000    0.302    -1.923      # 15 Part 1 C of G          (skipped)
       ...                            # 16-18 Part C of G         (skipped)
       -992.700  80.000   76.248      # 19 Inboard CV Centre -> Inner axle joint
       -992.700  40.000   76.248      # 20 Inner CV Axis Point    (skipped)

2. Labelled blocks (the "(N) Point N: <name> <x>" format), mapped by name:

       (1) Point 1: Lower Wishbone Front Pivot -421.700
       298.400
       62.800

Non-pickup rows ("Part N C of G", "Inner CV Axis Point", ...) are skipped; the
outer axle joint is taken from the wheel spindle point and the inner axle
joint from the inboard CV centre (unchanged design assumptions).
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

MM_PER_IN = 25.4

# ---- Canonical (legacy CSV) row order, shared with data/*.csv ----
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

# ---- Shared-format naming (mirror of src/sussyanal/io.py + csv_to_shared.py) ----
# Legacy row name -> shared point base name. Damper ends and spring pivots are
# the same physical points and collapse to shock_lower / shock_upper.
LEGACY_TO_SHARED = {
    "Lower wishbone front pivot": "LCA_front",
    "Lower wishbone rear pivot": "LCA_rear",
    "Lower wishbone outer ball joint": "LCA_outer",
    "Upper wishbone front pivot": "UCA_front",
    "Upper wishbone rear pivot": "UCA_rear",
    "Upper wishbone outer ball joint": "UCA_outer",
    "Damper wishbone end": "shock_lower",
    "Lower spring pivot point": "shock_lower",
    "Damper body end": "shock_upper",
    "Upper spring pivot point": "shock_upper",
    "Outer track rod ball joint": "trackrod_outer",
    "Inner track rod ball joint": "trackrod_inner",
    "Wheel spindle point": "spindle",
    "Wheel centre point": "wheel_center",
    "Outer axle joint": "axle_outer",
    "Inner axle joint": "axle_inner",
}

SHARED_POINT_ORDER = [
    "LCA_front", "LCA_rear", "LCA_outer",
    "UCA_front", "UCA_rear", "UCA_outer",
    "shock_lower", "shock_upper",
    "trackrod_outer", "trackrod_inner",
    "spindle", "wheel_center",
    "axle_outer", "axle_inner",
]

# Shared config variable order (values always blank from a LOTUS import).
CONFIG_VARS = [
    "steering_rack",
    "shock_bump_front", "shock_droop_front", "wheel_size_front",
    "shock_lower_mount_front",
    "shock_bump_rear", "shock_droop_rear", "wheel_size_rear",
    "shock_lower_mount_rear",
]

_AXLE_PREFIX = {"front": "F", "rear": "R"}

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

_VAR_LINE_RE = re.compile(r'^\s*"([^"]+)"\s*=\s*(.*?)\s*$')


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
    """Format inches: ints without decimal, floats up to 6 places, trimmed."""
    if v == int(v):
        return str(int(v))
    s = f"{v:.6f}"
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


def _read_existing(path: Path) -> dict[str, str]:
    """Existing shared file -> {variable name: raw value text ('' if blank)}."""
    vars_: dict[str, str] = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            m = _VAR_LINE_RE.match(line)
            if m is not None:
                vars_[m.group(1)] = m.group(2)
    return vars_


def _to_shared_bases(rows: dict[str, list[float]]) -> dict[str, list[float]]:
    """Collapse canonical rows (inches) into shared base points (inches)."""
    base_xyz: dict[str, list[float]] = {}
    for name, xyz in rows.items():
        base = LEGACY_TO_SHARED[name]
        if base in base_xyz:
            if any(abs(a - b) > 1e-6 for a, b in zip(base_xyz[base], xyz)):
                raise ValueError(
                    f"'{name}' conflicts with '{base}' "
                    f"({base_xyz[base]} vs {xyz})"
                )
        else:
            base_xyz[base] = xyz
    return base_xyz


def write_shared(
    out_path: Path, base_xyz: dict[str, list[float]], axle: str
) -> None:
    """Merge the imported corner into ``out_path`` (shared .txt format)."""
    prefix = _AXLE_PREFIX[axle]
    existing = _read_existing(out_path)

    core = [b for b in SHARED_POINT_ORDER if not b.startswith("axle_")]
    missing = [b for b in core if b not in base_xyz]
    if missing:
        raise ValueError("Missing hardpoints: " + ", ".join(missing))

    lines: list[str] = []
    for var in CONFIG_VARS:
        lines.append(f'"{var}" = {existing.get(var, "")}')
    for ax in ("F", "R"):
        for base in SHARED_POINT_ORDER:
            for axis, idx in (("x", 1), ("y", 2), ("z", 0)):
                key = f"{ax}_{base}_{axis}"
                if ax == prefix and base in base_xyz:
                    # legacy (x,y,z) inches -> shared (y,z,x): take by index.
                    value = fmt(base_xyz[base][idx])
                else:
                    value = existing.get(key, "")
                lines.append(f'"{key}" = {value}')

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="text file with the corner's points in mm")
    ap.add_argument("output", help="shared .txt path (e.g. datanew/2027Baja.txt)")
    ap.add_argument("--axle", choices=("front", "rear"), required=True,
                    help="which corner this export describes")
    args = ap.parse_args()

    text = Path(args.input).read_text(encoding="utf-8")
    points = parse_labeled(text) if "Point" in text else parse_plain(text)
    if not points:
        raise SystemExit("error: no points found in input")

    rows = build_rows(points)  # legacy canonical names, inches
    base_xyz = _to_shared_bases(rows)  # shared base names, legacy-frame inches
    out = Path(args.output)
    write_shared(out, base_xyz, args.axle)
    filled = sum(1 for b in base_xyz for _ in "xyz")
    print(f"Updated {out} ({args.axle} corner, {filled} coordinate variables; "
          f"mm -> in /{MM_PER_IN:g} + frame fix)")


if __name__ == "__main__":
    main()
