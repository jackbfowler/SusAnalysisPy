#!/usr/bin/env python3
"""Convert legacy data/*.csv hardpoints into the shared SolidWorks/Python .txt.

Reads a FRONT csv and a REAR csv (the legacy ``io.py`` format) and emits ONE
shared file in SolidWorks linked-equations syntax: pure ``"name" = value``
lines, no comments and no header. Every value is a variable definition.

Coordinate frames
-----------------
Legacy CSV frame (one corner, left side):
    +X = forward, +Y = left (outboard), +Z = up.
Shared frame (left side of car):
    +X = left, +Y = up, +Z = front.

The transform is therefore a pure permutation::

    (x, y, z)_new = (y, z, x)_old

Values and signs are preserved verbatim (no mirroring, no translation — the
front and rear CSVs already share one origin).

Point naming
------------
``<axle>_<point>_<axis>`` with axle ``F_``/``R_``. The damper wishbone end and
the lower spring pivot are the same physical point and collapse to
``shock_lower``; the damper body end and upper spring pivot collapse to
``shock_upper``.

Config
------
``steering_rack`` is shared (one value, taken from the front file; blank if the
front CSV has none). ``shock_bump``, ``shock_droop``, ``wheel_size`` and
``shock_lower_mount`` get ``_front``/``_rear`` suffixes. Config rows the legacy
CSV omits are written as blank lines — the analyzer then falls back to its
defaults for that corner, tracked in ``Config.missing``. ``wheelbase`` is
dropped.

Usage
-----
    python tools/csv_to_shared.py data/2026BajaFront_12-16.csv \\
        data/2026BajaRear_3-3.csv -o datanew/2026Baja.txt
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

# Legacy CSV row name -> shared point base name (axle prefix + _axis appended).
POINT_MAP = {
    "Lower wishbone front pivot": "LCA_front",
    "Lower wishbone rear pivot": "LCA_rear",
    "Lower wishbone outer ball joint": "LCA_outer",
    "Upper wishbone front pivot": "UCA_front",
    "Upper wishbone rear pivot": "UCA_rear",
    "Upper wishbone outer ball joint": "UCA_outer",
    # Damper wishbone end == Lower spring pivot point -> shock_lower.
    "Damper wishbone end": "shock_lower",
    "Lower spring pivot point": "shock_lower",
    # Damper body end == Upper spring pivot point -> shock_upper.
    "Damper body end": "shock_upper",
    "Upper spring pivot point": "shock_upper",
    "Outer track rod ball joint": "trackrod_outer",
    "Inner track rod ball joint": "trackrod_inner",
    "Wheel spindle point": "spindle",
    "Wheel centre point": "wheel_center",
    "Outer axle joint": "axle_outer",
    "Inner axle joint": "axle_inner",
}

POINT_ORDER = [
    "LCA_front",
    "LCA_rear",
    "LCA_outer",
    "UCA_front",
    "UCA_rear",
    "UCA_outer",
    "shock_lower",
    "shock_upper",
    "trackrod_outer",
    "trackrod_inner",
    "spindle",
    "wheel_center",
    "axle_outer",
    "axle_inner",
]

# Legacy config row (substring of lowercased name) -> shared config base name.
# "wheelbase" is intentionally absent (removed).
CONFIG_MAP = {
    "steering rack": "steering_rack",
    "shock bump": "shock_bump",
    "shock droop": "shock_droop",
    "wheel size": "wheel_size",
    "shock lower mount": "shock_lower_mount",
}

CONFIG_ORDER = ["shock_bump", "shock_droop", "wheel_size", "shock_lower_mount"]


def fmt(v: float) -> str:
    """Format a number cleanly: ints without decimal, floats up to 6 places."""
    if v == int(v):
        return str(int(v))
    s = f"{v:.6f}"
    return s.rstrip("0").rstrip(".") if "." in s else s


def parse_csv(path: Path) -> tuple[dict[str, list[float]], dict[str, str]]:
    """Return (points {name: [x,y,z]}, config {key: raw_string})."""
    points: dict[str, list[float]] = {}
    config: dict[str, str] = {}
    with path.open(newline="", encoding="utf-8-sig") as fh:
        for row in csv.reader(fh):
            if not row:
                continue
            name = (row[0] or "").strip()
            if not name:
                continue
            lower = name.lower()
            matched = next((k for k in CONFIG_MAP if k in lower), None)
            if matched is not None:
                val = row[1].strip() if len(row) > 1 else ""
                if not val:
                    raise ValueError(f"{path}: config row '{name}' has no value")
                config[CONFIG_MAP[matched]] = val
            else:
                values = [float(v) for v in row[1:4] if v.strip() != ""]
                if len(values) == 3:
                    points[name] = values
    return points, config


def to_shared(front: Path, rear: Path, out: Path) -> None:
    f_points, f_cfg = parse_csv(front)
    r_points, r_cfg = parse_csv(rear)

    lines: list[str] = []

    # Config: steering_rack is shared (front value; blank if the front CSV has
    # no steering rack row). Per-axle config rows the legacy CSV omits are
    # written blank -> analyzer defaults apply (tracked in Config.missing).
    rack = fmt(float(f_cfg["steering_rack"])) if "steering_rack" in f_cfg else ""
    lines.append(f'"steering_rack" = {rack}')
    if "steering_rack" in r_cfg and float(r_cfg["steering_rack"]) != 0:
        print(
            f"warning: {rear} steering_rack={r_cfg['steering_rack']} "
            f"(non-zero rear rack ignored; shared value is front's)",
            file=sys.stderr,
        )
    for axle, cfg in (("front", f_cfg), ("rear", r_cfg)):
        for key in CONFIG_ORDER:
            val = fmt(float(cfg[key])) if key in cfg else ""
            lines.append(f'"{key}_{axle}" = {val}')

    # Geometry.
    for axle, points in (("F", f_points), ("R", r_points)):
        by_name: dict[str, list[float]] = {}
        for name, xyz in points.items():
            base = POINT_MAP[name]
            if base in by_name:
                if any(abs(a - b) > 1e-6 for a, b in zip(by_name[base], xyz)):
                    raise ValueError(
                        f"{axle}: '{name}' conflicts with an earlier row for "
                        f"'{base}' ({by_name[base]} vs {xyz})"
                    )
            else:
                by_name[base] = xyz
        for base in POINT_ORDER:
            if base not in by_name:
                raise ValueError(f"{axle}: missing point '{base}'")
            x, y, z = by_name[base]
            nx, ny, nz = y, z, x  # (x, y, z)_new = (y, z, x)_old
            lines.append(f'"{axle}_{base}_x" = {fmt(nx)}')
            lines.append(f'"{axle}_{base}_y" = {fmt(ny)}')
            lines.append(f'"{axle}_{base}_z" = {fmt(nz)}')

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {out} ({len(lines)} variables)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("front", help="legacy FRONT csv (io.py format)")
    ap.add_argument("rear", help="legacy REAR csv (io.py format)")
    ap.add_argument("-o", "--out", required=True, help="output shared .txt path")
    args = ap.parse_args()
    to_shared(Path(args.front), Path(args.rear), Path(args.out))


if __name__ == "__main__":
    main()
