"""CSV hardpoint/config parsing for suspension data files.

Ports the parsing logic that is duplicated across the MATLAB sources
(``detectImportOptions`` + ``readtable`` + row-name -> field mapping), including
``SusAnalysis/print_axle.m``.

CSV format: a header row ``,x,y,z`` followed by rows of
``<point name>,<x>,<y>,<z>`` (hardpoints) or ``<config name>,<value>,,``
(configuration).
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .geometry import Config, Geometry


def _camel_to_snake(name: str) -> str:
    """``LowerWishboneFrontPivot`` -> ``lower_wishbone_front_pivot``."""
    out = [name[0].lower()]
    for ch in name[1:]:
        if ch.isupper():
            out.append("_")
            out.append(ch.lower())
        else:
            out.append(ch)
    return "".join(out)


def _clean_name(name: str) -> str:
    """Mirror MATLAB ``makeValidName(regexprep(..., title-case, remove spaces))``.

    ``'Lower wishbone front pivot'`` -> ``'LowerWishboneFrontPivot'``.
    """
    return "".join(word.capitalize() for word in name.lower().split())


def parse_csv(path) -> tuple[Geometry, Config]:
    """Parse a suspension CSV into ``(Geometry, Config)``.

    Configuration rows are matched the same way as the MATLAB ``contains``
    checks (e.g. ``"shock bump" in name``); everything else is treated as a
    hardpoint and stored as a 3-vector in inches.
    """
    path = Path(path)
    points: dict[str, np.ndarray] = {}
    config = Config()

    with path.open(newline="", encoding="utf-8-sig") as fh:
        rows = list(csv.reader(fh))

    for row in rows:
        if not row:
            continue
        name = (row[0] or "").strip()
        if not name:
            continue

        lower = name.lower()
        if "shock bump" in lower:
            config.bump = float(row[1])
        elif "shock droop" in lower:
            config.droop = float(row[1])
        elif "wheel size" in lower:
            config.wheel_size = float(row[1])
        elif "steering rack" in lower:
            config.steer_sweep = abs(float(row[1]))
        elif "shock lower mount" in lower:
            config.shock_mount_lca = int(float(row[1]))
        elif "wheelbase" in lower:
            config.wheelbase = float(row[1])
        else:
            values = [float(v) for v in row[1:4] if v.strip() != ""]
            if len(values) == 3 and all(np.isfinite(values)):
                points[_clean_name(name)] = np.asarray(values, dtype=float)

    return Geometry.from_mapping(points), config
