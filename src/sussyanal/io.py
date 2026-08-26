"""CSV hardpoint/config parsing for suspension data files.

Ports the parsing logic duplicated across the MATLAB sources
(``detectImportOptions`` + ``readtable`` + row-name -> field mapping), including
``SusAnalysis/print_axle.m``.

CSV format: a header row ``,x,y,z`` followed by rows of
``<point name>,<x>,<y>,<z>`` (hardpoints) or ``<config name>,<value>,,``
(configuration). The configuration belongs to the hardpoint set it ships in —
see ``SuspensionData``.
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .geometry import Config, Geometry, SuspensionData

_CONFIG_ROWS = {
    "shock bump": "bump",
    "shock droop": "droop",
    "wheel size": "wheel_size",
    "steering rack": "steer_sweep",
    "shock lower mount": "shock_mount_lca",
    "wheelbase": "wheelbase",
}


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


def parse_csv(path) -> SuspensionData:
    """Parse a suspension CSV into a ``SuspensionData`` (hardpoints + config).

    Each configuration row found in the CSV overrides that parameter for this
    hardpoint set; parameters the CSV omits keep their fallback defaults and
    are listed in ``Config.missing``.
    """
    path = Path(path)
    points: dict[str, np.ndarray] = {}
    config = Config()
    found: set[str] = set()

    with path.open(newline="", encoding="utf-8-sig") as fh:
        rows = list(csv.reader(fh))

    for row in rows:
        if not row:
            continue
        name = (row[0] or "").strip()
        if not name:
            continue

        lower = name.lower()
        matched = next((k for k in _CONFIG_ROWS if k in lower), None)
        if matched is not None:
            setattr(config, _CONFIG_ROWS[matched], float(row[1]))
            found.add(_CONFIG_ROWS[matched])
        else:
            values = [float(v) for v in row[1:4] if v.strip() != ""]
            if len(values) == 3 and all(np.isfinite(values)):
                points[_clean_name(name)] = np.asarray(values, dtype=float)

    config.shock_mount_lca = int(config.shock_mount_lca)
    config.missing = tuple(f for f in Config.__dataclass_fields__ if f not in found and f != "missing")

    return SuspensionData(geometry=Geometry.from_mapping(points), config=config)
