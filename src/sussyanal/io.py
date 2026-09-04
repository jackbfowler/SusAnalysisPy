"""Parsing of the shared suspension format (+ the deprecated legacy CSV).

Ports the parsing logic duplicated across the MATLAB sources
(``detectImportOptions`` + ``readtable`` + row-name -> field mapping), including
``SusAnalysis/print_axle.m``.

Shared format (current)
-----------------------
One file per year/version holds BOTH axles in SolidWorks linked-equations
syntax: a ``"name" = value`` line per variable, nothing else. Point variables
are ``<F|R>_<point>_<axis>`` in the shared CAD frame (``+X`` left, ``+Y`` up,
``+Z`` front); config variables are ``steering_rack`` (one shared value, the
front rack) plus ``shock_bump``, ``shock_droop``, ``wheel_size`` and
``shock_lower_mount`` with a ``_front``/``_rear`` suffix. ``wheelbase`` is no
longer carried.

``parse_csv(path, axle="front"|"rear")`` parses ONE axle into a
:class:`SuspensionData` — front and rear simulations stay separate and the
caller chooses the corner. Shared-frame coordinates are transformed back into
the analyzer's internal frame (``+X`` forward, ``+Y`` left, ``+Z`` up — the
frame of the legacy MATLAB solver)::

    (x, y, z)_analyzer = (z, x, y)_shared

so results are identical to the old per-corner CSVs. A rear parse leaves
``steer_sweep`` at its 0 default because only the front axle steers.

Legacy CSV format (deprecated)
------------------------------
The old per-corner ``data/*.csv`` format — header ``,x,y,z``, rows of
``<point name>,<x>,<y>,<z>`` (hardpoints) or ``<config name>,<value>,,``
(configuration) — still parses, but emits a ``DeprecationWarning``. ``axle``
is ignored for legacy files (they hold a single corner).
"""
from __future__ import annotations

import csv
import io
import re
import warnings
from pathlib import Path

import numpy as np

from .geometry import Config, Geometry, SuspensionData

# ---- Legacy CSV: config row (lowercased substring) -> Config field ----
_LEGACY_CONFIG_ROWS = {
    "shock bump": "bump",
    "shock droop": "droop",
    "wheel size": "wheel_size",
    "steering rack": "steer_sweep",
    "shock lower mount": "shock_mount_lca",
    "wheelbase": "wheelbase",
}

# ---- Shared format: point base name -> legacy row name (frame handled below) ----
_SHARED_POINTS = {
    "LCA_front": "Lower wishbone front pivot",
    "LCA_rear": "Lower wishbone rear pivot",
    "LCA_outer": "Lower wishbone outer ball joint",
    "UCA_front": "Upper wishbone front pivot",
    "UCA_rear": "Upper wishbone rear pivot",
    "UCA_outer": "Upper wishbone outer ball joint",
    "shock_lower": "Lower spring pivot point",
    "shock_upper": "Upper spring pivot point",
    "trackrod_outer": "Outer track rod ball joint",
    "trackrod_inner": "Inner track rod ball joint",
    "spindle": "Wheel spindle point",
    "wheel_center": "Wheel centre point",
    "axle_outer": "Outer axle joint",
    "axle_inner": "Inner axle joint",
}

# 12 points required by Geometry.from_mapping; the axle joints are optional.
_SHARED_CORE = [b for b in _SHARED_POINTS if not b.startswith("axle_")]

# Shared config base name (after stripping _front/_rear) -> Config field.
_SHARED_CONFIG = {
    "shock_bump": "bump",
    "shock_droop": "droop",
    "wheel_size": "wheel_size",
    "shock_lower_mount": "shock_mount_lca",
}

_AXLE_PREFIX = {"front": "F", "rear": "R"}
_AXLE_SUFFIX = {"front": "_front", "rear": "_rear"}

_VAR_RE = re.compile(r'^\s*"([^"]+)"\s*=\s*(-?[0-9]*\.?[0-9]+)\s*[A-Za-z]*\s*$')


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


def _is_shared_text(text: str) -> bool:
    """True if the file uses the shared ``"name" = value`` syntax."""
    for line in text.splitlines():
        if line.strip():
            return line.lstrip().startswith('"')
    return False


def detect_format(path) -> str:
    """Classify a file as ``"shared"`` (datanew/*.txt) or ``"legacy"`` (data/*.csv)."""
    text = Path(path).read_text(encoding="utf-8-sig")
    return "shared" if _is_shared_text(text) else "legacy"


def parse_csv(path, axle: str | None = None) -> SuspensionData:
    """Parse a suspension file into a ``SuspensionData`` for one axle.

    Shared front+rear files (``datanew/*.txt``) require ``axle="front"`` or
    ``axle="rear"``. Deprecated legacy per-corner CSVs are accepted without
    ``axle`` and emit a ``DeprecationWarning``.
    """
    path = Path(path)
    text = path.read_text(encoding="utf-8-sig")
    if _is_shared_text(text):
        if axle not in _AXLE_PREFIX:
            raise ValueError(
                f"{path} is a shared front+rear file: "
                f"pass axle='front' or axle='rear'"
            )
        return _parse_shared(text, axle)
    warnings.warn(
        "legacy per-corner CSV format (data/*.csv) is deprecated; "
        "convert to the shared datanew/*.txt format and choose axle='front' or "
        "axle='rear'",
        DeprecationWarning,
        stacklevel=2,
    )
    return _parse_legacy(text)


def _build(points: dict[str, np.ndarray], values: dict[str, float]) -> SuspensionData:
    """Assemble Geometry + Config; fields absent from ``values`` keep defaults
    and are tracked in ``Config.missing``."""
    config = Config()
    for fld, val in values.items():
        setattr(config, fld, float(val))
    config.shock_mount_lca = int(config.shock_mount_lca)
    config.missing = tuple(
        f for f in Config.__dataclass_fields__ if f not in values and f != "missing"
    )
    return SuspensionData(geometry=Geometry.from_mapping(points), config=config)


def _parse_shared(text: str, axle: str) -> SuspensionData:
    """Parse the shared ``"name" = value`` file for one axle (front|rear)."""
    prefix, suffix = _AXLE_PREFIX[axle], _AXLE_SUFFIX[axle]

    variables: dict[str, float] = {}
    for line in text.splitlines():
        m = _VAR_RE.match(line)
        if m is not None:
            variables[m.group(1)] = float(m.group(2))

    # Config: per-axle values carry the axle suffix; steering_rack is shared
    # and belongs to the front axle (rear parses leave steer_sweep = 0).
    values: dict[str, float] = {}
    # Points: axis variables <F|R>_<point>_<axis>, shared frame.
    coords: dict[str, dict[str, float]] = {}
    for var, val in variables.items():
        if var == "steering_rack":
            if axle == "front":
                values["steer_sweep"] = val
        elif var.endswith(suffix):
            base = var[: -len(suffix)]
            if base in _SHARED_CONFIG:
                values[_SHARED_CONFIG[base]] = val
        elif var.startswith(prefix + "_"):
            stem, _, axis = var[len(prefix) + 1 :].rpartition("_")
            if stem in _SHARED_POINTS and axis in "xyz":
                coords.setdefault(stem, {})[axis] = val

    missing = [b for b in _SHARED_CORE if b not in coords]
    if missing:
        raise ValueError("Shared file missing hardpoints: " + ", ".join(missing))

    points: dict[str, np.ndarray] = {}
    for base, legacy_row in _SHARED_POINTS.items():
        c = coords.get(base)
        if c is None:
            continue  # optional axle joint absent
        miss = [a for a in "xyz" if a not in c]
        if miss:
            raise ValueError(f"{prefix}_{base} missing coordinate{'s' if len(miss) > 1 else ''}: {', '.join(miss)}")
        # Shared frame (+X left, +Y up, +Z front) -> analyzer frame
        # (+X forward, +Y left, +Z up): analyzer = (z, x, y)_shared.
        points[_clean_name(legacy_row)] = np.asarray(
            [c["z"], c["x"], c["y"]], dtype=float
        )
    return _build(points, values)


def _parse_legacy(text: str) -> SuspensionData:
    """Parse the deprecated per-corner CSV format."""
    points: dict[str, np.ndarray] = {}
    values: dict[str, float] = {}

    rows = list(csv.reader(io.StringIO(text)))
    for row in rows:
        if not row:
            continue
        name = (row[0] or "").strip()
        if not name:
            continue

        lower = name.lower()
        matched = next((k for k in _LEGACY_CONFIG_ROWS if k in lower), None)
        if matched is not None:
            values[_LEGACY_CONFIG_ROWS[matched]] = float(row[1])
        else:
            vals = [float(v) for v in row[1:4] if v.strip() != ""]
            if len(vals) == 3 and all(np.isfinite(vals)):
                points[_clean_name(name)] = np.asarray(vals, dtype=float)

    return _build(points, values)
