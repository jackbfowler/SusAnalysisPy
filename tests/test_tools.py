"""Tests for tools/points_to_csv.py (LOTUS corner import -> shared datanew txt)."""
from __future__ import annotations

import csv
import subprocess
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

from sussyanal.io import parse_csv

REPO = Path(__file__).resolve().parent.parent
TOOL = REPO / "tools" / "points_to_csv.py"
# Pinned LOTUS sample (2027 REAR export). The repo-root pointsimport.txt is a
# live working file and may change, so tests use this committed fixture copy.
POINTSIMPORT = Path(__file__).resolve().parent / "fixtures" / "lotus_rear_2027.txt"
LEGACY_FRONT = REPO / "data" / "2026BajaFront_12-16.csv"
MM = 25.4


def _run(input_path: Path, output_path: Path, axle: str) -> None:
    subprocess.run(
        [sys.executable, str(TOOL), str(input_path), str(output_path), "--axle", axle],
        check=True,
        capture_output=True,
        text=True,
    )


def _point_rows(csv_path: Path) -> dict[str, list[float]]:
    """Legacy CSV -> {canonical row name: [x, y, z] (inches)}."""
    with csv_path.open(newline="", encoding="utf-8-sig") as fh:
        raw = list(csv.reader(fh))
    points = {}
    for r in raw:
        if not r or not r[0].strip():
            continue
        vals = [float(v) for v in r[1:4] if v.strip()]
        if len(vals) == 3:
            points[r[0].strip()] = vals
    return points


def _front_lotus_text(tmp_path: Path) -> Path:
    """20-row LOTUS-style mm export synthesized from a legacy front CSV."""
    by_name = _point_rows(LEGACY_FRONT)
    names = [
        "Lower wishbone front pivot", "Lower wishbone rear pivot",
        "Lower wishbone outer ball joint",
        "Upper wishbone front pivot", "Upper wishbone rear pivot",
        "Upper wishbone outer ball joint",
        "Damper wishbone end", "Damper body end",
        "Outer track rod ball joint", "Inner track rod ball joint",
        "Upper spring pivot point", "Lower spring pivot point",
        "Wheel spindle point", "Wheel centre point",
    ]
    rows = [by_name[n] for n in names]
    rows += [[0.0, 0.0, 0.0]] * 4                       # rows 15-18: C of G (skipped)
    rows += [by_name["Inner axle joint"], [0.0, 0.0, 0.0]]  # rows 19, 20
    p = tmp_path / "front_lotus.txt"
    p.write_text(
        "\n".join("\t".join(f"{v * MM:.4f}" for v in xyz) for xyz in rows) + "\n"
    )
    return p


def test_rear_import_creates_file_with_blank_front(tmp_path):
    out = tmp_path / "2027Baja.txt"
    _run(POINTSIMPORT, out, "rear")

    text = out.read_text()
    assert len(text.splitlines()) == 93  # 9 config + 84 point variables
    # rear corner filled, front corner blank
    assert '"R_wheel_center_x" = 26.402402' in text
    assert '"F_LCA_front_x" =' in text and '"F_LCA_front_x" = 6.2' not in text
    # config cannot come from a LOTUS export -> blank for manual fill
    assert '"steering_rack" =' in text and '"steering_rack" = 1.3' not in text

    # rear parses and matches the old 2027 rear CSV wheel centre
    rear = parse_csv(out, axle="rear")
    assert np.allclose(
        rear.geometry.wheel_centre_point,
        [-976.460 / MM, 670.621 / MM, 12.7 / MM],
        atol=1e-6,
    )
    # front is not analyzable until its corner has been imported
    with pytest.raises(ValueError, match="missing hardpoints"):
        parse_csv(out, axle="front")


def test_front_import_merges_and_preserves_rear(tmp_path):
    out = tmp_path / "2027Baja.txt"
    _run(POINTSIMPORT, out, "rear")
    _run(_front_lotus_text(tmp_path), out, "front")

    warnings.simplefilter("ignore", DeprecationWarning)
    front = parse_csv(out, axle="front")
    legacy = parse_csv(LEGACY_FRONT)
    for f in front.geometry.__dataclass_fields__:
        va, vb = getattr(front.geometry, f), getattr(legacy.geometry, f)
        if va is None or vb is None:
            assert (va is None) == (vb is None), f
        else:
            assert np.allclose(va, vb, atol=5e-6), f  # harness mm rounding

    rear = parse_csv(out, axle="rear")
    assert np.allclose(
        rear.geometry.wheel_centre_point,
        [-976.460 / MM, 670.621 / MM, 12.7 / MM],
        atol=1e-6,
    )

    names = [ln.split('"')[1] for ln in out.read_text().splitlines()]
    assert len(names) == len(set(names))  # no duplicate variables after merge
