"""Core sanity tests for the SussyAnal Python port."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from sussyanal.geometry import SuspensionModel, intersect_circle_sphere, rotate_point, unit
from sussyanal.io import parse_csv
from sussyanal.kinematics import solve_sweep
from sussyanal.forces import run_quasistatic

DATA = Path(__file__).resolve().parent.parent / "data" / "2026BajaFront_1-20.csv"

# Shared-format pair first converted to datanew/ (front 12-16 + rear 3-3).
SHARED = Path(__file__).resolve().parent.parent / "datanew" / "2026Baja.txt"
LEGACY_FRONT = Path(__file__).resolve().parent.parent / "data" / "2026BajaFront_12-16.csv"
LEGACY_REAR = Path(__file__).resolve().parent.parent / "data" / "2026BajaRear_3-3.csv"


def test_unit():
    assert np.allclose(unit(np.array([3.0, 0.0, 0.0])), [1.0, 0.0, 0.0])


def test_rotate_point_90deg():
    p = np.array([1.0, 0.0, 0.0])
    r = rotate_point(p, np.zeros(3), np.array([0.0, 0.0, 1.0]), np.pi / 2)
    assert np.allclose(r, [0.0, 1.0, 0.0], atol=1e-9)


def test_intersect_circle_sphere():
    # circle radius 1 in XY plane at origin; sphere center (2,0,0) radius 2
    c = np.zeros(3)
    n = np.array([0.0, 0.0, 1.0])
    s = np.array([2.0, 0.0, 0.0])
    p1, p2 = intersect_circle_sphere(c, n, 1.0, s, 2.0)
    for p in (p1, p2):
        assert abs(np.linalg.norm(p[:2]) - 1.0) < 1e-9
        assert abs(p[2]) < 1e-9
        assert abs(np.linalg.norm(p - s) - 2.0) < 1e-9
    assert np.linalg.norm(p1 - p2) > 1e-6


def test_parse_csv():
    data = parse_csv(DATA)
    assert data.config.shock_mount_lca == 1
    assert data.config.wheel_size == 23.0
    assert data.config.wheelbase == 58.0
    assert data.config.missing == ()
    assert data.geometry.has_axle


def test_legacy_csv_deprecated():
    with pytest.warns(DeprecationWarning, match="deprecated"):
        parse_csv(DATA)


def test_shared_requires_axle():
    with pytest.raises(ValueError, match="axle='front' or axle='rear'"):
        parse_csv(SHARED)


def _assert_same_points(a, b, atol: float = 1e-9):
    for f in a.__dataclass_fields__:
        va, vb = getattr(a, f), getattr(b, f)
        if va is None or vb is None:
            assert (va is None) == (vb is None), f
        else:
            assert np.allclose(np.asarray(va, float), np.asarray(vb, float), atol=atol), f


def test_shared_front_matches_legacy_front():
    """New-format front parse must reproduce the old front CSV exactly."""
    shared = parse_csv(SHARED, axle="front")
    legacy = parse_csv(LEGACY_FRONT)
    _assert_same_points(shared.geometry, legacy.geometry)
    assert shared.config.bump == legacy.config.bump == 5.01
    assert shared.config.droop == legacy.config.droop == 2.99
    assert shared.config.wheel_size == legacy.config.wheel_size == 23.0
    assert shared.config.steer_sweep == legacy.config.steer_sweep == 1.3
    assert shared.config.shock_mount_lca == legacy.config.shock_mount_lca == 1
    assert shared.config.missing == legacy.config.missing == ("wheelbase",)


def test_shared_rear_matches_legacy_rear():
    """New-format rear parse must reproduce the old rear CSV geometry; the rear
    does not steer (legacy rack = 0), and wheelbase is no longer carried."""
    shared = parse_csv(SHARED, axle="rear")
    legacy = parse_csv(LEGACY_REAR)
    _assert_same_points(shared.geometry, legacy.geometry)
    assert shared.config.bump == 4.71
    assert shared.config.droop == 4.29
    assert shared.config.wheel_size == 23.0
    assert shared.config.shock_mount_lca == 1
    assert shared.config.steer_sweep == 0.0  # rear axle does not steer
    assert shared.config.wheelbase == 0.0    # removed from the shared file
    assert shared.config.missing == ("steer_sweep", "wheelbase")


def test_config_is_per_hardpoint_set():
    """Config parameters are specific to each hardpoint set's CSV."""
    front26 = parse_csv(DATA.parent / "2026BajaFront.csv")
    rear25 = parse_csv(DATA.parent / "2025BajaRear.csv")
    sparse = parse_csv(DATA.parent / "2024BajaRear.csv")

    assert front26.config.bump == 6.1 and front26.config.steer_sweep == 1.3
    assert front26.config.shock_mount_lca == 1
    assert front26.config.missing == ("wheelbase",)

    assert rear25.config.bump == 4.41 and rear25.config.shock_mount_lca == 0
    assert rear25.config.missing == ("wheelbase",)

    # defaults fall back only for rows the set omits, and are tracked
    assert sparse.config.bump == 5.0
    assert sparse.config.missing == ("steer_sweep", "shock_mount_lca", "wheelbase")


def test_solve_sweep_finite():
    data = parse_csv(DATA)
    model = SuspensionModel.from_data(data)
    res = solve_sweep(model, n_shock_steps=50)
    assert res.camber.shape == (res.n_steer_steps, 50)
    assert res.n_steer_steps > 1  # this CSV has steering
    assert np.all(np.isfinite(res.camber))
    assert np.all(np.isfinite(res.toe))
    assert np.all(np.isfinite(res.motion_ratio))


def test_forces_finite_and_positive():
    result = run_quasistatic(DATA)
    f = result.forces
    assert f.f_shock > 0
    assert f.f_tie > 0
    for v in (
        f.f_lca_outer,
        f.f_uca_outer,
        f.f_lca_front,
        f.f_lca_rear,
        f.f_uca_front,
        f.f_uca_rear,
    ):
        assert np.all(np.isfinite(v))
