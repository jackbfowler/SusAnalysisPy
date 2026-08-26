"""Core sanity tests for the SussyAnal Python port."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from sussyanal.geometry import SuspensionModel, intersect_circle_sphere, rotate_point, unit
from sussyanal.io import parse_csv
from sussyanal.kinematics import solve_sweep
from sussyanal.forces import run_quasistatic

DATA = Path(__file__).resolve().parent.parent / "data" / "2026BajaFront_1-20.csv"


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
    geometry, config = parse_csv(DATA)
    assert config.shock_mount_lca == 1
    assert config.wheel_size == 23.0
    assert config.wheelbase == 58.0
    assert geometry.has_axle


def test_solve_sweep_finite():
    geometry, config = parse_csv(DATA)
    model = SuspensionModel(geometry, config)
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
