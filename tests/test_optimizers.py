"""Tests for the hardpoint grid-search optimizer (port of sussy_optimize.m)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from sussyanal.kinematics import optimize

DATA = Path(__file__).resolve().parent.parent / "data" / "2026BajaFront_1-20.csv"


def test_optimize_1d_reduces_cost():
    res = optimize(
        DATA,
        opt_point="OuterTrackRodBallJoint",
        sweep_axes=(3,),
        sweep_range=(0.5,),
        sweep_steps=(11,),
        objective="bump_steer",
        n_shock_steps=20,
    )
    assert res.cost.shape == (11,)
    assert np.all(np.isfinite(res.cost))
    # the grid includes the base position, so the optimum cannot be worse
    assert res.opt_cost <= res.base_cost + 1e-9
    assert np.isfinite(res.opt_cost)


def test_optimize_2d_shape():
    res = optimize(
        DATA,
        opt_point="OuterTrackRodBallJoint",
        sweep_axes=(2, 3),
        sweep_range=(0.4, 0.4),
        sweep_steps=(7, 9),
        objective="bump_steer",
        n_shock_steps=15,
    )
    assert res.cost.shape == (7, 9)
    assert np.isfinite(res.opt_cost)
