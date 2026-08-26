"""Tests for the optimizer modules (ports of sussy_optimize.m / sussy_tie_on_arm.m)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from sussyanal.kinematics import optimize, optimize_tie

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


def test_tie_optimizer_converges():
    res = optimize_tie(DATA, n_steps=10, max_iter=80)
    assert np.isfinite(res.max_err)
    assert res.opt_point.shape == (3,)
    assert len(res.toe_vals) == 10
    # bounded to the LCA pivot box +- 5 (documented deviation from MATLAB)
    lo = np.minimum(res.model.lca.front, res.model.lca.rear) - 5.0
    hi = np.maximum(res.model.lca.front, res.model.lca.rear) + 5.0
    assert np.all(res.opt_point >= lo - 1e-9)
    assert np.all(res.opt_point <= hi + 1e-9)
    # improves over the MATLAB-style starting point
    x0 = 0.7 * res.model.lca.rear + 0.3 * res.model.lca.outer_init
    from sussyanal.kinematics.tie_on_arm import _eval_bump_steer
    base = _eval_bump_steer(res.model, x0, 10)[0]
    assert res.max_err <= base + 1e-9
