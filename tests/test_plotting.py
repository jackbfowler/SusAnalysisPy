"""Smoke tests for the Plotly figures (build without error, correct structure)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from sussyanal.geometry import SuspensionModel
from sussyanal.io import parse_csv
from sussyanal.kinematics import solve_sweep
from sussyanal.plotting import kinematics as kin_plot
from sussyanal.plotting import suspension3d

DATA = Path(__file__).resolve().parent.parent / "data" / "2026BajaFront_1-20.csv"


def _results(n_shock: int = 30):
    data = parse_csv(DATA)
    model = SuspensionModel.from_data(data)
    return model, solve_sweep(model, n_shock_steps=n_shock)


def test_suspension3d_2d_has_shock_and_steer_frames():
    model, res = _results()
    assert res.is_2d
    fig = suspension3d.suspension_figure(model, res)
    names = [f.name for f in fig.frames]
    assert any(n.startswith("h") for n in names)   # shock frames
    assert any(n.startswith("s") for n in names)   # steering frames
    assert len(fig.layout.sliders) == 2            # shock + steering sliders


def test_suspension3d_1d_has_only_shock_frames():
    data = parse_csv(Path(__file__).resolve().parent.parent / "data" / "2024BajaRear.csv")
    model = SuspensionModel.from_data(data)
    res = solve_sweep(model, n_shock_steps=20)
    assert not res.is_2d
    fig = suspension3d.suspension_figure(model, res)
    assert len(fig.layout.sliders) == 1
    assert all(f.name.startswith("h") for f in fig.frames)


def test_envelope_figure_2d():
    _, res = _results()
    fig = kin_plot.envelope_figure(res)
    # one trace per steer step per metric (9 metrics), plus 4 overlay traces
    assert len(fig.data) >= 9 * res.n_steer_steps + 9 * 4
    # sync config present and structurally valid
    cfg = fig._sync_config
    assert len(cfg["metrics"]) == 9
    assert len(cfg["steerTravel"]) == res.n_steer_steps
    assert len(cfg["shockTravel"]) == res.n_shock_steps
    for m in cfg["metrics"]:
        assert 0 <= m["line"] < m["point"] < m["min"] < m["max"] < len(fig.data)
        assert len(m["data"]) == res.n_steer_steps
        assert len(m["data"][0]) == res.n_shock_steps


def test_viewer_indices_match_frames():
    model, res = _results()
    shock_idxs, mid_steer, mid_shock = suspension3d.viewer_indices(res)
    assert mid_shock in shock_idxs
    fig = suspension3d.suspension_figure(model, res)
    frame_names = {f.name for f in fig.frames}
    assert {f"h{i}" for i in shock_idxs} <= frame_names


def test_sync_scripts_build():
    from sussyanal.plotting import sync

    sender = sync.sender_script([0, 5, 10, 15], mid_steer=10, mid_shock=8)
    assert "BroadcastChannel" in sender and "plotly_sliderend" in sender
    assert '"shock": [0, 5, 10, 15]' in sender

    receiver = sync.receiver_script({"metrics": [], "steerTravel": [], "shockTravel": []})
    assert "BroadcastChannel" in receiver and "Plotly.restyle" in receiver
    assert "kind: \"hello\"" in receiver
