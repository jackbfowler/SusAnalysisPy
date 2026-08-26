"""Smoke tests for the Plotly figures (build without error, correct structure)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from sussyanal.geometry import SuspensionModel
from sussyanal.io import parse_csv
from sussyanal.kinematics import solve_sweep
from sussyanal.plotting import kinematics as kin_plot
from sussyanal.plotting import page as page_module
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


def test_envelope_figure_2d_static_and_live():
    _, res = _results()

    static = kin_plot.envelope_figure(res, live=False)
    assert not hasattr(static, "_envelope_config")
    # per metric: steering family + current line; plus one colorbar carrier
    assert len(static.data) == 9 * (res.n_steer_steps + 1) + 1

    live = kin_plot.envelope_figure(res, live=True)
    assert len(live.data) == 9 * (res.n_steer_steps + 2)  # + red point per metric
    cfg = live._envelope_config
    assert set(cfg) == {"metrics"}
    assert len(cfg["metrics"]) == 9
    for m in cfg["metrics"]:
        assert 0 <= m["line"] < m["point"] < len(live.data)
        assert set(m) == {"key", "line", "point", "x", "data"}
        assert len(m["data"]) == res.n_steer_steps
        assert len(m["data"][0]) == res.n_shock_steps


def test_envelope_steer_lines_colorcoded():
    _, res = _results()
    fig = kin_plot.envelope_figure(res, live=False)
    first = fig.data[0].line.color            # steer 0 (full negative)
    last = fig.data[res.n_steer_steps - 1].line.color  # full positive steer
    assert first != last and first != "rgb(217,217,217)"
    # no min/max triangles anywhere
    for t in fig.data:
        marker = getattr(t, "marker", None)
        if marker is not None and marker.symbol in ("triangle-down", "triangle-up"):
            raise AssertionError("min/max triangle marker still present")
    # standalone plot carries a steering colorbar
    assert any(getattr(t, "marker", None) is not None and t.marker.showscale for t in fig.data)


def test_analyze_page_assembles():
    model, res = _results()
    live_env = kin_plot.envelope_figure(res, live=True)
    shock_idxs, mid_steer, mid_shock = suspension3d.viewer_indices(res)
    page = page_module.analyze_page(
        suspension3d.suspension_figure(model, res),
        live_env,
        live_env._envelope_config,
        shock_idxs,
        mid_steer,
        mid_shock,
    )
    assert 'id="viewer3d"' in page
    assert 'id="envelope"' in page
    assert "plotly_sliderend" in page
    assert "100vh" in page        # viewer fills the first viewport
    assert page.count("Plotly.newPlot") == 2  # both figures on one page
    # the envelope figure must come after the viewer in document order
    assert page.index('id="envelope"') > page.index('id="viewer3d"')
