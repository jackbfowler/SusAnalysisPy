"""Kinematic curve + surface figures (ports of MATLAB ``create_plots``)."""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ..kinematics.solver import KinematicResults

_METRICS = [
    ("camber", "Camber", "deg"),
    ("toe", "Toe / Steer Angle", "deg"),
    ("caster", "Caster", "deg"),
    ("kpi", "KPI", "deg"),
    ("scrub", "Scrub", "in"),
    ("trail", "Trail", "in"),
    ("wheel_travel", "Wheel Travel", "in"),
    ("motion_ratio", "Motion Ratio", "MR"),
]


def _series(results: KinematicResults, key: str) -> np.ndarray:
    return getattr(results, key)


def curve_figure(results: KinematicResults, x_axis: str = "shock") -> go.Figure:
    """1-D curves vs shock (or wheel) travel, 4x2 subplots."""
    n = len(_METRICS)
    fig = make_subplots(rows=4, cols=2, subplot_titles=[m[1] for m in _METRICS])

    if x_axis == "wheel":
        x = _series(results, "wheel_travel")[0, :]
        x_title = "Wheel Travel (in)"
    else:
        x = results.shock_travel_axis
        x_title = "Shock Travel (in)"

    for k, (key, _, unit) in enumerate(_METRICS):
        y = _series(results, key)[0, :]
        row, col = divmod(k, 2)
        fig.add_trace(
            go.Scatter(x=x, y=y, mode="lines", name=key, line=dict(width=2)),
            row=row + 1,
            col=col + 1,
        )
        fig.update_xaxes(title_text=x_title, row=row + 1, col=col + 1)
        fig.update_yaxes(title_text=unit, row=row + 1, col=col + 1)

    fig.update_layout(height=900, showlegend=False, title_text="Suspension Kinematics")
    return fig


def surfaces_figure(results: KinematicResults) -> go.Figure:
    """3x3 grid of surfaces for a 2-D (shock x steer) sweep."""
    keys = [m[0] for m in _METRICS]
    n = len(keys)
    fig = make_subplots(
        rows=3, cols=3, subplot_titles=[m[1] for m in _METRICS],
        specs=[[{"type": "surface"}] * 3 for _ in range(3)],
    )
    x = results.shock_travel_axis
    y = results.rack_travel_axis
    for k, key in enumerate(keys):
        z = _series(results, key)
        row, col = divmod(k, 3)
        fig.add_trace(
            go.Surface(x=x, y=y, z=z, name=key, showscale=False),
            row=row + 1,
            col=col + 1,
        )
        fig.update_xaxes(title_text="Shock", row=row + 1, col=col + 1)
        fig.update_yaxes(title_text="Rack", row=row + 1, col=col + 1)
    fig.update_layout(height=900, title_text="Suspension Analysis (Surface)")
    return fig
