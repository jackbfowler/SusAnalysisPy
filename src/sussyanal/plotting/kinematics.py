"""Kinematic curve + surface figures (ports of MATLAB ``create_plots``)."""
from __future__ import annotations

import math

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

_ENVELOPE_METRICS = _METRICS + [("track_change", "Track Change", "in")]


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


def envelope_figure(results: KinematicResults, live: bool = False) -> go.Figure:
    """Envelope plots (MATLAB ``do_envelope``): shock travel on the x-axis with
    one gray line per steering step, plus overlay traces for the current state —
    a bold current-steer line, min/max markers, and (``live=True``) a red
    current-point marker and readout annotation.

    ``live=True`` additionally stores the restyle config on the figure as
    ``fig._envelope_config`` so the combined analyze page can update the
    overlays when the 3-D viewer's sliders move. ``live=False`` is the static
    standalone output (no moving parts).
    """
    n = len(_ENVELOPE_METRICS)  # 9
    fig = make_subplots(rows=3, cols=3, subplot_titles=[m[1] for m in _ENVELOPE_METRICS])
    x = results.shock_travel_axis
    ns = results.n_steer_steps
    mid = math.ceil(ns / 2) - 1
    mid_shock = int(np.argmin(np.abs(results.shock_travel_axis)))

    metrics_cfg = []
    for k, (key, name, unit) in enumerate(_ENVELOPE_METRICS):
        z = _series(results, key)
        row, col = divmod(k, 3)

        # gray family: one line per steering step (never restyled)
        for s in range(ns):
            fig.add_trace(
                go.Scatter(x=x, y=z[s, :], mode="lines",
                           line=dict(color="rgb(217,217,217)", width=0.8),
                           name=f"{name} (steer {s})",
                           showlegend=False, hoverinfo="skip"),
                row=row + 1, col=col + 1,
            )

        # bold current-steer line
        fig.add_trace(
            go.Scatter(x=x, y=z[mid, :], mode="lines",
                       line=dict(color="rgb(31,119,180)", width=2.5),
                       name=f"{name} (current steer)", showlegend=False, hoverinfo="x+y"),
            row=row + 1, col=col + 1,
        )
        line_idx = len(fig.data) - 1

        # min/max markers of the current line
        i_min, i_max = int(np.argmin(z[mid, :])), int(np.argmax(z[mid, :]))
        fig.add_trace(
            go.Scatter(x=[x[i_min]], y=[z[mid, i_min]], mode="markers",
                       marker=dict(symbol="triangle-down", size=10, color="black"),
                       showlegend=False, hoverinfo="skip"),
            row=row + 1, col=col + 1,
        )
        min_idx = len(fig.data) - 1
        fig.add_trace(
            go.Scatter(x=[x[i_max]], y=[z[mid, i_max]], mode="markers",
                       marker=dict(symbol="triangle-up", size=10, color="black"),
                       showlegend=False, hoverinfo="skip"),
            row=row + 1, col=col + 1,
        )
        max_idx = len(fig.data) - 1

        if live:
            # red current-point marker (moved by the shock slider)
            fig.add_trace(
                go.Scatter(x=[x[mid_shock]], y=[z[mid, mid_shock]], mode="markers",
                           marker=dict(symbol="circle", size=10, color="red"),
                           name=f"{name} (current point)", showlegend=False, hoverinfo="skip"),
                row=row + 1, col=col + 1,
            )
            point_idx = len(fig.data) - 1
            metrics_cfg.append(
                {
                    "key": key, "line": line_idx, "point": point_idx,
                    "min": min_idx, "max": max_idx,
                    "x": [float(v) for v in x],
                    "data": [[float(v) for v in row] for row in z],
                }
            )

        fig.update_xaxes(title_text="Shock Travel (in)", row=row + 1, col=col + 1)
        fig.update_yaxes(title_text=unit, row=row + 1, col=col + 1)

    if live:
        fig.add_annotation(
            text=f"Steer: {results.rack_travel_axis[mid]:+.2f} in | "
                 f"Shock: {results.shock_travel_axis[mid_shock]:+.2f} in",
            xref="paper", yref="paper", x=0.0, y=1.05, showarrow=False,
            bgcolor="rgba(255,255,255,0.85)", bordercolor="black", borderpad=4,
        )
        fig._envelope_config = {
            "metrics": metrics_cfg,
            "steerTravel": [float(v) for v in results.rack_travel_axis],
            "shockTravel": [float(v) for v in x],
        }

    fig.update_layout(height=900, title_text="Envelope (one line per steering step)")
    return fig
