"""Kinematic curve + surface figures (ports of MATLAB ``create_plots``)."""
from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go
from plotly.colors import sample_colorscale
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

# Component-analysis metrics (MATLAB Figure 3/4): axle + arm articulation angles.
_COMPONENT_METRICS = [
    ("plunge", "Axle Plunge", "in"),
    ("cv_out", "Outer CV", "deg"),
    ("cv_in", "Inner CV", "deg"),
    ("lca_angle", "LCA Articulation Delta", "deg"),
    ("uca_angle", "UCA Articulation Delta", "deg"),
    ("lca_angle_abs", "LCA Articulation Abs", "deg"),
    ("uca_angle_abs", "UCA Articulation Abs", "deg"),
]

# target labeled tick count per axis (roughly double Plotly's auto density)
_TARGET_TICKS = 10


def _nice_tick_step(span: float, target_ticks: int) -> float:
    """A 'nice' tick step (1/2/2.5/5 x 10^k) giving ~target_ticks intervals."""
    if span <= 0:
        return 1.0
    raw = span / max(target_ticks, 1)
    mag = 10.0 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if m * mag >= raw - 1e-12:
            return m * mag
    return 10.0 * mag


def _nice_tick0(vmin: float, step: float) -> float:
    return math.floor(vmin / step) * step


def _series(results: KinematicResults, key: str) -> np.ndarray:
    return getattr(results, key)


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


def envelope_figure(
    results: KinematicResults,
    live: bool = False,
    metrics: list | None = None,
    colorbar: bool = True,
    title: str = "",
) -> go.Figure:
    """Envelope plots (MATLAB ``do_envelope``): shock travel on the x-axis with
    one line per steering step, **colorcoded by steering position** (Plotly
    ``sunsetdark``, full negative steer -> full positive steer).

    ``metrics`` is the list of ``(key, name, unit)`` to plot (default: the
    suspension metrics). ``live=True`` stores the restyle config on the figure
    as ``fig._envelope_config`` so the combined analyze page can update the
    overlays when the 3-D viewer's sliders move; the live figure keeps a bold
    current-steer line plus a red current-point marker. ``live=False`` is the
    static standalone output: **no** bold zero-steer highlight, every steering
    line is hoverable with the steering angle shown in the tooltip, and a
    steering colorbar is drawn on the right unless ``colorbar=False``.
    ``title`` sets the figure title text.

    1-D sets (no steering sweep) render a single line per metric — no steering
    family and no colorbar — so the same figure doubles as the live overlay
    for the zero-steer case.
    """
    if metrics is None:
        metrics = _ENVELOPE_METRICS
    cols = math.ceil(math.sqrt(len(metrics)))
    rows = math.ceil(len(metrics) / cols)
    fig = make_subplots(rows=rows, cols=cols, subplot_titles=[m[1] for m in metrics])
    x = results.shock_travel_axis
    ns = results.n_steer_steps
    mid = math.ceil(ns / 2) - 1
    mid_shock = int(np.argmin(np.abs(results.shock_travel_axis)))
    rack = [float(v) for v in results.rack_travel_axis]

    # color per steering step: negative steer -> one end of sunsetdark
    positions = [s / (ns - 1) if ns > 1 else 0.5 for s in range(ns)]
    steer_colors = sample_colorscale("sunsetdark", positions)

    metrics_cfg = []
    # shared x-axis (shock travel): a nice step giving ~_TARGET_TICKS labels
    dx = _nice_tick_step(float(x[-1] - x[0]), _TARGET_TICKS)
    x0t = _nice_tick0(float(np.min(x)), dx)

    for k, (key, name, unit) in enumerate(metrics):
        z = _series(results, key)
        row, col = divmod(k, cols)

        # steering family: one line per steering step, colorcoded by steer.
        # Static figures: every line is hoverable and the tooltip shows the
        # steering angle of that line. Live figures: the family is a backdrop
        # (hover off) behind the bold current line + red dot.
        # (1-D sets have a single step: the family is one line, skip it)
        if ns > 1:
            for s in range(ns):
                fig.add_trace(
                    go.Scatter(
                        x=x, y=z[s, :], mode="lines",
                        line=dict(color=steer_colors[s], width=1.2),
                        name=f"{name} (steer {s})",
                        showlegend=False,
                        hoverinfo="skip" if live else "x+y",
                        hovertemplate=None if live else (
                            f"{name}: %{{y:.3f}} {unit}<br>"
                            f"Steering: {rack[s]:.2f} in<extra></extra>"
                        ),
                    ),
                    row=row + 1, col=col + 1,
                )

        # bold current line: the live-page highlight, and the only line for
        # 1-D sets. Static 2-D figures have no bold zero-steer line.
        if live or ns == 1:
            fig.add_trace(
                go.Scatter(x=x, y=z[mid, :], mode="lines",
                           line=dict(color="rgb(31,119,180)", width=2.5),
                           name=f"{name} (current steer)", showlegend=False,
                           hoverinfo="x+y",
                           hovertemplate=None if ns > 1 else (
                               f"{name}: %{{y:.3f}} {unit}<extra></extra>"
                           )),
                row=row + 1, col=col + 1,
            )
            line_idx = len(fig.data) - 1
        else:
            line_idx = None

        if live:
            metrics_cfg.append(
                {
                    "key": key, "line": line_idx,
                    "x": [float(v) for v in x],
                    "data": [[float(v) for v in row] for row in z],
                }
            )

        # ~2x the labeled ticks on both axes (explicit linear tick spacing)
        zmin = float(np.nanmin(z))
        zspan = float(np.nanmax(z) - zmin)
        dz = _nice_tick_step(zspan, _TARGET_TICKS)
        z0t = _nice_tick0(zmin, dz)
        fig.update_xaxes(tickmode="linear", dtick=dx, tick0=x0t,
                         title_text="Shock Travel (in)", row=row + 1, col=col + 1)
        fig.update_yaxes(tickmode="linear", dtick=dz, tick0=z0t,
                         title_text=unit, row=row + 1, col=col + 1)

    if live:
        # current shock-travel indicator: one vertical line per subplot,
        # moved with Plotly.relayout (layout-level shapes — far more robust
        # than restyling a marker trace, which Plotly can silently misindex).
        for k in range(len(metrics)):
            axis = k + 1
            xref = "x" if axis == 1 else f"x{axis}"
            yref = "y domain" if axis == 1 else f"y{axis} domain"
            fig.add_shape(
                dict(
                    type="line", x0=float(x[mid_shock]), x1=float(x[mid_shock]),
                    y0=0, y1=1, xref=xref, yref=yref,
                    line=dict(color="red", width=2),
                )
            )
        for k, m in enumerate(metrics_cfg):
            m["shape"] = k  # shapes[k] corresponds to metric k (added in order)

    if not live and colorbar and ns > 1:
        # steering colorbar (invisible dummy markers carry the scale)
        fig.add_trace(
            go.Scatter(x=[float("nan")] * ns, y=[float("nan")] * ns, mode="markers",
                       marker=dict(color=rack, colorscale="sunsetdark", showscale=True,
                                   cmin=min(rack), cmax=max(rack),
                                   colorbar=dict(title="Steering (in)", thickness=14),
                                   size=1, opacity=0),
                       showlegend=False, hoverinfo="skip"),
            row=1, col=1,
        )

    if live:
        # (readout removed — the sliders already show the current values)
        fig._envelope_config = {"metrics": metrics_cfg}

    fig.update_layout(height=900, title_text=title)
    return fig


def component_figure(
    results: KinematicResults,
    live: bool = False,
    title: str = "",
) -> go.Figure:
    """Static component-analysis envelope (MATLAB Figure 4): axle plunge, CV
    angles, and LCA/UCA articulation angles, in the same colorcoded-envelope
    style as :func:`envelope_figure` (same 3x3 grid, spacing, and steering
    colorbar). Axle metrics are omitted when the set has no axle.
    """
    metrics = [m for m in _COMPONENT_METRICS if _series(results, m[0]) is not None]
    return envelope_figure(results, live=live, metrics=metrics, colorbar=True, title=title)
