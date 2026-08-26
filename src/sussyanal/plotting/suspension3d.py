"""Interactive 3-D suspension visualizer (Plotly) — port of the MATLAB
``create_vis`` figure with shock slider + steering slider.

Plotly animates a single frame axis, so for 2-D (steer x shock) data we
generate only the frames each slider needs:

- the **shock slider** sweeps all shock steps at the static steer, and
- the **steering slider** sweeps all steer steps at static ride height.

Moving one slider snaps the other back to its static position (standard
Plotly two-slider behavior); the full 2-D coupling is best viewed in the
surface / envelope plots.
"""
from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go

from ..geometry import SuspensionModel
from ..kinematics.solver import KinematicResults
from . import common


def _static_points(model: SuspensionModel):
    """Fixed chassis-side points, as an (N,3) array."""
    pts = [
        model.lca.front,
        model.lca.rear,
        model.uca.front,
        model.uca.rear,
        model.shock.upper,
        model.tierod.inner_static,
    ]
    if model.has_axle:
        pts.append(model.axle.inner)
    return np.vstack(pts)


def _state_traces(
    model: SuspensionModel,
    results: KinematicResults,
    steer_idx: int,
    shock_idx: int,
    wheel_verts: np.ndarray,
    wheel_tri: np.ndarray,
    static_pts: np.ndarray,
) -> list:
    lca_f, lca_r = model.lca.front, model.lca.rear
    uca_f, uca_r = model.uca.front, model.uca.rear
    shock_upper = model.shock.upper

    lo = results.lca_outer[:, steer_idx, shock_idx]
    uo = results.uca_outer[:, steer_idx, shock_idx]
    to = results.tie_outer[:, steer_idx, shock_idx]
    sl = results.shock_lower[:, steer_idx, shock_idx]
    sp = results.wheel_spindle[:, steer_idx, shock_idx]
    wc = results.wheel_center[:, steer_idx, shock_idx]
    hub = results.hub_axis[:, steer_idx, shock_idx]
    kg = results.kp_ground[:, steer_idx, shock_idx]
    cp = results.contact_patch[:, steer_idx, shock_idx]
    gz = results.ground_z[steer_idx, shock_idx]
    rack_y = float(model.tierod.inner_static[1] + results.rack_travel[steer_idx, shock_idx])

    traces: list = []

    # ground plane
    gx = [wc[0] - 20, wc[0] + 20, wc[0] + 20, wc[0] - 20]
    gy = [wc[1] - 20, wc[1] - 20, wc[1] + 20, wc[1] + 20]
    traces.append(
        go.Mesh3d(
            x=gx, y=gy, z=[gz] * 4, i=[0, 0], j=[1, 2], k=[2, 3],
            color=common.COLORS["ground"], opacity=0.2, name="ground",
            showscale=False, hoverinfo="skip",
        )
    )

    # static pivots
    traces.append(
        go.Scatter3d(
            x=static_pts[:, 0], y=static_pts[:, 1], z=static_pts[:, 2],
            mode="markers", marker=dict(size=5, color=common.COLORS["static_pt"], symbol="square"),
            name="fixed pivots", hoverinfo="skip",
        )
    )

    # A-arms (filled triangles)
    traces.append(
        go.Mesh3d(
            x=[lca_f[0], lca_r[0], lo[0]], y=[lca_f[1], lca_r[1], lo[1]],
            z=[lca_f[2], lca_r[2], lo[2]], i=[0], j=[1], k=[2],
            color=common.COLORS["lca"], opacity=0.3, name="LCA", showscale=False,
        )
    )
    traces.append(
        go.Mesh3d(
            x=[uca_f[0], uca_r[0], uo[0]], y=[uca_f[1], uca_r[1], uo[1]],
            z=[uca_f[2], uca_r[2], uo[2]], i=[0], j=[1], k=[2],
            color=common.COLORS["uca"], opacity=0.3, name="UCA", showscale=False,
        )
    )

    # kingpin + extension
    traces.append(
        go.Scatter3d(
            x=[lo[0], uo[0]], y=[lo[1], uo[1]], z=[lo[2], uo[2]],
            mode="lines", line=dict(color=common.COLORS["kp"], width=6), name="kingpin",
        )
    )
    if not np.isnan(kg).any():
        traces.append(
            go.Scatter3d(
                x=[lo[0], kg[0]], y=[lo[1], kg[1]], z=[lo[2], kg[2]],
                mode="lines", line=dict(color="red", width=2, dash="dash"),
                name="KP extension",
            )
        )
        traces.append(
            go.Scatter3d(
                x=[kg[0], kg[0]], y=[kg[1], cp[1]], z=[kg[2], kg[2]],
                mode="lines", line=dict(color="green", width=3), name="scrub",
            )
        )
        traces.append(
            go.Scatter3d(
                x=[kg[0], cp[0]], y=[cp[1], cp[1]], z=[kg[2], kg[2]],
                mode="lines", line=dict(color="blue", width=3), name="trail",
            )
        )
        traces.append(
            go.Scatter3d(
                x=[kg[0]], y=[kg[1]], z=[kg[2]], mode="markers",
                marker=dict(size=6, color="red", symbol="x"), name="KP ground",
            )
        )

    # shock / tie / axle / hub
    traces.append(
        go.Scatter3d(
            x=[shock_upper[0], sl[0]], y=[shock_upper[1], sl[1]], z=[shock_upper[2], sl[2]],
            mode="lines", line=dict(color=common.COLORS["shock"], width=5), name="shock",
        )
    )
    traces.append(
        go.Scatter3d(
            x=[model.tierod.inner_static[0], to[0]],
            y=[rack_y, to[1]],
            z=[model.tierod.inner_static[2], to[2]],
            mode="lines", line=dict(color=common.COLORS["tie"], width=3), name="tie rod",
        )
    )
    if model.has_axle:
        ax = results.axle_outer[:, steer_idx, shock_idx]
        traces.append(
            go.Scatter3d(
                x=[model.axle.inner[0], ax[0]], y=[model.axle.inner[1], ax[1]],
                z=[model.axle.inner[2], ax[2]], mode="lines",
                line=dict(color=common.COLORS["axle"], width=6), name="axle",
            )
        )
    traces.append(
        go.Scatter3d(
            x=[sp[0], wc[0]], y=[sp[1], wc[1]], z=[sp[2], wc[2]],
            mode="lines", line=dict(color=common.COLORS["hub"], width=3), name="hub axis",
        )
    )

    # ball joints + contact patch
    traces.append(
        go.Scatter3d(
            x=[lo[0], uo[0]], y=[lo[1], uo[1]], z=[lo[2], uo[2]],
            mode="markers", marker=dict(size=6, color="yellow", symbol="circle"),
            name="ball joints", hoverinfo="skip",
        )
    )
    traces.append(
        go.Scatter3d(
            x=[cp[0]], y=[cp[1]], z=[cp[2]], mode="markers",
            marker=dict(size=6, color="green", symbol="x"), name="contact patch",
        )
    )

    # wheel (mesh + rim)
    wx, wy, wz = common.transform_points(wheel_verts, wc, hub)
    traces.append(
        go.Mesh3d(
            x=wx, y=wy, z=wz, i=wheel_tri[:, 0], j=wheel_tri[:, 1], k=wheel_tri[:, 2],
            color=common.COLORS["wheel"], opacity=0.8, name="wheel", showscale=False,
        )
    )
    rx, ry, rz = common.rim_circle(wc, hub, model.config.wheel_size)
    traces.append(
        go.Scatter3d(
            x=rx, y=ry, z=rz, mode="lines", line=dict(color="white", width=3),
            name="rim", hoverinfo="skip",
        )
    )
    return traces


def suspension_figure(
    model: SuspensionModel,
    results: KinematicResults,
    steer_idx: int | None = None,
    n_shock_frames: int | None = None,
) -> go.Figure:
    """Interactive 3-D figure with shock slider (+ steering slider when 2-D).

    The shock slider sweeps all shock steps at the static steer; for 2-D data a
    steering slider sweeps all steer steps at static ride height.
    """
    if results.n_steer_steps > 1:
        steer_idxs = list(range(results.n_steer_steps))
        mid_steer = math.ceil(results.n_steer_steps / 2) - 1
    else:
        steer_idxs = [0]
        mid_steer = 0
    if steer_idx is None:
        steer_idx = mid_steer
    steer_idx = int(np.clip(steer_idx, 0, results.n_steer_steps - 1))

    if n_shock_frames is None:
        n_shock_frames = min(results.n_shock_steps, 40)
    shock_idxs = np.linspace(0, results.n_shock_steps - 1, n_shock_frames).round().astype(int).tolist()
    mid_shock = int(np.argmin(np.abs(results.shock_travel_axis)))
    if mid_shock not in shock_idxs:  # the steering slider needs a mid-shock frame
        shock_idxs.append(mid_shock)
        shock_idxs.sort()

    # coarser wheel mesh keeps the per-frame data small
    wheel_verts, wheel_tri = common.wheel_mesh(
        model.config.wheel_size, model.config.wheel_width, n_theta=24
    )
    static_pts = _static_points(model)

    def traces(s: int, h: int):
        return _state_traces(model, results, s, h, wheel_verts, wheel_tri, static_pts)

    # Frames: every (mid_steer, h) named h{h}, plus every (s, mid_shock) named s{s}.
    frames = [go.Frame(data=traces(mid_steer, h), name=f"h{h}") for h in shock_idxs]
    for s in steer_idxs:
        if s != mid_steer:
            frames.append(go.Frame(data=traces(s, mid_shock), name=f"s{s}"))

    fig = go.Figure(data=traces(steer_idx, mid_shock))
    common.layout3d(fig, title="Suspension Geometry (shock sweep; steering at static ride height)")
    fig.frames = frames

    shock_mid_pos = int(np.argmin(np.abs(np.array(shock_idxs) - mid_shock)))
    steer_mid_pos = int(np.argmin(np.abs(np.array(steer_idxs) - mid_steer)))

    # ---- shock slider: sweep at static steer (instant frame jump, no animation) ----
    shock_steps = [
        dict(
            method="animate",
            args=[
                [f"h{h}"],
                dict(mode="immediate", sliders=[dict(active=steer_mid_pos)]),
            ],
            label=f"{results.shock_travel_axis[h]:.2f}",
        )
        for h in shock_idxs
    ]

    sliders = [
        dict(
            active=shock_mid_pos,
            steps=shock_steps,
            currentvalue=dict(prefix="Shock travel: ", suffix=" in"),
            len=0.9, x=0.05, y=0.0,
        )
    ]

    # ---- steering slider (2-D only): sweep at static ride height ----
    if results.n_steer_steps > 1:
        steer_steps = []
        for s in steer_idxs:
            frame_ref = f"h{mid_shock}" if s == mid_steer else f"s{s}"
            steer_steps.append(
                dict(
                    method="animate",
                    args=[
                        [frame_ref],
                        dict(mode="immediate", sliders=[dict(active=shock_mid_pos)]),
                    ],
                    label=f"{results.rack_travel_axis[s]:.2f}",
                )
            )
        sliders.append(
            dict(
                active=steer_mid_pos,
                steps=steer_steps,
                currentvalue=dict(prefix="Steering (rack): ", suffix=" in"),
                len=0.9, x=0.05, y=-0.14,
            )
        )

    fig.update_layout(sliders=sliders)
    return fig
