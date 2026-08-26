"""3-D force visualization (port of ``visualize_forces.m``)."""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ..forces.run_quasistatic import QuasistaticResult
from . import common


def forces_figure(result: QuasistaticResult) -> go.Figure:
    gs = result.geom_step
    f = result.forces
    loads = result.loads
    params = result.params

    p_lf, p_lr = gs.lca_front, gs.lca_rear
    p_uf, p_ur = gs.uca_front, gs.uca_rear
    b_l, b_u = gs.lca_outer, gs.uca_outer
    s_u, s_l = gs.shock_upper, gs.shock_lower
    tr_i, b_t = gs.tie_inner, gs.tie_outer
    wc = gs.wheel_center

    fig = go.Figure()
    line = dict(width=3)

    # geometry links
    fig.add_trace(go.Scatter3d(x=[p_lf[0], b_l[0], p_lr[0]], y=[p_lf[1], b_l[1], p_lr[1]],
                               z=[p_lf[2], b_l[2], p_lr[2]], mode="lines", line=dict(color="blue", width=2), name="LCA"))
    fig.add_trace(go.Scatter3d(x=[p_uf[0], b_u[0], p_ur[0]], y=[p_uf[1], b_u[1], p_ur[1]],
                               z=[p_uf[2], b_u[2], p_ur[2]], mode="lines", line=dict(color="red", width=2), name="UCA"))
    fig.add_trace(go.Scatter3d(x=[b_l[0], b_u[0]], y=[b_l[1], b_u[1]], z=[b_l[2], b_u[2]],
                               mode="lines", line=dict(color="black", width=4), name="Upright"))
    fig.add_trace(go.Scatter3d(x=[tr_i[0], b_t[0]], y=[tr_i[1], b_t[1]], z=[tr_i[2], b_t[2]],
                               mode="lines", line=dict(color="green", width=2), name="Tie Rod"))
    fig.add_trace(go.Scatter3d(x=[s_u[0], s_l[0]], y=[s_u[1], s_l[1]], z=[s_u[2], s_l[2]],
                               mode="lines", line=dict(color="magenta", width=3), name="Shock"))

    if gs.hub_axis is not None:
        verts, tri = common.wheel_mesh(params.wheel_diameter, 8.0)
        wx, wy, wz = common.transform_points(verts, wc, gs.hub_axis)
        fig.add_trace(go.Mesh3d(x=wx, y=wy, z=wz, i=tri[:, 0], j=tri[:, 1], k=tri[:, 2],
                                color=common.COLORS["wheel"], opacity=0.3, name="Wheel", showscale=False))

    # vector scale (MATLAB: target ~10 in for max force)
    mags = [np.linalg.norm(loads.f), abs(f.f_shock), abs(f.f_tie),
            np.linalg.norm(f.f_lca_outer), np.linalg.norm(f.f_uca_outer)]
    max_f = max(mags)
    if max_f < 1.0:
        max_f = 1.0
    s = 10.0 / max_f

    p_app = wc + loads.offset
    common.add_arrow(fig, p_app, loads.f, "black", "Applied Load", scale=s)

    u_shock_down = (s_l - s_u) / np.linalg.norm(s_l - s_u)
    common.add_arrow(fig, s_l, f.f_shock * u_shock_down, "magenta", "Shock Force", scale=s)

    u_tie_in = (tr_i - b_t) / np.linalg.norm(tr_i - b_t)
    common.add_arrow(fig, b_t, f.f_tie * u_tie_in, "green", "Tie Rod Force", scale=s)

    common.add_arrow(fig, b_l, f.f_lca_outer, "blue", "LCA Outer", scale=s)
    common.add_arrow(fig, b_u, f.f_uca_outer, "red", "UCA Outer", scale=s)
    common.add_arrow(fig, p_lf, -f.f_lca_front, "blue", "LCA Front", scale=s)
    common.add_arrow(fig, p_lr, -f.f_lca_rear, "blue", "LCA Rear", scale=s)
    common.add_arrow(fig, p_uf, -f.f_uca_front, "red", "UCA Front", scale=s)
    common.add_arrow(fig, p_ur, -f.f_uca_rear, "red", "UCA Rear", scale=s)

    # applied moment arc
    m_vec = loads.m
    m_mag = float(np.linalg.norm(m_vec))
    if m_mag > 1.0:
        ax = m_vec / m_mag
        arb = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        u_m = common.unit(np.cross(ax, arb))
        v_m = np.cross(ax, u_m)
        theta = np.linspace(0.0, 1.5 * np.pi, 30)
        r_m = 4.0
        arc = p_app[:, None] + r_m * (np.cos(theta)[None, :] * u_m[:, None] + np.sin(theta)[None, :] * v_m[:, None])
        fig.add_trace(go.Scatter3d(x=arc[0], y=arc[1], z=arc[2], mode="lines",
                                   line=dict(color="black", width=2), name="Applied Moment"))

    common.layout3d(fig, title="Suspension Force Analysis")
    return fig
