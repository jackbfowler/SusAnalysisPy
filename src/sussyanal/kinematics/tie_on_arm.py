"""Inner tie-rod mount optimizer for bump steer (port of ``sussy_tie_on_arm.m``).

The inner tie-rod ball joint is rigidly attached to the LCA; the optimizer
moves its static position to minimize the toe variation over shock travel.
``fminsearch`` (MATLAB) is replaced by an equivalent pure-numpy Nelder-Mead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ..geometry import (
    SuspensionModel,
    intersect_circle_sphere,
    rotate_point,
    unit,
    _perp_basis,
)
from ..io import parse_csv

Vec3 = np.ndarray


# --------------------------------------------------------------------------- #
# Nelder-Mead (equivalent to MATLAB fminsearch, with optional box bounds)
# --------------------------------------------------------------------------- #
def _nelder_mead(f, x0, max_iter=200, tol=1e-4, bounds=None):
    """Unconstrained Nelder-Mead; returns ``(best_point, best_value)``.

    If ``bounds`` is ``[(lo, hi), ...]`` per dimension, every candidate is
    clamped into the box before evaluation (the MATLAB ``sussy_tie_on_arm.m``
    computes such bounds but never applies them).
    """
    n = len(x0)
    x0 = np.asarray(x0, dtype=float).copy()

    def clamp(p):
        if bounds is None:
            return p
        return np.clip(p, [b[0] for b in bounds], [b[1] for b in bounds])

    sim = [x0]
    for i in range(n):
        p = x0.copy()
        p[i] = p[i] * 1.05 if p[i] != 0.0 else 0.00025
        sim.append(clamp(p))
    fs = [f(p) for p in sim]

    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5
    for _ in range(max_iter):
        order = np.argsort(fs)
        sim = [sim[i] for i in order]
        fs = [fs[i] for i in order]

        if np.std(fs) < tol:
            break
        x_bar = np.mean(sim[:-1], axis=0)

        x_r = clamp(x_bar + alpha * (x_bar - sim[-1]))
        f_r = f(x_r)
        if fs[0] <= f_r < fs[-2]:
            sim[-1], fs[-1] = x_r, f_r
        elif f_r < fs[0]:
            x_e = clamp(x_bar + gamma * (x_r - x_bar))
            f_e = f(x_e)
            if f_e < f_r:
                sim[-1], fs[-1] = x_e, f_e
            else:
                sim[-1], fs[-1] = x_r, f_r
        else:
            x_c = clamp(x_bar + rho * (sim[-1] - x_bar))
            f_c = f(x_c)
            if f_c < fs[-1]:
                sim[-1], fs[-1] = x_c, f_c
            else:
                for i in range(1, n + 1):
                    sim[i] = clamp(sim[0] + sigma * (sim[i] - sim[0]))
                    fs[i] = f(sim[i])

    order = np.argsort(fs)
    return sim[order[0]], fs[order[0]]


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def _signed_angle(v1: Vec3, v2: Vec3, n: Vec3) -> float:
    return math.atan2(float(np.dot(np.cross(v1, v2), n)), float(np.dot(v1, v2)))


def _wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _lca_rotation(model: SuspensionModel, cur_lca_outer: Vec3, delta_phi: float) -> float:
    """Rotation angle of the LCA about its own axis at the current step."""
    if model.config.shock_mount_lca == 1:
        return delta_phi  # LCA is the shock-driven arm
    lca = model.lca
    ax = lca.axis
    v_static = lca.outer_init - lca.center
    v_curr = cur_lca_outer - lca.center
    v1 = v_static - np.dot(v_static, ax) * ax
    v2 = v_curr - np.dot(v_curr, ax) * ax
    return _signed_angle(v1, v2, ax)


def _instant_center(model: SuspensionModel) -> Vec3:
    """Front-view (YZ) instant center of the two A-arms."""
    l_in, l_out = model.lca.front, model.lca.outer_init
    u_in, u_out = model.uca.front, model.uca.outer_init
    m_l = (l_out[2] - l_in[2]) / (l_out[1] - l_in[1])
    m_u = (u_out[2] - u_in[2]) / (u_out[1] - u_in[1])
    b_l = l_in[2] - m_l * l_in[1]
    b_u = u_in[2] - m_u * u_in[1]
    ic_y = (b_u - b_l) / (m_l - m_u)
    ic_z = m_l * ic_y + b_l
    return np.array([0.0, ic_y, ic_z])


# --------------------------------------------------------------------------- #
# Objective
# --------------------------------------------------------------------------- #
def _eval_bump_steer(model: SuspensionModel, p_static: Vec3, n_steps: int = 20):
    """Bump-steer evaluation for a candidate inner tie static position.

    Returns ``(max_err, toe_vals, shock_disp, lca_pts, uca_pts, in_tie, out_tie)``.
    """
    up = model.upright
    shock = model.shock
    tie_len = float(np.linalg.norm(up.tie_init - p_static))
    shock_disp = np.linspace(-model.config.bump, model.config.droop, n_steps)

    toe_vals = np.zeros(n_steps)
    lca_pts = np.zeros((3, n_steps))
    uca_pts = np.zeros((3, n_steps))
    in_tie = np.zeros((3, n_steps))
    out_tie = np.zeros((3, n_steps))

    for i, s_disp in enumerate(shock_disp):
        shock_len = shock.length_init + s_disp
        cur_lca, cur_uca, _, delta_phi = model.solve_arms(shock_len)
        lca_rot = _lca_rotation(model, cur_lca, delta_phi)
        cur_inner = rotate_point(p_static, model.lca.origin, model.lca.axis, lca_rot)

        kp = unit(cur_uca - cur_lca)
        tie_c = cur_lca + up.tie_axial * kp
        t1, t2 = intersect_circle_sphere(tie_c, kp, up.tie_rad, cur_inner, tie_len)
        p1, p2 = _perp_basis(kp)
        a1 = math.atan2(float(np.dot(t1 - tie_c, p2)), float(np.dot(t1 - tie_c, p1)))
        a2 = math.atan2(float(np.dot(t2 - tie_c, p2)), float(np.dot(t2 - tie_c, p1)))
        d1 = _wrap_pi(a1 - up.tie_ang)
        d2 = _wrap_pi(a2 - up.tie_ang)
        if abs(d1) < abs(d2):
            toe_vals[i] = math.degrees(d1)
            cur_tie = t1
        else:
            toe_vals[i] = math.degrees(d2)
            cur_tie = t2

        lca_pts[:, i] = cur_lca
        uca_pts[:, i] = cur_uca
        in_tie[:, i] = cur_inner
        out_tie[:, i] = cur_tie

    max_err = float(np.ptp(toe_vals) + 0.1 * np.max(np.abs(toe_vals)))
    return max_err, toe_vals, shock_disp, lca_pts, uca_pts, in_tie, out_tie


@dataclass
class TieResult:
    opt_point: np.ndarray      # optimal inner tie static XYZ (in)
    max_err: float             # objective: range + 0.1*max (deg)
    toe_vals: np.ndarray
    shock_disp: np.ndarray
    lca_pts: np.ndarray
    uca_pts: np.ndarray
    in_tie: np.ndarray
    out_tie: np.ndarray
    instant_center: np.ndarray
    model: SuspensionModel


def optimize_tie(csv, n_steps: int = 20, max_iter: int = 200, x0=None) -> TieResult:
    """Find the LCA-mounted inner tie position that minimizes bump steer.

    The search is bounded to the LCA pivot bounding box +-5 in (the bounds the
    reference MATLAB computes but never applies); an unbounded search drifts to
    physically meaningless locations on these geometries.
    """
    geometry, config = parse_csv(csv)
    model = SuspensionModel(geometry, config)
    if x0 is None:
        x0 = 0.7 * model.lca.rear + 0.3 * model.lca.outer_init

    lo = np.minimum(model.lca.front, model.lca.rear) - 5.0
    hi = np.maximum(model.lca.front, model.lca.rear) + 5.0
    bounds = list(zip(lo, hi))

    def f(p):
        return _eval_bump_steer(model, np.asarray(p, dtype=float), n_steps)[0]

    x_opt, fval = _nelder_mead(f, np.asarray(x0, dtype=float), max_iter=max_iter, bounds=bounds)
    max_err, toe_vals, shock_disp, lca_pts, uca_pts, in_tie, out_tie = _eval_bump_steer(
        model, x_opt, n_steps
    )
    return TieResult(
        opt_point=np.asarray(x_opt),
        max_err=max_err,
        toe_vals=toe_vals,
        shock_disp=shock_disp,
        lca_pts=lca_pts,
        uca_pts=uca_pts,
        in_tie=in_tie,
        out_tie=out_tie,
        instant_center=_instant_center(model),
        model=model,
    )


def tie_figure(res: TieResult) -> go.Figure:
    """Bump-steer curve + front-view geometry with instant center."""
    m = res.model
    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=["Bump Steer Curve (Optimized)", "Front View Geometry & IC"],
    )

    fig.add_trace(go.Scatter(x=res.shock_disp, y=res.toe_vals, mode="lines+markers",
                             name="toe", line=dict(width=2)), row=1, col=1)
    fig.update_xaxes(title_text="Shock Disp (in)", row=1, col=1)
    fig.update_yaxes(title_text="Toe (deg)", row=1, col=1)

    fig.add_trace(
        go.Scatter(x=[m.lca.front[1], m.lca.outer_init[1]], y=[m.lca.front[2], m.lca.outer_init[2]],
                   mode="lines+markers", name="LCA", line=dict(color="blue")), row=1, col=2)
    fig.add_trace(
        go.Scatter(x=[m.uca.front[1], m.uca.outer_init[1]], y=[m.uca.front[2], m.uca.outer_init[2]],
                   mode="lines+markers", name="UCA", line=dict(color="red")), row=1, col=2)
    fig.add_trace(
        go.Scatter(x=[res.opt_point[1], m.upright.tie_init[1]], y=[res.opt_point[2], m.upright.tie_init[2]],
                   mode="lines+markers", name="tie rod", line=dict(color="green", width=2)), row=1, col=2)
    ic = res.instant_center
    fig.add_trace(
        go.Scatter(x=[ic[1]], y=[ic[2]], mode="markers", name="IC",
                   marker=dict(symbol="x", size=12, color="black")), row=1, col=2)
    fig.add_trace(
        go.Scatter(x=[m.upright.tie_init[1], ic[1]], y=[m.upright.tie_init[2], ic[2]],
                   mode="lines", name="IC line", line=dict(color="black", dash="dash")), row=1, col=2)

    fig.update_xaxes(title_text="Y (in)", row=1, col=2)
    fig.update_yaxes(title_text="Z (in)", row=1, col=2, scaleanchor="x", scaleratio=1)
    fig.update_layout(height=560, title="Tie-on-Arm Optimization")
    return fig
