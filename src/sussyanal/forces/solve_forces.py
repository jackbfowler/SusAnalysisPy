"""Quasistatic suspension force solver (7x7 linear system).

Faithful port of ``SusAnalysis/SussyForces/solve_forces.m``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vec3 = np.ndarray  # shape (3,)


def _skew(v: Vec3) -> np.ndarray:
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


@dataclass
class GeomStep:
    """Geometry points for one suspension step (moving + fixed)."""
    lca_outer: Vec3
    uca_outer: Vec3
    tie_outer: Vec3
    shock_lower: Vec3
    wheel_center: Vec3
    lca_front: Vec3
    lca_rear: Vec3
    uca_front: Vec3
    uca_rear: Vec3
    shock_upper: Vec3
    tie_inner: Vec3
    hub_axis: Vec3


@dataclass
class Loads:
    f: Vec3        # global force at contact patch
    m: Vec3        # global moment at contact patch
    offset: Vec3   # wheel center -> application point


@dataclass
class ForceParams:
    shock_mount_lca: int
    wheel_diameter: float


@dataclass
class Forces:
    f_uca_outer: Vec3
    f_lca_outer: Vec3
    f_tie: float
    f_shock: float
    f_lca_front: Vec3
    f_lca_rear: Vec3
    f_uca_front: Vec3
    f_uca_rear: Vec3


def solve_forces(geom: GeomStep, params: ForceParams, loads: Loads) -> Forces:
    """Solve reaction forces for one suspension step. Returns a ``Forces``."""
    b_l = geom.lca_outer
    b_u = geom.uca_outer
    b_t = geom.tie_outer
    wc = geom.wheel_center
    s_l = geom.shock_lower
    p_lf = geom.lca_front
    p_lr = geom.lca_rear
    p_uf = geom.uca_front
    p_ur = geom.uca_rear
    s_u = geom.shock_upper
    tr_i = geom.tie_inner

    u_tie = (b_t - tr_i) / np.linalg.norm(b_t - tr_i)
    u_shock = (s_u - s_l) / np.linalg.norm(s_u - s_l)
    axis_lca = (p_lr - p_lf) / np.linalg.norm(p_lr - p_lf)
    axis_uca = (p_ur - p_uf) / np.linalg.norm(p_ur - p_uf)

    f_load = loads.f
    m_load = loads.m + np.cross(loads.offset, loads.f)

    a = np.zeros((7, 7))
    b = np.zeros(7)

    # force equations
    a[0:3, 0:3] = np.eye(3)
    a[0:3, 3:6] = np.eye(3)
    a[0:3, 6] = -u_tie
    b[0:3] = -f_load

    # moment equations about wheel center
    r_u = b_u - wc
    r_l = b_l - wc
    r_t = b_t - wc
    a[3:6, 0:3] = _skew(r_u)
    a[3:6, 3:6] = _skew(r_l)
    a[3:6, 6] = _skew(r_t) @ (-u_tie)
    b[3:6] = -m_load

    # constraint: non-shock arm cannot resist moment about its pivot axis
    if params.shock_mount_lca == 1:
        vec_const = np.cross(axis_uca, b_u - p_uf)
        a[6, 0:3] = vec_const
    else:
        vec_const = np.cross(axis_lca, b_l - p_lf)
        a[6, 3:6] = vec_const
    b[6] = 0.0

    x = np.linalg.solve(a, b)
    f_u = x[0:3]
    f_l = x[3:6]
    f_tie = float(x[6])

    # shock force from arm moment balance
    if params.shock_mount_lca == 1:
        f_react_on_arm = -f_l
        r_ball = b_l - p_lf
        r_shock_mount = s_l - p_lf
        axis = axis_lca
    else:
        f_react_on_arm = -f_u
        r_ball = b_u - p_uf
        r_shock_mount = s_l - p_uf
        axis = axis_uca

    m_ball_axis = float(np.dot(axis, np.cross(r_ball, f_react_on_arm)))
    term_shock = float(np.dot(axis, np.cross(r_shock_mount, -u_shock)))
    f_shock = -m_ball_axis / term_shock

    # ---- chassis pivot forces (LCA) ----
    f_l_react = -f_l
    f_s_vec = f_shock * (-u_shock) if params.shock_mount_lca == 1 else np.zeros(3)
    f_ext_l = f_l_react + f_s_vec
    m_ext_l = np.cross(b_l - p_lf, f_l_react) + np.cross(s_l - p_lf, f_s_vec)
    r_l_axis = p_lr - p_lf
    f_ext_l_axial = float(np.dot(f_ext_l, axis_lca)) * axis_lca
    f_ext_l_radial = f_ext_l - f_ext_l_axial
    f_lr_radial = np.cross(r_l_axis, m_ext_l) / np.linalg.norm(r_l_axis) ** 2
    f_lf_radial = -f_ext_l_radial - f_lr_radial
    f_lca_front = -0.5 * f_ext_l_axial + f_lf_radial
    f_lca_rear = -0.5 * f_ext_l_axial + f_lr_radial

    # ---- chassis pivot forces (UCA) ----
    f_u_react = -f_u
    f_s_vec_u = f_shock * (-u_shock) if params.shock_mount_lca == 0 else np.zeros(3)
    f_ext_u = f_u_react + f_s_vec_u
    m_ext_u = np.cross(b_u - p_uf, f_u_react) + np.cross(s_l - p_uf, f_s_vec_u)
    r_u_axis = p_ur - p_uf
    f_ext_u_axial = float(np.dot(f_ext_u, axis_uca)) * axis_uca
    f_ext_u_radial = f_ext_u - f_ext_u_axial
    f_ur_radial = np.cross(r_u_axis, m_ext_u) / np.linalg.norm(r_u_axis) ** 2
    f_uf_radial = -f_ext_u_radial - f_ur_radial
    f_uca_front = -0.5 * f_ext_u_axial + f_uf_radial
    f_uca_rear = -0.5 * f_ext_u_axial + f_ur_radial

    # ---- equilibrium check (same tolerance as MATLAB) ----
    f_sum = f_u + f_l + f_tie * (-u_tie) + f_load
    m_sum = (
        np.cross(b_u - wc, f_u)
        + np.cross(b_l - wc, f_l)
        + np.cross(b_t - wc, f_tie * (-u_tie))
        + m_load
    )
    if np.linalg.norm(f_sum) > 1e-3 or np.linalg.norm(m_sum) > 1e-3:
        import warnings

        warnings.warn(
            f"Equilibrium NOT satisfied! Res_F: {np.linalg.norm(f_sum):.4f}, "
            f"Res_M: {np.linalg.norm(m_sum):.4f}"
        )

    return Forces(
        f_uca_outer=f_u,
        f_lca_outer=f_l,
        f_tie=f_tie,
        f_shock=f_shock,
        f_lca_front=f_lca_front,
        f_lca_rear=f_lca_rear,
        f_uca_front=f_uca_front,
        f_uca_rear=f_uca_rear,
    )
