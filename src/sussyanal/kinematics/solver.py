"""Kinematic sweep solver and results container.

Port of the solver loop + post-processing in ``SusAnalysis/SussyAnal/sussy_steer.m``
(sections 3-4). Array shapes match MATLAB: scalars are ``(nSteer, nShock)`` and
3-D points are ``(3, nSteer, nShock)``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry import Config, Geometry, SuspensionModel


@dataclass
class KinematicResults:
    config: Config
    geometry: Geometry
    n_shock_steps: int
    n_steer_steps: int
    is_2d: bool

    shock_travel_axis: np.ndarray  # (nShock,)
    rack_travel_axis: np.ndarray   # (nSteer,)
    shock_travel: np.ndarray       # (nSteer, nShock)
    rack_travel: np.ndarray        # (nSteer, nShock)

    lca_outer: np.ndarray
    uca_outer: np.ndarray
    tie_outer: np.ndarray
    shock_lower: np.ndarray
    wheel_spindle: np.ndarray
    wheel_center: np.ndarray
    hub_axis: np.ndarray
    kp_ground: np.ndarray
    contact_patch: np.ndarray

    wheel_travel: np.ndarray
    camber: np.ndarray
    toe: np.ndarray
    caster: np.ndarray
    kpi: np.ndarray
    scrub: np.ndarray
    trail: np.ndarray
    ground_z: np.ndarray
    motion_ratio: np.ndarray
    wheel_rate: np.ndarray
    track_change: np.ndarray
    lca_angle: np.ndarray
    uca_angle: np.ndarray
    lca_angle_abs: np.ndarray
    uca_angle_abs: np.ndarray

    steering_arm_length: float
    ackermann: float | None = None
    axle_outer: np.ndarray | None = None
    plunge: np.ndarray | None = None
    cv_in: np.ndarray | None = None
    cv_out: np.ndarray | None = None


def _cotd(deg: float) -> float:
    return 1.0 / math.tan(math.radians(deg))


def _acotd(x: float) -> float:
    return math.degrees(math.atan2(1.0, x))


def solve_sweep(
    model: SuspensionModel,
    n_shock_steps: int = 100,
    n_steer_steps: int | None = None,
    progress: bool = False,
) -> KinematicResults:
    """Run the shock x steer sweep for ``model``."""
    cfg = model.config
    shock = model.shock

    if n_steer_steps is None:
        n_steer_steps = 21 if cfg.steer_sweep > 0.001 else 1

    shock_min = shock.length_init - abs(cfg.bump)
    shock_max = shock.length_init + abs(cfg.droop)
    shock_sweep = np.linspace(shock_max, shock_min, n_shock_steps)
    steer_sweep = np.linspace(-cfg.steer_sweep, cfg.steer_sweep, n_steer_steps)

    shock_travel_axis = shock_sweep - shock.length_init
    rack_travel_axis = steer_sweep

    ns, nt = n_shock_steps, n_steer_steps
    p3 = (3, nt, ns)

    lca_outer = np.empty(p3)
    uca_outer = np.empty(p3)
    tie_outer = np.empty(p3)
    shock_lower = np.empty(p3)
    wheel_spindle = np.empty(p3)
    wheel_center = np.empty(p3)
    hub_axis = np.empty(p3)
    kp_ground = np.full(p3, np.nan)
    contact_patch = np.empty(p3)

    wheel_travel = np.empty((nt, ns))
    camber = np.empty((nt, ns))
    toe = np.empty((nt, ns))
    caster = np.empty((nt, ns))
    kpi = np.empty((nt, ns))
    scrub = np.empty((nt, ns))
    trail = np.empty((nt, ns))
    ground_z = np.empty((nt, ns))
    lca_angle = np.empty((nt, ns))
    uca_angle = np.empty((nt, ns))

    has_axle = model.has_axle
    axle_outer = np.empty(p3) if has_axle else None
    plunge = np.empty((nt, ns)) if has_axle else None
    cv_in = np.empty((nt, ns)) if has_axle else None
    cv_out = np.empty((nt, ns)) if has_axle else None

    for i in range(ns):
        shock_len = shock_sweep[i]
        for j in range(nt):
            rack_pos = model.tierod.inner_static + np.array([0.0, steer_sweep[j], 0.0])
            s = model.solve_step(shock_len, rack_pos)

            lca_outer[:, j, i] = s.lca_outer
            uca_outer[:, j, i] = s.uca_outer
            tie_outer[:, j, i] = s.tie_outer
            shock_lower[:, j, i] = s.shock_lower
            wheel_spindle[:, j, i] = s.spindle
            wheel_center[:, j, i] = s.wheel_center
            hub_axis[:, j, i] = s.hub_axis
            kp_ground[:, j, i] = s.kp_ground
            contact_patch[:, j, i] = s.contact_patch

            wheel_travel[j, i] = s.wheel_center[2] - model.upright.wheel_center_init[2]
            camber[j, i] = s.camber
            toe[j, i] = s.toe
            caster[j, i] = s.caster
            kpi[j, i] = s.kpi
            scrub[j, i] = s.scrub
            trail[j, i] = s.trail
            ground_z[j, i] = s.ground_z
            lca_angle[j, i] = s.lca_angle
            uca_angle[j, i] = s.uca_angle

            if has_axle:
                axle_outer[:, j, i] = s.axle_outer
                plunge[j, i] = s.plunge
                cv_in[j, i] = s.cv_in
                cv_out[j, i] = s.cv_out
        if progress and (i + 1) % 50 == 0:
            print(f"  shock step {i + 1}/{ns}")

    # ---- post-processing (sussy_steer.m) ----
    mid_shock_idx = int(np.argmin(np.abs(shock_sweep - shock.length_init)))
    mid_steer_idx = math.ceil(nt / 2) - 1

    static_toe = toe[mid_steer_idx, mid_shock_idx]
    toe = toe - static_toe

    lca_angle_abs = lca_angle.copy()
    lca_angle = lca_angle - lca_angle[mid_steer_idx, mid_shock_idx]
    uca_angle_abs = uca_angle.copy()
    uca_angle = uca_angle - uca_angle[mid_steer_idx, mid_shock_idx]

    d_shock = np.gradient(shock_travel_axis)
    motion_ratio = np.gradient(wheel_travel, axis=1) / d_shock
    wheel_rate = motion_ratio**2

    track_change = 2.0 * (wheel_center[1, :, :] - model.upright.wheel_center_init[1])

    # steering arm length
    kp_vec_static = model.uca.outer_init - model.lca.outer_init
    steer_arm_vec = model.upright.tie_init - model.lca.outer_init
    steering_arm_length = float(
        np.linalg.norm(np.cross(steer_arm_vec, kp_vec_static)) / np.linalg.norm(kp_vec_static)
    )

    # Ackermann percentage
    ackermann: float | None = None
    if cfg.wheelbase > 1 and cfg.steer_sweep > 0:
        toe_sweep = toe[:, mid_shock_idx]
        delta_inner = max(abs(float(np.max(toe_sweep))), abs(float(np.min(toe_sweep))))
        delta_outer = min(abs(float(np.max(toe_sweep))), abs(float(np.min(toe_sweep))))
        if delta_inner > 0.1:
            cot_do_ideal = _cotd(delta_inner) + (model.track_width / cfg.wheelbase)
            delta_outer_ideal = _acotd(cot_do_ideal)
            denom = delta_inner - delta_outer_ideal
            if abs(denom) > 1e-4:
                ackermann = (delta_inner - delta_outer) / denom * 100.0

    return KinematicResults(
        config=cfg,
        geometry=model.geometry,
        n_shock_steps=ns,
        n_steer_steps=nt,
        is_2d=nt > 1,
        shock_travel_axis=shock_travel_axis,
        rack_travel_axis=rack_travel_axis,
        shock_travel=np.broadcast_to(shock_travel_axis, (nt, ns)),
        rack_travel=np.broadcast_to(rack_travel_axis[:, None], (nt, ns)),
        lca_outer=lca_outer,
        uca_outer=uca_outer,
        tie_outer=tie_outer,
        shock_lower=shock_lower,
        wheel_spindle=wheel_spindle,
        wheel_center=wheel_center,
        hub_axis=hub_axis,
        kp_ground=kp_ground,
        contact_patch=contact_patch,
        wheel_travel=wheel_travel,
        camber=camber,
        toe=toe,
        caster=caster,
        kpi=kpi,
        scrub=scrub,
        trail=trail,
        ground_z=ground_z,
        motion_ratio=motion_ratio,
        wheel_rate=wheel_rate,
        track_change=track_change,
        lca_angle=lca_angle,
        uca_angle=uca_angle,
        lca_angle_abs=lca_angle_abs,
        uca_angle_abs=uca_angle_abs,
        steering_arm_length=steering_arm_length,
        ackermann=ackermann,
        axle_outer=axle_outer,
        plunge=plunge,
        cv_in=cv_in,
        cv_out=cv_out,
    )
