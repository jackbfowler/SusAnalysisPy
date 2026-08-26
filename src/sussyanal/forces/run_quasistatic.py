"""Quasistatic force analysis driver (wheel-frame -> global -> solve -> report).

Port of ``SusAnalysis/SussyForces/run_quasistatic.m``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..geometry import Config, SuspensionModel, unit
from ..io import parse_csv
from ..kinematics.solver import KinematicResults, solve_sweep
from .solve_forces import ForceParams, Forces, GeomStep, Loads, solve_forces

Vec3 = np.ndarray  # shape (3,)


@dataclass
class QuasistaticInputs:
    wft_force: Vec3 = field(default_factory=lambda: np.array([534.0, 72.0, 710.0]))        # [Fx, Fy, Fz] lbs
    wft_moment_ft_lbs: Vec3 = field(default_factory=lambda: np.array([-59.0, 6.5, 69.0]))  # [Mx, My, Mz] ft-lbs
    wft_offset_dist: float = 0.53                                                          # inches outward
    target_shock_travel: float = 0.0
    target_steer_travel: float = 0.0


@dataclass
class QuasistaticResult:
    model: SuspensionModel
    config: Config
    results: KinematicResults
    shock_idx: int
    steer_idx: int
    geom_step: GeomStep
    loads: Loads
    params: ForceParams
    forces: Forces


def run_quasistatic(csv, inputs: QuasistaticInputs | None = None) -> QuasistaticResult:
    """Run kinematics -> geometry at target step -> transform loads -> solve."""
    if inputs is None:
        inputs = QuasistaticInputs()

    data = parse_csv(csv)
    config = data.config
    model = SuspensionModel.from_data(data)

    # Kinematics (mirrors run_quasistatic.m calling sussy_steer)
    results = solve_sweep(model, n_shock_steps=100)

    shock_idx = int(np.argmin(np.abs(results.shock_travel_axis - inputs.target_shock_travel)))
    steer_idx = int(np.argmin(np.abs(results.rack_travel_axis - inputs.target_steer_travel)))

    actual_shock = float(results.shock_travel[steer_idx, shock_idx])
    actual_steer = float(results.rack_travel[steer_idx, shock_idx])

    gs = GeomStep(
        lca_outer=results.lca_outer[:, steer_idx, shock_idx],
        uca_outer=results.uca_outer[:, steer_idx, shock_idx],
        tie_outer=results.tie_outer[:, steer_idx, shock_idx],
        shock_lower=results.shock_lower[:, steer_idx, shock_idx],
        wheel_center=results.wheel_center[:, steer_idx, shock_idx],
        hub_axis=results.hub_axis[:, steer_idx, shock_idx],
        lca_front=model.lca.front,
        lca_rear=model.lca.rear,
        uca_front=model.uca.front,
        uca_rear=model.uca.rear,
        shock_upper=model.shock.upper,
        tie_inner=model.tierod.inner_static + np.array([0.0, actual_steer, 0.0]),
    )

    # ---- wheel-frame -> global transform (run_quasistatic.m section 4) ----
    y_w = gs.hub_axis.copy()
    if gs.wheel_center[1] > 0:  # left side
        if y_w[1] < 0:
            y_w = -y_w
    else:  # right side
        if y_w[1] > 0:
            y_w = -y_w
    y_w = unit(y_w)

    x_w = np.cross(y_w, np.array([0.0, 0.0, 1.0]))
    if x_w[0] < 0:
        x_w = -x_w
    x_w = unit(x_w)

    z_w = np.cross(x_w, y_w)
    z_w = unit(z_w)

    u_wft_x = -x_w
    u_wft_y = -y_w
    u_wft_z = z_w

    f_global = (
        inputs.wft_force[0] * u_wft_x
        + inputs.wft_force[1] * u_wft_y
        + inputs.wft_force[2] * u_wft_z
    )
    m_ft_lbs = inputs.wft_moment_ft_lbs
    m_global = (
        m_ft_lbs[0] * 12.0 * u_wft_x
        + m_ft_lbs[1] * 12.0 * u_wft_y
        + m_ft_lbs[2] * 12.0 * u_wft_z
    )
    r_offset_global = inputs.wft_offset_dist * y_w

    loads = Loads(f=f_global, m=m_global, offset=r_offset_global)
    params = ForceParams(
        shock_mount_lca=config.shock_mount_lca,
        wheel_diameter=config.wheel_size,
    )

    forces = solve_forces(gs, params, loads)

    return QuasistaticResult(
        model=model,
        config=config,
        results=results,
        shock_idx=shock_idx,
        steer_idx=steer_idx,
        geom_step=gs,
        loads=loads,
        params=params,
        forces=forces,
    )


def report(result: QuasistaticResult) -> str:
    """Format a text report matching the MATLAB fprintf output."""
    f = result.forces
    loads = result.loads
    lines = [
        "=== REACTION FORCES (lbs) ===",
        f"Shock Force:     {f.f_shock:8.1f} (Compression +)",
        f"Tie Rod Force:   {f.f_tie:8.1f} (Tension +)",
        "--------------------------------",
        "Joint           X       Y       Z       Mag",
    ]
    rows = [
        ("LCA Outer", f.f_lca_outer),
        ("UCA Outer", f.f_uca_outer),
        ("LCA Front", f.f_lca_front),
        ("LCA Rear ", f.f_lca_rear),
        ("UCA Front", f.f_uca_front),
        ("UCA Rear ", f.f_uca_rear),
    ]
    for name, v in rows:
        lines.append(f"{name:<12} {v[0]:7.1f} {v[1]:7.1f} {v[2]:7.1f} {np.linalg.norm(v):9.1f}")
    return "\n".join(lines)
