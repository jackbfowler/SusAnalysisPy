"""Suspension geometry analyzer with steering sweep (port of ``sussy_steer.m``)."""
from __future__ import annotations

from ..geometry import SuspensionModel
from ..io import parse_csv
from .solver import KinematicResults, solve_sweep


def analyze(
    csv,
    n_shock_steps: int = 100,
    n_steer_steps: int | None = None,
    progress: bool = False,
) -> KinematicResults:
    """Parse ``csv``, build the model, and run the shock x steer sweep.

    The configuration is taken from the hardpoint set's own CSV.
    """
    data = parse_csv(csv)
    config = data.config
    model = SuspensionModel.from_data(data)
    results = solve_sweep(
        model, n_shock_steps=n_shock_steps, n_steer_steps=n_steer_steps, progress=progress
    )

    mount = "Lower Control Arm" if config.shock_mount_lca == 1 else "Upper Control Arm"
    mode = "2D Surface + Envelope" if results.is_2d else "1D Analysis"
    print(f"Config:\n  Bump: {config.bump:.2f} | Droop: {config.droop:.2f}")
    print(f"  Steer: +/- {config.steer_sweep:.2f} in | Shock Mount: {mount} | Mode: {mode}")
    print(f"Steering Arm Length: {results.steering_arm_length:.3f} in")
    if results.ackermann is not None:
        print(f"Ackermann: {results.ackermann:.1f}%")
    return results
